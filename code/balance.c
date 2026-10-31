#include "balance.h"
#include "balance_config.h"
#include <math.h>

#define RAD2DEG                 (57.29578f)
#define CTRL_DT                 ((float)CTRL_PERIOD_MS / 1000.0f)
#define SPEED_DT                ((float)(CTRL_PERIOD_MS * SPEED_LOOP_DIV) / 1000.0f)

/* ===================== 内部状态 ===================== */
static float g_angle        = 0.0f;     /* 互补滤波后的倾角（°） */
static float g_gyro_rate    = 0.0f;     /* 滤波后的倾角角速度（°/s） */
static float g_gyro_yaw     = 0.0f;     /* 滤波后的 yaw 角速度（°/s） */
static float g_speed        = 0.0f;     /* 左右轮平均速度（计数/5ms，低通后） */
static float g_sl           = 0.0f;     /* 左轮速度（计数/5ms） */
static float g_sr           = 0.0f;     /* 右轮速度（计数/5ms） */
static float g_corr         = 0.0f;     /* 速度环输出（°） */
static float g_speed_i      = 0.0f;     /* 速度环积分 */
static int16 g_duty         = 0;
static uint8 g_armed        = 0;
static uint16 g_arm_cnt     = 0;
static uint16 g_fall_cnt    = 0;
static uint16 g_key_cnt     = 0;
static uint16 g_stall_cnt   = 0;
static uint16 g_runaway_cnt = 0;
static uint16 g_overspeed_cnt = 0;
static uint16 g_imu0_cnt    = 0;
static uint16 g_run_ms      = 0;
static uint16 g_start_cnt   = 0;
static float  g_bias_sum   = 0.0f;   /* 零偏累加 */
static float  g_gyro_bias  = 0.0f;
static float  g_acc_ang    = 0.0f;   /* 诊断：本帧加速度计角 */
static float  g_gyro_angle = 0.0f;   /* 诊断：纯陀螺积分角 */
static float  g_acc_corr   = 0.0f;   /* 诊断：本帧加速度计修正量 */   /* 陀螺零偏（°/s） */
static uint16 g_bias_cnt   = 0;
static uint8  g_speed_div   = 0;        /* 速度环分频计数 */
static uint8  g_fault       = 0;        /* 0无 1急停 2摔倒 3堵转 4失控 5IMU掉线 6标定失败 */
static uint8  g_need_rearm  = 0;        /* 正常上电不锁；仅故障或 B 之后置 1 */        /* 故障后必须显式重新启动（串口 R） */
static uint8  g_calib_ok    = 0;        /* 零偏标定是否通过静止检查 */
static float  g_bias_min    = 0.0f;     /* 标定期内角速度最小值 */
static float  g_bias_max    = 0.0f;     /* 标定期内角速度最大值 */
static float  g_raw_out     = 0.0f;     /* 限幅前的原始 PD 请求 */
static uint16 g_innov_rej   = 0;        /* 加速度修正被连续拒绝的周期数 */

/* ---- 加速度污染诊断记录（运行时 A1 开启） ---- */
#define ACC_REC_N               (800)          /* 800 行 x 5ms = 4 秒 */
static balance_acc_row_t g_acc_rows[ACC_REC_N];
static uint16 g_acc_cnt   = 0;
static uint8  g_acc_done  = 0;
static uint8  g_acc_started = 0;   /* 检测到运动后才开始记录 */
static uint8  g_diag_mode = 0;


int16 g_enc_dbg[4] = {0, 0, 0, 0};

/* ---- TEST_MODE=8 脉冲试验 ---- */
static float  g_pulse_duty = PULSE_TEST_DUTY;   /* 本次施加的输入，可用串口 X 改 */
#if (TEST_MODE == 8)
static balance_pulse_row_t g_pulse[PULSE_REC_N];
static uint16 g_pulse_cnt  = 0;
static uint8  g_pulse_done = 0;
#endif

/* 运行时可调参数（无线串口可以改） */
static float  g_kp          = BALANCE_KP;
static float  g_kd          = BALANCE_KD;
static float  g_zero        = BALANCE_ANGLE_ZERO;   /* 实际生效的零点（缓变后） */
static float  g_zero_req    = BALANCE_ANGLE_ZERO;   /* 串口请求的零点 */
static float  g_speed_sign  = SPEED_SIGN;
static float  g_skp         = SPEED_KP;
static float  g_ski         = SPEED_KI;
static float  g_target      = 0.0f;

/* ===================== 小工具 ===================== */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 读陀螺仪：倾角轴 + yaw 轴（已乘符号） */
static void imu_read_tilt_rate(float *tilt_rate, float *yaw_rate)
{
    imu660rc_get_gyro();

#if (IMU_TILT_AXIS == 0)
    *tilt_rate = imu660rc_gyro_transition(imu660rc_gyro_x) * (float)IMU_TILT_SIGN;
#elif (IMU_TILT_AXIS == 1)
    *tilt_rate = imu660rc_gyro_transition(imu660rc_gyro_y) * (float)IMU_TILT_SIGN;
#else
    *tilt_rate = imu660rc_gyro_transition(imu660rc_gyro_z) * (float)IMU_TILT_SIGN;
#endif

#if (IMU_YAW_AXIS == 0)
    *yaw_rate = imu660rc_gyro_transition(imu660rc_gyro_x) * (float)IMU_YAW_SIGN;
#elif (IMU_YAW_AXIS == 1)
    *yaw_rate = imu660rc_gyro_transition(imu660rc_gyro_y) * (float)IMU_YAW_SIGN;
#else
    *yaw_rate = imu660rc_gyro_transition(imu660rc_gyro_z) * (float)IMU_YAW_SIGN;
#endif
}

/* 用加速度算绝对倾角
 * 绕 X 轴转 -> 重力在 Y-Z 平面 -> atan2(ay, az)
 * 绕 Y 轴转 -> 重力在 X-Z 平面 -> atan2(ax, az)
 * 绕 Z 轴转 -> 重力在 X-Y 平面 -> atan2(ax, ay)  */
static float imu_acc_angle(void)
{
    float ax, ay, az;

    imu660rc_get_acc();
    ax = imu660rc_acc_transition(imu660rc_acc_x);
    ay = imu660rc_acc_transition(imu660rc_acc_y);
    az = imu660rc_acc_transition(imu660rc_acc_z);

#if (IMU_TILT_AXIS == 0)
    return atan2f(ay, az) * RAD2DEG * (float)IMU_TILT_SIGN;
#elif (IMU_TILT_AXIS == 1)
    return atan2f(ax, az) * RAD2DEG * (float)IMU_TILT_SIGN;
#else
    return atan2f(ax, ay) * RAD2DEG * (float)IMU_TILT_SIGN;
#endif
}

static void encoder_read(int16 *l, int16 *r)
{
    int16 raw_l = encoder_get_count(ENC_L);
    int16 raw_r = encoder_get_count(ENC_R);
    encoder_clear_count(ENC_L);
    encoder_clear_count(ENC_R);

    *l = (ENC_L_INVERT) ? (int16)(-raw_l) : raw_l;
    *r = (ENC_R_INVERT) ? (int16)(-raw_r) : raw_r;
}

/* 输出一个电机的占空比（带方向） */
static void motor_output(uint8 is_left, int16 duty)
{
    uint8 dir_level = 0;
    uint32 mag;

    if (duty < 0)
    {
        dir_level = 1;
        duty = (int16)(-duty);
    }
    if (is_left)
    {
        if (MOTOR_L_INVERT) dir_level = (uint8)(!dir_level);
    }
    else
    {
        if (MOTOR_R_INVERT) dir_level = (uint8)(!dir_level);
    }

    mag = (uint32)clampf((float)duty, 0.0f, (float)MOTOR_DUTY_MAX);

#if (MOTOR_MODE == 0)
    if (is_left) gpio_set_level(MOTOR_L_DIR, dir_level);
    else         gpio_set_level(MOTOR_R_DIR, dir_level);

    if (is_left) pwm_set_duty(MOTOR_L_PWM, mag);
    else         pwm_set_duty(MOTOR_R_PWM, mag);
#else
    if (is_left)
    {
        pwm_set_duty(MOTOR_L_PWM,  dir_level ? 0u : mag);
        pwm_set_duty(MOTOR_L_PWM2, dir_level ? mag : 0u);
    }
    else
    {
        pwm_set_duty(MOTOR_R_PWM,  dir_level ? 0u : mag);
        pwm_set_duty(MOTOR_R_PWM2, dir_level ? mag : 0u);
    }
#endif
}

static void motor_stop(void)
{
    motor_output(1, 0);
    motor_output(0, 0);
    g_duty    = 0;
    g_raw_out = 0.0f;          /* 停机后 raw 不再是当前请求 */
}

/* 统一停机：切断输出 + 清运行状态 + 锁定，必须串口 R 才能重新使能 */
static void ctrl_stop(uint8 fault_code)
{
    motor_stop();
    g_speed      = 0.0f;
    g_speed_i    = 0.0f;
    g_corr       = 0.0f;
    g_target     = g_zero;
    g_armed      = 0U;
    g_arm_cnt    = 0U;
    g_need_rearm = 1U;
    g_fault      = fault_code;
}

/* ===================== 初始化 ===================== */
void balance_init(void)
{
#if (TEST_MODE == 6)
    /* 编码器通道扫描：4 路全部初始化，看哪一路有反应 */
    encoder_dir_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
    encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);
    encoder_dir_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);
    pit_ms_init(CCU60_CH0, CTRL_PERIOD_MS);
    return;
#endif

#if (TEST_MODE == 4)
    /* 电平/PWM 扫描：4 个脚全部配成 PWM，用 20% 占空比逐个试 */
    pwm_init(ATOM0_CH0_P21_2, 17000, 0);
    pwm_init(ATOM0_CH1_P21_3, 17000, 0);
    pwm_init(ATOM0_CH2_P21_4, 17000, 0);
    pwm_init(ATOM0_CH3_P21_5, 17000, 0);
    pit_ms_init(CCU60_CH0, CTRL_PERIOD_MS);
    return;
#endif

    /* 1. 电机 */
    pwm_init(MOTOR_L_PWM, 17000, 0);
    pwm_init(MOTOR_R_PWM, 17000, 0);
#if (MOTOR_MODE == 0)
    gpio_init(MOTOR_L_DIR, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_DIR, GPO, 0, GPO_PUSH_PULL);
#else
    pwm_init(MOTOR_L_PWM2, 17000, 0);
    pwm_init(MOTOR_R_PWM2, 17000, 0);
#endif
    motor_stop();

    /* 2. 编码器 */
    encoder_dir_init(ENC_L, ENC_L_A, ENC_L_B);
    encoder_dir_init(ENC_R, ENC_R_A, ENC_R_B);

    /* 3. IMU */
    while (imu660rc_init(IMU660RC_QUARTERNION_DISABLE) != 0)
    {
        printf("IMU660RC init failed, retry...\r\n");
        system_delay_ms(300);
    }
    printf("IMU660RC init ok\r\n");

    /* 4. 急停按键 */
    gpio_init(KEY_STOP_PIN, GPI, 0, GPI_PULL_UP);

    /* 5. 5ms 控制中断 */
    pit_ms_init(CCU60_CH0, CTRL_PERIOD_MS);
    printf("balance ready. Hold the car upright (+-3 deg) for 1s to ARM.\r\n");
}

/* ===================== 5ms 控制 ===================== */
void balance_control_isr(void)
{
    float tilt_rate, yaw_rate, acc_ang;
    float target_angle, err, out;
    float yaw_corr = 0.0f;
    int16 enc_l, enc_r;
    float speed_now;

#if (TEST_MODE == 6)
    {
        g_enc_dbg[0] = encoder_get_count(TIM2_ENCODER); encoder_clear_count(TIM2_ENCODER);
        g_enc_dbg[1] = encoder_get_count(TIM5_ENCODER); encoder_clear_count(TIM5_ENCODER);
        g_enc_dbg[2] = encoder_get_count(TIM4_ENCODER); encoder_clear_count(TIM4_ENCODER);
        g_enc_dbg[3] = encoder_get_count(TIM6_ENCODER); encoder_clear_count(TIM6_ENCODER);
        return;
    }
#endif

#if (TEST_MODE == 4)
    {
        typedef struct { uint8 mask; uint16 ticks; } test_step_t;
        static const test_step_t steps[12] =
        {
            {1U, 600U}, {0U, 400U},
            {2U, 600U}, {0U, 400U},
            {3U, 600U}, {0U, 400U},
            {4U, 600U}, {0U, 400U},
            {8U, 600U}, {0U, 400U},
            {12U, 600U}, {0U, 600U}
        };
        static uint8  idx4 = 0;
        static uint16 cnt4 = 0;
        uint8 mask;

        cnt4++;
        if (cnt4 >= steps[idx4].ticks) { cnt4 = 0; idx4 = (uint8)((idx4 + 1U) % 12U); }
        mask = steps[idx4].mask;

        pwm_set_duty(ATOM0_CH0_P21_2, (mask & 1U)  ? MOTOR_TEST_DUTY : 0U);
        pwm_set_duty(ATOM0_CH1_P21_3, (mask & 2U)  ? MOTOR_TEST_DUTY : 0U);
        pwm_set_duty(ATOM0_CH2_P21_4, (mask & 4U)  ? MOTOR_TEST_DUTY : 0U);
        pwm_set_duty(ATOM0_CH3_P21_5, (mask & 8U)  ? MOTOR_TEST_DUTY : 0U);
        return;
    }
#endif



#if (TEST_MODE == 2)
    {
        static uint16 test_t = 0;
        test_t++;
        if (test_t < 300)       { motor_output(1, MOTOR_TEST_DUTY);  motor_output(0, MOTOR_TEST_DUTY); }
        else if (test_t < 500)  { motor_stop(); }
        else if (test_t < 800)  { motor_output(1, -MOTOR_TEST_DUTY); motor_output(0, -MOTOR_TEST_DUTY); }
        else if (test_t < 1000) { motor_stop(); }
        else                    { test_t = 0; }
        return;
    }
#endif

    /* ---------- 1. 采样 ---------- */
    imu_read_tilt_rate(&tilt_rate, &yaw_rate);
    acc_ang = imu_acc_angle();

    /* 上电静止标定：采样 GYRO_BIAS_SAMPLES 次求陀螺零偏，期间电机不动 */
    if (g_bias_cnt < GYRO_BIAS_SAMPLES)
    {
        g_bias_cnt++;
        g_bias_sum += tilt_rate;
        g_gyro_bias = g_bias_sum / (float)g_bias_cnt;

        if (g_bias_cnt == 1U || tilt_rate < g_bias_min) g_bias_min = tilt_rate;
        if (g_bias_cnt == 1U || tilt_rate > g_bias_max) g_bias_max = tilt_rate;

        g_angle      = acc_ang;     /* 用加速度角兜底 */
        g_gyro_angle = acc_ang;     /* 陀螺积分角同起点 */
        g_gyro_rate = 0.0f;
        g_gyro_yaw  = 0.0f;

        if (g_bias_cnt >= GYRO_BIAS_SAMPLES)
        {
            /* 两轮车标定必须手持，极差类判据必然误伤，这里只拦截传感器级异常：
             * 平均零偏过大说明陀螺在持续输出（接线异常 / 传感器故障 / 车在匀速转动）。
             * 正常手持上电应远小于该阈值。 */
            float mean = (g_gyro_bias < 0.0f) ? -g_gyro_bias : g_gyro_bias;

            g_calib_ok = (mean < CALIB_MEAN_MAX) ? 1U : 0U;
            if (!g_calib_ok) g_fault = 6U;
        }

        motor_stop();
        return;
    }

    /* 标定未通过：停机等待重新标定（发 B）。仍然刷新角度诊断，便于判断原因 */
    if (!g_calib_ok)
    {
        g_acc_ang    = acc_ang;
        g_angle      = acc_ang;
        g_gyro_angle = acc_ang;
        g_gyro_rate  = 0.0f;
        g_acc_corr   = 0.0f;
        motor_stop();
        return;
    }

    /* 零点缓变：未使能时直接跟随；使能中限制变化率，避免在线改 Z 造成目标角阶跃 */
    if (!g_armed)
    {
        g_zero = g_zero_req;
    }
    else
    {
        float dz = g_zero_req - g_zero;
        if (dz >  ZERO_SLEW_PER_CYCLE) dz =  ZERO_SLEW_PER_CYCLE;
        if (dz < -ZERO_SLEW_PER_CYCLE) dz = -ZERO_SLEW_PER_CYCLE;
        g_zero += dz;
    }

    /* 先扣零偏，再进滤波（顺序不能反，否则零偏会被滤波吃掉） */
    tilt_rate -= g_gyro_bias;
    yaw_rate  -= g_gyro_bias;

    g_gyro_rate += GYRO_LPF_ALPHA * (tilt_rate - g_gyro_rate);
    g_gyro_yaw  += GYRO_LPF_ALPHA * (yaw_rate  - g_gyro_yaw);

    /* 互补滤波：陀螺预测 + 加速度计修正（带创新门控）
     * 平移加速时加速度计会给出虚假倾角（实测偏差可达 27°），
     * 若直接按 0.02 灌入，融合角会被拖走并形成正反馈。 */
    {
        float gyro_pred = g_angle + g_gyro_rate * CTRL_DT;
        float innov     = acc_ang - gyro_pred;

        g_acc_ang = acc_ang;

        if (innov > ACC_INNOV_GATE_DEG || innov < -ACC_INNOV_GATE_DEG)
        {
            if (g_innov_rej < 60000) g_innov_rej++;

            /* 长时间持续拒绝 → 改为慢速收敛，防止融合角永久不纠偏 */
            if (g_innov_rej > (uint16)ACC_INNOV_REJECT_MAX)
            {
                g_acc_corr = 0.002f * innov;
            }
            else
            {
                g_acc_corr = 0.0f;
            }
        }
        else
        {
            g_innov_rej = 0;
            g_acc_corr  = 0.02f * innov;
        }

        g_angle       = gyro_pred + g_acc_corr;
        g_gyro_angle += g_gyro_rate * CTRL_DT;       /* 纯陀螺积分，无修正 */
    }

    /* 加速度污染诊断：强制停机，只记录原始量，不进入任何控制律 */
    if (g_diag_mode)
    {
        /* 自动触发：出现明显转动或加速度角突变才开始记录，
         * 避免按下 A1 后静止的几秒把缓存用光 */
        if (!g_acc_started)
        {
            float dr = g_gyro_rate;
            float da = g_acc_ang - g_angle;

            if (dr > 3.0f || dr < -3.0f || da > 1.0f || da < -1.0f)
            {
                g_acc_started = 1U;
            }
            else
            {
                g_armed   = 0U;
                g_arm_cnt = 0U;
                motor_stop();
                return;
            }
        }

        if (g_acc_cnt < ACC_REC_N)
        {
            int16 e_l = encoder_get_count(ENC_L);
            int16 e_r = encoder_get_count(ENC_R);
            encoder_clear_count(ENC_L);
            encoder_clear_count(ENC_R);

            g_acc_rows[g_acc_cnt].t_ms      = (uint16)(g_acc_cnt * CTRL_PERIOD_MS);
            g_acc_rows[g_acc_cnt].ang_x100  = (int16)(g_angle * 100.0f);
            g_acc_rows[g_acc_cnt].aa_x100   = (int16)(g_acc_ang * 100.0f);
            g_acc_rows[g_acc_cnt].ga_x100   = (int16)(g_gyro_angle * 100.0f);
            g_acc_rows[g_acc_cnt].ac_x1000  = (int16)(g_acc_corr * 1000.0f);
            g_acc_rows[g_acc_cnt].rate_x10  = (int16)(g_gyro_rate * 10.0f);
            g_acc_rows[g_acc_cnt].ax_raw    = imu660rc_acc_x;
            g_acc_rows[g_acc_cnt].ay_raw    = imu660rc_acc_y;
            g_acc_rows[g_acc_cnt].az_raw    = imu660rc_acc_z;
            g_acc_rows[g_acc_cnt].sl        = e_l;
            g_acc_rows[g_acc_cnt].sr        = e_r;

            g_acc_cnt++;
            if (g_acc_cnt >= ACC_REC_N) g_acc_done = 1U;
        }

        g_armed = 0U;
        g_arm_cnt = 0U;
        motor_stop();
        return;
    }

    /* ---------- 2. 急停按键 ---------- */
    {
        uint8 key_down = (gpio_get_level(KEY_STOP_PIN) == 0) ? 1 : 0;
#if (!KEY_STOP_ACTIVE_LOW)
        key_down = (uint8)(!key_down);
#endif
        if (key_down)
        {
            if (g_key_cnt < 1000) g_key_cnt++;
            if (g_key_cnt >= 3)
            {
                ctrl_stop(1);
                return;
            }
        }
        else
        {
            g_key_cnt = 0;
        }
    }

#if (TEST_MODE == 8)
    {
        /* 单次短脉冲极性试验：每次只施加一种输入，结束后锁定 */
        static uint8  s_phase = 0;      /* 0=等待扶正 1=采样 2=已锁定 */
        static uint16 s_tick  = 0;
        static uint16 s_cnt   = 0;
        int16  e_l, e_r, u_prev, u_cmd = 0;
        uint16 t_ms;
        uint8  ph, reason = 0;

        if (s_phase == 2) { motor_stop(); return; }

        if (s_phase == 0)
        {
            motor_stop();
            if (fabsf(g_angle - g_zero) < PULSE_ARM_TOL &&
                fabsf(g_gyro_rate) < PULSE_ARM_RATE)
            {
                if (++g_arm_cnt >= (ARM_HOLD_MS / CTRL_PERIOD_MS))
                {
                    g_arm_cnt = 0;
                    s_tick    = 0;
                    s_cnt     = 0;
                    g_pulse_cnt = 0;
                    encoder_clear_count(ENC_L);     /* 记录前清零，避免累积计数 */
                    encoder_clear_count(ENC_R);
                    s_phase   = 1;
                }
            }
            else
            {
                g_arm_cnt = 0;
            }
            return;
        }

        t_ms = (uint16)(s_tick * CTRL_PERIOD_MS);

        encoder_read(&e_l, &e_r);
        g_sl = (float)e_l;
        g_sr = (float)e_r;

        u_prev = g_duty;

        if (t_ms < PULSE_PRE_MS)
        {
            ph = 0;
            u_cmd = 0;
        }
        else if (t_ms < (PULSE_PRE_MS + PULSE_ON_MS))
        {
            ph = 1;
            u_cmd = (int16)g_pulse_duty;
        }
        else if (t_ms < (PULSE_PRE_MS + PULSE_ON_MS + PULSE_POST_MS))
        {
            ph = 2;
            u_cmd = 0;
        }
        else
        {
            ph = 3;
            u_cmd = 0;
        }

        /* 中止：倾角越界 / 急停按下 */
        if (fabsf(g_angle - g_zero) > PULSE_ANGLE_LIMIT) reason = 1;
        if (g_key_cnt != 0U)                             reason = 2;
        if (reason != 0U) u_cmd = 0;

        if (s_cnt < PULSE_REC_N)
        {
            g_pulse[s_cnt].t_ms         = t_ms;
            g_pulse[s_cnt].phase        = ph;
            g_pulse[s_cnt].ang_x100     = (int16)(g_angle * 100.0f);
            g_pulse[s_cnt].rate_raw_x10 = (int16)(tilt_rate * 10.0f);
            g_pulse[s_cnt].rate_x10     = (int16)(g_gyro_rate * 10.0f);
            g_pulse[s_cnt].u_prev       = u_prev;
            g_pulse[s_cnt].u_cmd        = u_cmd;
            g_pulse[s_cnt].sl           = e_l;
            g_pulse[s_cnt].sr           = e_r;
            g_pulse[s_cnt].reason       = reason;
            g_pulse[s_cnt].acc_ang_x100     = (int16)(g_acc_ang * 100.0f);
            g_pulse[s_cnt].gyro_angle_x100  = (int16)(g_gyro_angle * 100.0f);
            g_pulse[s_cnt].acc_corr_x100    = (int16)(g_acc_corr * 100.0f);
            s_cnt++;
        }

        g_duty = u_cmd;
        motor_output(1, u_cmd);
        motor_output(0, u_cmd);

        s_tick++;

        if (reason != 0U || s_cnt >= PULSE_REC_N)
        {
            g_duty = 0;
            motor_stop();
            s_phase      = 2;       /* 锁定：不自动重新解锁 */
            g_pulse_cnt  = s_cnt;
            g_pulse_done = 1;
            g_armed      = 0;
            g_fault      = 8;
        }
        return;
    }
#endif

#if (TEST_MODE == 5)
    {
        /* 起转门槛扫描：800 -> 1100 -> 1400 -> 1700 -> 2000 -> 2500
         * 每档正向转 1s、停 0.5s，只跑一遍；复位后重新开始。
         * 看轮子从哪一档开始明显转动，那一档就是起转门槛。 */
        static uint16 t5 = 0;
        static const int16 amps[6] = {800, 1100, 1400, 1700, 2000, 2500};
        const uint16 on_ticks  = (uint16)(3000U / CTRL_PERIOD_MS);   /* 3s，方便观察 */
        const uint16 off_ticks = (uint16)(1000U / CTRL_PERIOD_MS);   /* 停 1s */
        const uint16 step      = (uint16)(on_ticks + off_ticks);
        const uint16 total     = (uint16)(6U * step);
        uint8  idx;
        uint16 phase;
        int16  l5, r5;
        int16  test_duty = 0;

        encoder_read(&l5, &r5);
        g_sl = (float)l5;
        g_sr = (float)r5;
        g_speed += 0.1f * (((float)l5 + (float)r5) * 0.5f - g_speed);

        /* 急停：首次检测到按下就锁存，复位后才重新测试 */
        if (g_key_cnt != 0U) g_fault = 1;
        if (g_fault != 0U)
        {
            motor_stop();
            return;
        }

        if (t5 < total)
        {
            idx   = (uint8)(t5 / step);
            phase = (uint16)(t5 % step);
            if (phase < on_ticks) test_duty = amps[idx];
            t5++;
        }

        g_duty = test_duty;
        motor_output(1, test_duty);
        motor_output(0, test_duty);
        return;
    }
#endif

    /* ---------- 3. 测试模式：只读传感器 ---------- */
#if (TEST_MODE == 1)
    {
        int16 tl, tr;
        float tnow;
        encoder_read(&tl, &tr);
        g_sl = (float)tl;
        g_sr = (float)tr;
        tnow = ((float)tl + (float)tr) * 0.5f;
        g_speed += 0.1f * (tnow - g_speed);
    }
    motor_stop();
    return;
#endif

    /* ---------- 3b. 纯计算验证：电机不转，但速度环照算 ---------- */
#if (TEST_MODE == 7)
    {
        int16 tl, tr;
        float tnow, serr, tgt;

        encoder_read(&tl, &tr);
        g_sl = (float)tl;
        g_sr = (float)tr;
        tnow = ((float)tl + (float)tr) * 0.5f;
        g_speed += 0.1f * (tnow - g_speed);

        serr = g_speed - SPEED_TARGET;
        g_corr = g_skp * serr;                 /* KI 固定 0，只看 P */
        tgt = g_zero + g_speed_sign * g_corr;
        tgt = clampf(tgt, g_zero - TARGET_ANGLE_MAX, g_zero + TARGET_ANGLE_MAX);
        g_target = tgt;
    }
    motor_stop();
    return;
#endif

    /* ---------- 4. IMU 掉线 ---------- */
    if (imu660rc_gyro_x == 0 && imu660rc_gyro_y == 0 && imu660rc_gyro_z == 0 &&
        imu660rc_acc_x == 0 && imu660rc_acc_y == 0 && imu660rc_acc_z == 0)
    {
        if (g_imu0_cnt < 60000) g_imu0_cnt++;
        if (g_imu0_cnt > (IMU_ZERO_TIME_MS / CTRL_PERIOD_MS))
        {
            ctrl_stop(5);
            return;
        }
    }
    else
    {
        g_imu0_cnt = 0;
    }

    /* ---------- 5. 摔倒 ---------- */
    if (fabsf(g_angle - g_zero) > FALL_ANGLE)
    {
        /* 一旦超限就停机并锁定：即使很快回到角度范围，也不允许自动恢复输出 */
        ctrl_stop(2);
        return;
    }
    g_fall_cnt = 0;

    /* ---------- 6. 启动判定 ---------- */
    if (!g_armed)
    {
        /* 故障后必须显式重新启动（串口发 R），避免松开急停就等于授权恢复驱动 */
        if (g_need_rearm)
        {
            g_arm_cnt = 0;
            g_speed   = 0.0f;
            encoder_clear_count(ENC_L);
            encoder_clear_count(ENC_R);
            motor_stop();
            return;
        }

        if (fabsf(g_angle - g_zero) < ARM_ANGLE_TOL)
        {
            if (++g_arm_cnt >= (ARM_HOLD_MS / CTRL_PERIOD_MS))
            {
                g_armed = 1;
                g_arm_cnt = 0;
                g_speed_i = 0.0f;
                g_speed_div = 0;
                g_corr = 0.0f;
                g_run_ms = 0;
                g_fault = 0;
                /* 清掉上一次运行残留的保护计数，避免刚启动就误触发 */
                g_fall_cnt = 0;
                g_stall_cnt = 0;
                g_runaway_cnt = 0;
                g_overspeed_cnt = 0;
                g_raw_out = 0.0f;
            }
        }
        else
        {
            g_arm_cnt = 0;
        }

        /* 未启动时也要清编码器计数：否则从上次 disarm 到现在的计数会累积，
         * arm 的第一帧会读到一个巨大的速度尖峰，把速度环带飞。 */
        encoder_clear_count(ENC_L);
        encoder_clear_count(ENC_R);
        g_speed = 0.0f;

        motor_stop();
        return;
    }

    /* ---------- 7. 速度环（每 SPEED_LOOP_DIV 个周期算一次） ---------- */
    encoder_read(&enc_l, &enc_r);
    g_sl = (float)enc_l;
    g_sr = (float)enc_r;

    speed_now = ((float)enc_l + (float)enc_r) * 0.5f;
    g_speed += 0.1f * (speed_now - g_speed);

    /* 超速保护：只看轮速，不依赖倾角；持续超限即停机锁定 */
    if (fabsf(g_speed) > OVERSPEED_LIMIT)
    {
        if (g_overspeed_cnt < 60000) g_overspeed_cnt++;
        if (g_overspeed_cnt > (OVERSPEED_TIME_MS / CTRL_PERIOD_MS))
        {
            ctrl_stop(7);
            return;
        }
    }
    else
    {
        g_overspeed_cnt = 0;
    }

    /* ---------- 7b. 速度环：只做 P，叠加到目标角 ---------- */
    target_angle = g_zero;
    g_corr = 0.0f;

#if (SPEED_LOOP_ENABLE)
    g_corr = g_skp * (g_speed - SPEED_TARGET);
    target_angle += g_speed_sign * g_corr;
#endif

    target_angle = clampf(target_angle, g_zero - TARGET_ANGLE_MAX, g_zero + TARGET_ANGLE_MAX);
    g_target = target_angle;

    /* ---------- 8. 直立 PD ---------- */
    err = g_angle - target_angle;
    out = g_kp * err + g_kd * g_gyro_rate;
    g_raw_out = out;            /* 记录限幅前的原始请求，便于判断是否被截断 */

    /* ---------- 9. 转向差速（默认关闭） ---------- */
#if (YAW_DAMP_ENABLE)
    yaw_corr = YAW_KP * (g_gyro_yaw - YAW_TARGET);
#endif

    /* ---------- 10. 软启动 ---------- */
    if (g_run_ms < (SOFT_START_MS / CTRL_PERIOD_MS)) g_run_ms++;
    {
        float ramp_max = (float)MOTOR_DUTY_MAX * ((float)g_run_ms * (float)CTRL_PERIOD_MS) / (float)SOFT_START_MS;
        if (ramp_max > (float)MOTOR_DUTY_MAX) ramp_max = (float)MOTOR_DUTY_MAX;
        out = clampf(out, -ramp_max, ramp_max);
    }

    /* ---------- 11. 输出 ---------- */
    out = clampf(out, -(float)MOTOR_DUTY_MAX, (float)MOTOR_DUTY_MAX);

    /* 死区补偿：小占空比电机不转，会变成"要么不动、要么猛冲" */
    if (out > 0.0f && out < (float)MOTOR_DEADZONE)          out = (float)MOTOR_DEADZONE;
    else if (out < 0.0f && out > -(float)MOTOR_DEADZONE)    out = -(float)MOTOR_DEADZONE;

#if (MOTOR_DUTY_CAP > 0)
    /* 诊断期限幅：限制最大输出，避免持续单向加速时车速过高 */
    out = clampf(out, -(float)MOTOR_DUTY_CAP, (float)MOTOR_DUTY_CAP);
#endif

    g_duty = (int16)out;

    motor_output(1, (int16)clampf(out + yaw_corr, -(float)MOTOR_DUTY_MAX, (float)MOTOR_DUTY_MAX));
    motor_output(0, (int16)clampf(out - yaw_corr, -(float)MOTOR_DUTY_MAX, (float)MOTOR_DUTY_MAX));

    /* ---------- 12. 堵转 / 失控 ---------- */
    if (fabsf((float)g_duty) > (float)STALL_DUTY_TH && enc_l == 0 && enc_r == 0)
    {
        if (g_stall_cnt < 60000) g_stall_cnt++;
        if (g_stall_cnt > (STALL_TIME_MS / CTRL_PERIOD_MS))
        {
            ctrl_stop(3);
        }
    }
    else
    {
        g_stall_cnt = 0;
    }

    if (fabsf(g_angle - g_zero) > RUNAWAY_ANGLE)
    {
        if (g_runaway_cnt < 60000) g_runaway_cnt++;
        if (g_runaway_cnt > (RUNAWAY_TIME_MS / CTRL_PERIOD_MS))
        {
            ctrl_stop(4);
        }
    }
    else
    {
        g_runaway_cnt = 0;
    }
}

/* ===================== 调试打印 ===================== */
void balance_debug_print(void)
{
    printf("ang_x100=%d rate_x10=%d gx=%d gy=%d gz=%d ax=%d ay=%d az=%d spd_x10=%d duty=%d fault=%d %s\r\n",
           (int)(g_angle * 100.0f),
           (int)(g_gyro_rate * 10.0f),
           (int)imu660rc_gyro_x,
           (int)imu660rc_gyro_y,
           (int)imu660rc_gyro_z,
           (int)imu660rc_acc_x,
           (int)imu660rc_acc_y,
           (int)imu660rc_acc_z,
           (int)(g_speed * 10.0f),
           (int)g_duty,
           (int)g_fault,
           g_armed ? "ARMED" : "disarmed");
#if (TEST_MODE == 6)
    printf("e1=%d e2=%d e3=%d e4=%d\r\n",
           (int)g_enc_dbg[0], (int)g_enc_dbg[1], (int)g_enc_dbg[2], (int)g_enc_dbg[3]);
#endif
    printf("tgt_x100=%d corr_x100=%d sl=%d sr=%d arm=%d\r\n",
           (int)(g_target * 100.0f), (int)(g_corr * 100.0f),
           (int)g_sl, (int)g_sr, (int)g_armed);
}

/* ===================== 运行时参数接口 ===================== */
void  balance_set_kp(float v)   { g_kp = clampf(v, 0.0f, 3000.0f); }
void  balance_set_kd(float v)   { g_kd = clampf(v, 0.0f, 500.0f); }
void  balance_set_zero(float v) { g_zero_req = clampf(v, -30.0f, 30.0f); }

float balance_get_kp(void)      { return g_kp; }
float balance_get_kd(void)      { return g_kd; }
float balance_get_zero(void)    { return g_zero; }

float balance_get_angle(void)   { return g_angle; }
float balance_get_speed(void)   { return g_speed; }
float balance_get_sl(void)      { return g_sl; }
float balance_get_sr(void)      { return g_sr; }
float balance_get_corr(void)    { return g_corr; }
int16 balance_get_duty(void)    { return g_duty; }
uint8 balance_is_armed(void)    { return g_armed; }
uint8 balance_get_fault(void)   { return g_fault; }

void  balance_set_speed_sign(float v) { g_speed_sign = (v < 0.0f) ? -1.0f : 1.0f; }
float balance_get_speed_sign(void)    { return g_speed_sign; }
float balance_get_target(void)        { return g_target; }

void  balance_set_speed_kp(float v)   { g_skp = clampf(v, 0.0f, 1.0f); }
void  balance_set_speed_ki(float v)   { g_ski = clampf(v, 0.0f, 1.0f); }
float balance_get_speed_kp(void)      { return g_skp; }
float balance_get_speed_ki(void)      { return g_ski; }

/* 诊断遥测接口：只读，不改变平衡控制计算。 */
float balance_get_rate(void) { return g_gyro_rate; }
float balance_get_acc_ang(void)    { return g_acc_ang; }
float balance_get_gyro_angle(void) { return g_gyro_angle; }
float balance_get_acc_corr(void)   { return g_acc_corr; }
float balance_get_gyro_bias(void)    { return g_gyro_bias; }
void  balance_restart_gyro_calib(void)
{
    /* 先切驱动，再清状态，最后重新标定：不允许带着旧使能状态回到控制 */
    g_armed      = 0;
    g_arm_cnt    = 0;
    g_run_ms     = 0;
    g_speed      = 0.0f;
    g_speed_i    = 0.0f;
    g_corr       = 0.0f;
    g_target     = g_zero;
    g_zero_req   = g_zero;
    g_raw_out    = 0.0f;
    g_fall_cnt   = 0;
    g_stall_cnt  = 0;
    g_runaway_cnt= 0;

    g_bias_cnt   = 0;
    g_bias_sum   = 0.0f;
    g_gyro_bias  = 0.0f;
    g_gyro_angle = 0.0f;
    g_gyro_rate  = 0.0f;
    g_bias_min   = 0.0f;
    g_bias_max   = 0.0f;
    g_calib_ok   = 0;

    g_innov_rej  = 0;

    g_need_rearm = 1U;          /* 标定结束后需要显式 R 才能重新使能 */
    g_fault      = 0;

    motor_stop();
}

float balance_get_raw_out(void) { return g_raw_out; }

uint8 balance_clear_rearm(void)
{
    /* 标定未完成、或标定未通过时拒绝解锁，避免把真正的故障掩盖掉 */
    if (g_bias_cnt < GYRO_BIAS_SAMPLES) return 0U;
    if (!g_calib_ok)                    return 0U;

    g_need_rearm  = 0U;
    g_fault       = 0U;
    g_arm_cnt     = 0U;
    g_fall_cnt    = 0U;
    g_stall_cnt   = 0U;
    g_runaway_cnt = 0U;
    g_overspeed_cnt = 0U;
    g_speed       = 0.0f;
    g_raw_out     = 0.0f;
    return 1U;
}
void  balance_set_diag(uint8 mode)
{
    g_armed   = 0U;
    g_arm_cnt = 0U;
    motor_stop();

    g_acc_cnt     = 0;
    g_acc_done    = 0;
    g_acc_started = 0;
    g_diag_mode   = (mode != 0U) ? 1U : 0U;
}

uint8  balance_diag_is_done(void)      { return g_acc_done; }
uint16 balance_acc_row_count(void)     { return g_acc_cnt; }
const balance_acc_row_t *balance_acc_rows(void) { return g_acc_rows; }

void   balance_acc_clear(void)
{
    g_acc_cnt  = 0;
    g_acc_done = 0;
}

uint8 balance_get_test_mode(void) { return (uint8)TEST_MODE; }
const char *balance_get_build(void) { return "DIAG-20261031-C"; }

/* ===================== TEST_MODE=8 数据导出 ===================== */
void  balance_set_pulse_duty(float v) { g_pulse_duty = v; }
float balance_get_pulse_duty(void)    { return g_pulse_duty; }

#if (TEST_MODE == 8)
uint8  balance_pulse_is_done(void)    { return g_pulse_done; }
uint16 balance_pulse_row_count(void)  { return g_pulse_cnt; }
const balance_pulse_row_t *balance_pulse_rows(void) { return g_pulse; }
void   balance_pulse_clear(void)      { g_pulse_done = 0; }
void   balance_pulse_dump(void)       { }
#else
uint8  balance_pulse_is_done(void)    { return 0; }
uint16 balance_pulse_row_count(void)  { return 0; }
const balance_pulse_row_t *balance_pulse_rows(void) { return 0; }
void   balance_pulse_clear(void)      { }
void   balance_pulse_dump(void)       { }
#endif
