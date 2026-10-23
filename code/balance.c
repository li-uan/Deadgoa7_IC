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
static uint16 g_imu0_cnt    = 0;
static uint16 g_run_ms      = 0;
static uint16 g_start_cnt   = 0;
static uint8  g_speed_div   = 0;        /* 速度环分频计数 */
static uint8  g_fault       = 0;        /* 0无 1急停 2摔倒 3堵转 4失控 5IMU掉线 */

int16 g_enc_dbg[4] = {0, 0, 0, 0};      /* TEST_MODE=6 时打印 4 路编码器 */

/* 运行时可调参数（无线串口可以改） */
static float  g_kp          = BALANCE_KP;
static float  g_kd          = BALANCE_KD;
static float  g_zero        = BALANCE_ANGLE_ZERO;
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
    g_duty = 0;
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

    g_gyro_rate += GYRO_LPF_ALPHA * (tilt_rate - g_gyro_rate);
    g_gyro_yaw  += GYRO_LPF_ALPHA * (yaw_rate  - g_gyro_yaw);

    g_angle = 0.98f * (g_angle + g_gyro_rate * CTRL_DT) + 0.02f * acc_ang;

    /* 上电前 100ms 用加速度角初始化 */
    if (g_start_cnt < 20)
    {
        g_start_cnt++;
        g_angle = acc_ang;
        g_gyro_rate = 0.0f;
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
                g_armed = 0;
                g_arm_cnt = 0;
                g_speed_i = 0.0f;
                g_fault = 1;
                motor_stop();
                return;
            }
        }
        else
        {
            g_key_cnt = 0;
        }
    }

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
            g_armed = 0;
            g_fault = 5;
            motor_stop();
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
        if (++g_fall_cnt > (FALL_HOLD_MS / CTRL_PERIOD_MS))
        {
            g_armed = 0;
            g_arm_cnt = 0;
            g_speed_i = 0.0f;
            g_fault = 2;
            motor_stop();
        }
        return;
    }
    g_fall_cnt = 0;

    /* ---------- 6. 启动判定 ---------- */
    if (!g_armed)
    {
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

        motor_stop();
        return;
    }

    /* ---------- 7. 速度环（每 SPEED_LOOP_DIV 个周期算一次） ---------- */
    encoder_read(&enc_l, &enc_r);
    g_sl = (float)enc_l;
    g_sr = (float)enc_r;

    speed_now = ((float)enc_l + (float)enc_r) * 0.5f;
    g_speed += 0.1f * (speed_now - g_speed);

    target_angle = g_zero;
#if (SPEED_LOOP_ENABLE)
    if (++g_speed_div >= SPEED_LOOP_DIV)
    {
        float speed_err;

        g_speed_div = 0;

        speed_err = g_speed - SPEED_TARGET;
        g_speed_i += g_ski * speed_err * SPEED_DT;
        g_speed_i = clampf(g_speed_i, -SPEED_I_LIMIT, SPEED_I_LIMIT);

        g_corr = g_skp * speed_err + g_speed_i;
    }
    target_angle += g_speed_sign * g_corr;
#endif
    target_angle = clampf(target_angle, g_zero - TARGET_ANGLE_MAX, g_zero + TARGET_ANGLE_MAX);
    g_target = target_angle;

    /* ---------- 8. 直立 PD ---------- */
    err = g_angle - target_angle;
    out = g_kp * err + g_kd * g_gyro_rate;

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

    g_duty = (int16)out;

    motor_output(1, (int16)clampf(out + yaw_corr, -(float)MOTOR_DUTY_MAX, (float)MOTOR_DUTY_MAX));
    motor_output(0, (int16)clampf(out - yaw_corr, -(float)MOTOR_DUTY_MAX, (float)MOTOR_DUTY_MAX));

    /* ---------- 12. 堵转 / 失控 ---------- */
    if (fabsf((float)g_duty) > (float)STALL_DUTY_TH && enc_l == 0 && enc_r == 0)
    {
        if (g_stall_cnt < 60000) g_stall_cnt++;
        if (g_stall_cnt > (STALL_TIME_MS / CTRL_PERIOD_MS))
        {
            g_armed = 0;
            g_fault = 3;
            motor_stop();
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
            g_armed = 0;
            g_fault = 4;
            motor_stop();
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
void  balance_set_kp(float v)   { g_kp = v; }
void  balance_set_kd(float v)   { g_kd = v; }
void  balance_set_zero(float v) { g_zero = v; }

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

void  balance_set_speed_sign(float v) { g_speed_sign = v; }
float balance_get_speed_sign(void)    { return g_speed_sign; }
float balance_get_target(void)        { return g_target; }

void  balance_set_speed_kp(float v)   { g_skp = v; }
void  balance_set_speed_ki(float v)   { g_ski = v; }
float balance_get_speed_kp(void)      { return g_skp; }
float balance_get_speed_ki(void)      { return g_ski; }

/* 诊断遥测接口：只读，不改变平衡控制计算。 */
float balance_get_rate(void) { return g_gyro_rate; }
uint8 balance_get_test_mode(void) { return (uint8)TEST_MODE; }
const char *balance_get_build(void) { return "DIAG-20261023-A"; }
