#ifndef _BALANCE_CONFIG_H_
#define _BALANCE_CONFIG_H_

#include "zf_common_headfile.h"

/* =====================================================================
 * 这个文件是"和硬件相关的一切"——换板子、换接线、反了方向，只改这里。
 * 所有标了 【明天核对】 的地方，都是你现在还没确认、必须先测的。
 * ===================================================================== */

/* ---------------- 1. 电机驱动 ---------------- */
/* 依据你发的引脚表：有刷电机 PWM = P21.2 / P21.3 / P21.4 / P21.5（4 路）
 * 4 路 PWM 对应"每个电机两路 IN1/IN2"，所以 MOTOR_MODE = 1
 * 如果驱动板实际是 PH/EN（PWM + DIR），把 MOTOR_MODE 改回 0，
 * 并把 PWM2 那两路当方向脚用。                                        */
#define MOTOR_MODE              (0)

/* 左电机：P21.2 = IN1，P21.3 = IN2 */
#define MOTOR_L_PWM             (ATOM0_CH1_P21_3)
#define MOTOR_L_PWM2            (ATOM0_CH1_P21_3)

/* 右电机：P21.4 = IN1，P21.5 = IN2（左右不确定，实测反了再对调） */
#define MOTOR_R_PWM             (ATOM0_CH3_P21_5)
#define MOTOR_R_PWM2            (ATOM0_CH3_P21_5)

/* MOTOR_MODE=0 时才用到的方向脚；当前模式用不到，先占位别管 */
#define MOTOR_L_DIR             (P21_2)
#define MOTOR_R_DIR             (P21_4)

/* 实测轮子转向反了就改成 1 */
#define MOTOR_L_INVERT          (1)   /* 电机线已对调，方向需反相 */
#define MOTOR_R_INVERT          (1)   /* 电机线已对调，方向需反相 */

/* 最大输出占空比（0~10000），第一版先压到 6000，防摔 */
#define MOTOR_DUTY_MAX          (6000)
/* 诊断期输出限幅：0 = 不限制。
 * 持续单向加速时用它把车速压住，方便观察 ang/ga，不影响控制方向。 */
#define MOTOR_DUTY_CAP          (3000)

/* 电机死区：占空比低于这个值时轮子转不动，因此只要需要动就给到这个值 */
#define MOTOR_DEADZONE          (0)     /* 关闭死区补偿：旧版实测为 0，线性输出 */

/* ---------------- 2. 编码器 ---------------- */
/* 逐飞的编码器编号和引脚是绑定的，见 E07_encoder_demo */
#define ENC_L                   (TIM2_ENCODER)
#define ENC_L_A                 (TIM2_ENCODER_CH1_P33_7)
#define ENC_L_B                 (TIM2_ENCODER_CH2_P33_6)

#define ENC_R                   (TIM4_ENCODER)
#define ENC_R_A                 (TIM4_ENCODER_CH1_P02_8)
#define ENC_R_B                 (TIM4_ENCODER_CH2_P00_9)

/* 手推车前进时，如果某个轮子计数是负的，就把它改成 1 */
#define ENC_L_INVERT            (0)
#define ENC_R_INVERT            (1)

/* ---------------- 3. IMU 安装方向 ---------------- */
/* 已实测：车头朝前时，IMU 的 X 轴是左右转轴，Y 轴指车头，Z 轴朝上 */
#define IMU_TILT_AXIS           (0)
#define IMU_TILT_SIGN           (-1)    /* 实测下压时角度为负，翻过来让前倾为正 */
#define IMU_YAW_AXIS            (2)
#define IMU_YAW_SIGN            (1)

/* ---------------- 4. 控制周期 ---------------- */
#define CTRL_PERIOD_MS          (5)     /* 5ms = 200Hz，新手够用 */

/* ---------------- 5. 直立环参数（先都从 0 开始，按文档一步步加） ---------------- */
#define BALANCE_ANGLE_ZERO      (1.85f) /* 平地静止实测值（符号翻转后） */
#define BALANCE_KP              (650.0f)   /* 保持 */  /* 建议从 120 开始，每次加 40 */
#define BALANCE_KD              (50.0f)  /* 建议从 8 开始，每次加 4 */

/* 角速度低通系数（0~1，越小越平滑但越滞后） */
#define GYRO_LPF_ALPHA          (0.5f)

/* 上电静止采样的次数，用来测陀螺零偏（200 x 5ms = 1s） */
#define GYRO_BIAS_SAMPLES       (400)
/* 标定异常判定：两轮车标定必须手持，只能拦截"明显异常"，不能要求完全静止
 * 极差超过该值说明车在明显晃动（°/s） */
#define CALIB_RATE_SPREAD_MAX   (12.0f)
/* 平均零偏超过该值说明车在持续转动或传感器异常（°/s） */
#define CALIB_MEAN_MAX          (30.0f)
/* 在线改 Z 的缓变速率（°/每 5ms 周期）：0.01 -> 2°/s，禁止目标角阶跃 */
#define ZERO_SLEW_PER_CYCLE     (0.01f)
/* 超速保护：两轮平均速度超过该值并持续 OVERSPEED_TIME_MS 即停机（计数/5ms） */
#define OVERSPEED_LIMIT         (120.0f)
#define OVERSPEED_TIME_MS       (200)

/* ---- TEST_MODE=8: 单次短脉冲极性试验 ----
 * 序列(相对触发)：0~20ms 输出0 -> 20~60ms 施加选定输入 -> 60~200ms 输出0
 * 每次运行只施加一种输入，结束后锁定，需重新上电再测。 */
#define PULSE_TEST_DUTY         (800)    /* 首轮建议 +350 / -350 / 0 */
#define PULSE_PRE_MS            (20)     /* 前段零输出 */
#define PULSE_ON_MS             (40)     /* 脉冲宽度 */
#define PULSE_POST_MS           (40)     /* 撤去后观察 */
#define PULSE_REC_N             (40)     /* 缓存 40 帧 = 200ms */
#define PULSE_ANGLE_LIMIT       (6.0f)
#define PULSE_ARM_TOL          (1.5f)   /* 触发要求：离零点 1.5 度以内 */
#define PULSE_ARM_RATE         (1.0f)   /* 触发要求：角速度 8 度/秒以内（近乎静止） */   /* 倾角越界立即归零并中止 */

/* ---------------- 6. 速度环（第一版先关掉，站住之后再开） ---------------- */
#define SPEED_LOOP_ENABLE       (1)
#define SPEED_KP                (0.0f)
#define SPEED_KI                (0.0f)
#define SPEED_TARGET            (0.0f)  /* 目标速度（编码器计数/5ms） */
#define SPEED_I_LIMIT           (300.0f)

/* 速度环叠加符号：串口命令 S1 / S-1 可运行时改 */
#define SPEED_SIGN              (-1.0f)

/* 速度阻尼：输出 += SPEED_DAMP * 速度（正占空比=后驱=刹车） */
#define SPEED_DAMP              (20.0f)

/* 速度环分频：每 N 个 5ms 周期算一次（10 = 50ms） */
#define SPEED_LOOP_DIV          (10)

/* 防止暴冲：速度环输出的目标倾角最大偏离（度） */
#define TARGET_ANGLE_MAX        (2.5f)

/* ---------------- 7. 转向/差速（第一版先关掉） ---------------- */
#define YAW_DAMP_ENABLE         (0)
#define YAW_KP                  (0.0f)
#define YAW_TARGET              (0.0f)

/* ---------------- 8. 安全保护 ---------------- */
#define ARM_ANGLE_TOL           (3.0f)   /* 扶正到 ±3° 内才允许启动 */
#define ARM_HOLD_MS             (1000)   /* 持续扶正 1 秒才启动 */
#define FALL_ANGLE              (45.0f)  /* 倾角超过 45° 立即停机 */
#define FALL_HOLD_MS            (100)    /* 持续 100ms 才判定摔倒 */

/* ---------------- 9. 调试 ---------------- */
#define DEBUG_PRINT_MS          (200)    /* 串口打印间隔 */

/* ---------------- 10. 安全测试 ---------------- */
/* TEST_MODE = 0：正常平衡模式
 * TEST_MODE = 1：只读传感器，电机绝对不动（验 IMU 方向用）
 * TEST_MODE = 2：电机测试，两轮同时 正转1.5s -> 停1s -> 反转1.5s -> 停1s 循环
 *                必须把车架起来、轮子悬空！                              */
#define TEST_MODE               (0)

/* TEST_MODE=2 时的测试占空比（0~10000），1500 约等于 15% */
#define MOTOR_TEST_DUTY         (2000)

/* 测试模式允许的最大测试幅度（诊断用，别超过 4000） */
#define MOTOR_TEST_DUTY_MAX     (4000)

/* 急停按键：按下立刻停机并清空输出。P20.6 是引脚表上的按键 IO。
 * 如果实测电平相反，把 KEY_STOP_ACTIVE_LOW 改成 0。                      */
#define KEY_STOP_PIN            (P20_6)
#define KEY_STOP_ACTIVE_LOW     (1)

/* ---------------- 11. 故障保护 ---------------- */
/* 堵转：输出大于阈值、但两个编码器 300ms 内都没动 -> 停机 */
#define STALL_DUTY_TH           (1200)  /* 必须低于 MOTOR_DUTY_CAP，否则永不触发 */
#define STALL_TIME_MS           (300)

/* 失控：倾角超过 20° 持续 800ms（PD 拉不回来）-> 停机 */
#define RUNAWAY_ANGLE           (20.0f)
#define RUNAWAY_TIME_MS         (800)

/* IMU 掉线：陀螺仪和加速度计原始值全 0 持续 200ms -> 停机 */
#define IMU_ZERO_TIME_MS        (200)

/* 软启动：启动后 300ms 内输出上限从 0 线性升到 MOTOR_DUTY_MAX */
#define SOFT_START_MS           (300)

#endif
