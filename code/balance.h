#ifndef _BALANCE_H_
#define _BALANCE_H_

#include "zf_common_headfile.h"

/* 初始化：PWM / 编码器 / IMU / 5ms 控制中断 */
void balance_init(void);

/* 5ms 中断里调用 */
void balance_control_isr(void);

/* 主循环里调用：串口打印调试信息（调试串口 UART_0） */
void balance_debug_print(void);

/* 运行时可调参数（供无线串口调参使用） */
void  balance_set_kp(float v);
void  balance_set_kd(float v);
void  balance_set_zero(float v);
float balance_get_kp(void);
float balance_get_kd(void);
float balance_get_zero(void);

/* 速度环运行时参数 */
void  balance_set_speed_sign(float v);
float balance_get_speed_sign(void);
float balance_get_target(void);
void  balance_set_speed_kp(float v);
void  balance_set_speed_ki(float v);
float balance_get_speed_kp(void);
float balance_get_speed_ki(void);

/* 状态读取 */
float balance_get_angle(void);
float balance_get_speed(void);      /* 左右轮平均速度 */
float balance_get_sl(void);         /* 左轮速度 */
float balance_get_sr(void);         /* 右轮速度 */
float balance_get_corr(void);       /* 速度环输出（°） */
int16 balance_get_duty(void);
uint8 balance_is_armed(void);
uint8 balance_get_fault(void);

/* 诊断遥测 */
float balance_get_rate(void);
float balance_get_acc_ang(void);
float balance_get_gyro_angle(void);
float balance_get_acc_corr(void);
float balance_get_gyro_bias(void);
void  balance_restart_gyro_calib(void);
float balance_get_raw_out(void);      /* 限幅前原始 PD 请求 */
uint8 balance_clear_rearm(void);      /* 故障后显式允许重新启动；返回 0 表示被拒绝 */

/* ---- TEST_MODE=8 脉冲试验 ---- */
void  balance_set_pulse_duty(float v);
float balance_get_pulse_duty(void);
uint8 balance_pulse_is_done(void);
void  balance_pulse_dump(void);

typedef struct {
    uint16 t_ms;
    uint8  phase;
    int16  ang_x100;
    int16  rate_raw_x10;
    int16  rate_x10;
    int16  u_prev;
    int16  u_cmd;
    int16  sl;
    int16  sr;
    uint8  reason;
    int16  acc_ang_x100;
    int16  gyro_angle_x100;
    int16  acc_corr_x100;
} balance_pulse_row_t;

uint16 balance_pulse_row_count(void);
const balance_pulse_row_t *balance_pulse_rows(void);
void   balance_pulse_clear(void);
uint8 balance_get_test_mode(void);
const char *balance_get_build(void);

#endif
