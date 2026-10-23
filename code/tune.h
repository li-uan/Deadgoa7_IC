#ifndef _TUNE_H_
#define _TUNE_H_

#include "zf_common_headfile.h"

/* 无线串口调参：用普通 uart 驱动 UART_2（P10.5 / P10.6），不依赖逐飞无线串口库 */
void tune_init(void);

/* 主循环里反复调用：收命令 + 回传状态 */
void tune_poll(void);

/* 主动发一帧遥测（主循环按 100ms 调一次） */
void tune_send_telemetry(void);
void tune_pulse_dump(void);

#endif
