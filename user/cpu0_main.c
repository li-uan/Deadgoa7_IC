/*********************************************************************************************************************
* 平衡车：主循环（调试打印 + 无线串口调参）
********************************************************************************************************************/
#include "isr_config.h"
#include "zf_common_headfile.h"
#include "balance.h"
#include "tune.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    uint16 loop_cnt = 0;

    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 调试串口 UART_0

    balance_init();                 // PWM / 编码器 / IMU / 5ms 控制中断
    tune_init();                    // 无线串口 UART_2（P10.5/P10.6）

    cpu_wait_event_ready();

    while (TRUE)
    {
        tune_pulse_dump();          /* TEST_MODE=8 数据走无线口 */

        tune_poll();                // 解析上位机命令：P300 / D8 / Z2.5 / ?

        if (++loop_cnt >= 10)       // 100ms
        {
            loop_cnt = 0;
            tune_send_telemetry();  // 无线串口回传状态
            balance_debug_print();  // 调试串口打印
        }

        system_delay_ms(10);
    }
}

#pragma section all restore
