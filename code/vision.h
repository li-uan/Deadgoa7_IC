#ifndef _VISION_H_
#define _VISION_H_

#include "zf_common_headfile.h"

/* 元素标志位掩码 */
#define BIT_CROSS   0x01  // bit0 十字
#define BIT_ROUND   0x02  // bit1 环岛
#define BIT_ZEBRA   0x04  // bit2 斑马线
#define BIT_OBS     0x08  // bit3 障碍
#define BIT_SLOPE   0x10  // bit4 坡道
#define BIT_S       0x20  // bit5 S弯

/* 视觉输出给电控的结构体 */
typedef struct {
    volatile int16_t offset;     // 中线偏差 (像素)，车偏右为正，偏左为负
    volatile uint8_t elements;   // 识别到的元素位掩码
    volatile uint8_t valid;      // 图像是否有效 (1=有效, 0=全黑/丢线)
} vision_t;

extern vision_t g_vision_data;

void vision_init(void);
void vision_task(void);

#endif