#ifndef COLOR_LIGHT_H
#define COLOR_LIGHT_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RGBW 四通道使用软件 PWM；value 范围为 0~255。 */
#define COLOR_LIGHT_CHANNEL_R    0U
#define COLOR_LIGHT_CHANNEL_G    1U
#define COLOR_LIGHT_CHANNEL_B    2U
#define COLOR_LIGHT_CHANNEL_W    3U

void ColorLight_Init(void);
void ColorLight_DeInit(void);
void ColorLight_Off(void);
void ColorLight_SetRgb(uint8_t r, uint8_t g, uint8_t b);
/** 一次更新四通道目标值，避免上层逐通道设置产生短暂混色。 */
void ColorLight_SetRgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void ColorLight_SetChannel(uint8_t channel, uint8_t value);
/** 输出指针可分别为 NULL，只返回调用方关心的通道。 */
void ColorLight_GetRgbw(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w);

#ifdef __cplusplus
}
#endif

#endif
