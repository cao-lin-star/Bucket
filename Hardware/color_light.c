#include "color_light.h"
#include "tim.h"

static uint8_t color_light_r;
static uint8_t color_light_g;
static uint8_t color_light_b;
static uint8_t color_light_w;
static uint8_t color_light_initialized;

static uint8_t ColorLight_ClampValue(uint8_t value)
{
  if (value > 100U)
  {
    return 100U;
  }
  return value;
}

static uint32_t ColorLight_DutyToPulse(uint8_t duty_percent)
{
  /* 将 0~255 亮度线性换算为定时器比较值，并限制在 ARR 范围内。 */
  uint32_t period;
  uint32_t pulse;

  period = htim4.Init.Period;
  pulse = ((period + 1U) * (uint32_t)duty_percent) / 100U;
  if (pulse > period)
  {
    pulse = period;
  }
  return pulse;
}

static void ColorLight_SetTimerChannel(uint32_t channel, uint8_t value)
{
  __HAL_TIM_SET_COMPARE(&htim4, channel, ColorLight_DutyToPulse(value));
}

static void ColorLight_Apply(void)
{
  /* 集中刷新四个比较寄存器，使 RGBW 颜色变化尽可能同步。 */
  if (color_light_initialized == 0U)
  {
    return;
  }

  ColorLight_SetTimerChannel(TIM_CHANNEL_1, color_light_r);
  ColorLight_SetTimerChannel(TIM_CHANNEL_2, color_light_g);
  ColorLight_SetTimerChannel(TIM_CHANNEL_3, color_light_b);
  ColorLight_SetTimerChannel(TIM_CHANNEL_4, color_light_w);
}

void ColorLight_Init(void)
{
  color_light_initialized = 1U;

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  ColorLight_Apply();
}

void ColorLight_DeInit(void)
{
  ColorLight_Off();

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
  color_light_initialized = 0U;
}

void ColorLight_Off(void)
{
  ColorLight_SetRgbw(0U, 0U, 0U, 0U);
}

void ColorLight_SetRgb(uint8_t r, uint8_t g, uint8_t b)
{
  ColorLight_SetRgbw(r, g, b, 0U);
}

void ColorLight_SetRgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
  color_light_r = ColorLight_ClampValue(r);
  color_light_g = ColorLight_ClampValue(g);
  color_light_b = ColorLight_ClampValue(b);
  color_light_w = ColorLight_ClampValue(w);
  ColorLight_Apply();
}     

void ColorLight_SetChannel(uint8_t channel, uint8_t value)
{
  /* 单通道修改后立即统一下发；非法通道不改变当前灯光。 */
  value = ColorLight_ClampValue(value);

  switch (channel)
  {
    case COLOR_LIGHT_CHANNEL_R:
      color_light_r = value;
      break;

    case COLOR_LIGHT_CHANNEL_G:
      color_light_g = value;
      break;

    case COLOR_LIGHT_CHANNEL_B:
      color_light_b = value;
      break;

    case COLOR_LIGHT_CHANNEL_W:
      color_light_w = value;
      break;

    default: 
      return;
  }

  ColorLight_Apply();
}
 
void ColorLight_GetRgbw(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w)
{
  if (r != NULL)
  {
    *r = color_light_r;
  }
  if (g != NULL)
  {
    *g = color_light_g;
  }
  if (b != NULL)
  {
    *b = color_light_b;
  }
  if (w != NULL)
  { 
    *w = color_light_w;
  }
}
