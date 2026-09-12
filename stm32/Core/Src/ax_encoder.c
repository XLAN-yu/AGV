#include "ax_encoder.h"
#include "tim.h"

#if (AX_ENCODER_COUNTS_PER_WHEEL_REV == 0U)
#error "AX_ENCODER_COUNTS_PER_WHEEL_REV must be greater than zero"
#endif

#if ((AX_ENCODER_M1_FORWARD_SIGN != 1) && (AX_ENCODER_M1_FORWARD_SIGN != -1)) || \
    ((AX_ENCODER_M2_FORWARD_SIGN != 1) && (AX_ENCODER_M2_FORWARD_SIGN != -1)) || \
    ((AX_ENCODER_M3_FORWARD_SIGN != 1) && (AX_ENCODER_M3_FORWARD_SIGN != -1)) || \
    ((AX_ENCODER_M4_FORWARD_SIGN != 1) && (AX_ENCODER_M4_FORWARD_SIGN != -1))
#error "Each AX_ENCODER_Mx_FORWARD_SIGN must be 1 or -1"
#endif

static void AX_ENCODER_Init(TIM_HandleTypeDef *htim)
{
  __HAL_TIM_SET_COUNTER(htim, 0U);
  if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
}

static int16_t AX_ENCODER_GetCounter(TIM_HandleTypeDef *htim)
{
  return (int16_t)__HAL_TIM_GET_COUNTER(htim);
}

static void AX_ENCODER_SetCounter(TIM_HandleTypeDef *htim, int16_t count)
{
  __HAL_TIM_SET_COUNTER(htim, (uint16_t)count);
}

void AX_ENCODER_A_Init(void)
{
  AX_ENCODER_Init(&htim2);
}

int16_t AX_ENCODER_A_GetCounter(void)
{
  return AX_ENCODER_GetCounter(&htim2);
}

void AX_ENCODER_A_SetCounter(int16_t count)
{
  AX_ENCODER_SetCounter(&htim2, count);
}

void AX_ENCODER_B_Init(void)
{
  AX_ENCODER_Init(&htim3);
}

int16_t AX_ENCODER_B_GetCounter(void)
{
  return AX_ENCODER_GetCounter(&htim3);
}

void AX_ENCODER_B_SetCounter(int16_t count)
{
  AX_ENCODER_SetCounter(&htim3, count);
}

void AX_ENCODER_C_Init(void)
{
  AX_ENCODER_Init(&htim4);
}

int16_t AX_ENCODER_C_GetCounter(void)
{
  return AX_ENCODER_GetCounter(&htim4);
}

void AX_ENCODER_C_SetCounter(int16_t count)
{
  AX_ENCODER_SetCounter(&htim4, count);
}

void AX_ENCODER_D_Init(void)
{
  AX_ENCODER_Init(&htim8);
}

int16_t AX_ENCODER_D_GetCounter(void)
{
  return AX_ENCODER_GetCounter(&htim8);
}

void AX_ENCODER_D_SetCounter(int16_t count)
{
  AX_ENCODER_SetCounter(&htim8, count);
}
