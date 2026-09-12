/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PE5   ------> S_TIM9_CH1
     PE9   ------> S_TIM1_CH1
     PE11   ------> S_TIM1_CH2
     PE13   ------> S_TIM1_CH3
     PE14   ------> S_TIM1_CH4
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M4_IN1_Pin|M4_IN2_Pin|MOTOR_STBY_Pin|US_TRIG_Pin
                          |M3_IN1_Pin|M3_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M1_IN1_Pin|M1_IN2_Pin|M2_IN1_Pin|M2_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M4_IN1_Pin M4_IN2_Pin US_TRIG_Pin
                           M3_IN1_Pin M3_IN2_Pin */
  GPIO_InitStruct.Pin = M4_IN1_Pin|M4_IN2_Pin|US_TRIG_Pin
                          |M3_IN1_Pin|M3_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* STBY is open-drain: RESET disables the bridge; SET releases the pin so
     the driver module's on-board 10 kOhm pull-up can provide a valid 5 V high. */
  GPIO_InitStruct.Pin = MOTOR_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_STBY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : US_ECHO_Pin */
  GPIO_InitStruct.Pin = US_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(US_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_IN1_Pin M1_IN2_Pin M2_IN1_Pin M2_IN2_Pin */
  GPIO_InitStruct.Pin = M1_IN1_Pin|M1_IN2_Pin|M2_IN1_Pin|M2_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Orange Pi shutdown request: normally-closed PC5-to-GND button.
     Releasing/opening the button lets the internal pull-up assert PC5. */
  GPIO_InitStruct.Pin = OPI_SHUTDOWN_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(OPI_SHUTDOWN_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* BLE pairing request: normally-open button from PB0 to GND. */
  GPIO_InitStruct.Pin = BLE_PAIR_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BLE_PAIR_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_PWM_Pin M2_PWM_Pin M3_PWM_Pin M4_PWM_Pin */
  GPIO_InitStruct.Pin = M1_PWM_Pin|M2_PWM_Pin|M3_PWM_Pin|M4_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* MPU6050 active-high data-ready interrupt. */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /* Priority 6 may call CMSIS-RTOS2 thread flag APIs from the ISR. */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
