/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
uint8_t CAR_BootWasExternalReset(void);

/* USER CODE BEGIN EFP */

void Emergency_MotorStop(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M4_IN1_Pin GPIO_PIN_2
#define M4_IN1_GPIO_Port GPIOE
#define M4_IN2_Pin GPIO_PIN_3
#define M4_IN2_GPIO_Port GPIOE
#define MOTOR_STBY_Pin GPIO_PIN_4
#define MOTOR_STBY_GPIO_Port GPIOE
#define US_ECHO_Pin GPIO_PIN_5
#define US_ECHO_GPIO_Port GPIOE
#define US_TRIG_Pin GPIO_PIN_6
#define US_TRIG_GPIO_Port GPIOE
#define M1_IN1_Pin GPIO_PIN_0
#define M1_IN1_GPIO_Port GPIOC
#define M1_IN2_Pin GPIO_PIN_1
#define M1_IN2_GPIO_Port GPIOC
#define M2_IN1_Pin GPIO_PIN_2
#define M2_IN1_GPIO_Port GPIOC
#define M2_IN2_Pin GPIO_PIN_3
#define M2_IN2_GPIO_Port GPIOC
#define OPI_SHUTDOWN_BUTTON_Pin GPIO_PIN_5
#define OPI_SHUTDOWN_BUTTON_GPIO_Port GPIOC
#define BLE_PAIR_BUTTON_Pin GPIO_PIN_0
#define BLE_PAIR_BUTTON_GPIO_Port GPIOB
#define ENC1_A_Pin GPIO_PIN_0
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_1
#define ENC1_B_GPIO_Port GPIOA
#define OPI_TX_Pin GPIO_PIN_2
#define OPI_TX_GPIO_Port GPIOA
#define OPI_RX_Pin GPIO_PIN_3
#define OPI_RX_GPIO_Port GPIOA
#define DRV_ADC_Pin GPIO_PIN_4
#define DRV_ADC_GPIO_Port GPIOA
#define ENC2_A_Pin GPIO_PIN_6
#define ENC2_A_GPIO_Port GPIOA
#define ENC2_B_Pin GPIO_PIN_7
#define ENC2_B_GPIO_Port GPIOA
#define M1_PWM_Pin GPIO_PIN_9
#define M1_PWM_GPIO_Port GPIOE
#define M2_PWM_Pin GPIO_PIN_11
#define M2_PWM_GPIO_Port GPIOE
#define M3_PWM_Pin GPIO_PIN_13
#define M3_PWM_GPIO_Port GPIOE
#define M4_PWM_Pin GPIO_PIN_14
#define M4_PWM_GPIO_Port GPIOE
#define ENC4_A_Pin GPIO_PIN_6
#define ENC4_A_GPIO_Port GPIOC
#define ENC4_B_Pin GPIO_PIN_7
#define ENC4_B_GPIO_Port GPIOC
#define DEBUG_TX_Pin GPIO_PIN_10
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_11
#define DEBUG_RX_GPIO_Port GPIOB
#define ENC3_A_Pin GPIO_PIN_6
#define ENC3_A_GPIO_Port GPIOB
#define ENC3_B_Pin GPIO_PIN_7
#define ENC3_B_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_5
#define MPU_INT_GPIO_Port GPIOB
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB
#define M3_IN1_Pin GPIO_PIN_0
#define M3_IN1_GPIO_Port GPIOE
#define M3_IN2_Pin GPIO_PIN_1
#define M3_IN2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
