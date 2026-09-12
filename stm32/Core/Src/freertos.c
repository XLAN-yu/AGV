/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "avg_protocol.h"
#include "car_config.h"
#include "mecanum_mixer.h"
#include "obstacle_guard.h"
#include "obstacle_motion_guard.h"
#include "motor_start_compensation.h"
#include "ax_motor.h"
#include "ax_encoder.h"
#include "ax_vin.h"
#include "hc_sr04.h"
#include "i2c.h"
#include "mpu6050.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  CAR_MOTION_BRAKE = 0,
  CAR_MOTION_FORWARD,
  CAR_MOTION_BACKWARD,
  CAR_MOTION_TURN_LEFT,
  CAR_MOTION_TURN_RIGHT,
  CAR_MOTION_STRAFE_LEFT,
  CAR_MOTION_STRAFE_RIGHT,
  CAR_MOTION_COAST,
  CAR_MOTION_DISABLED
} CarMotion_t;

typedef enum
{
  CAR_DRIVE_DISABLED = 0,
  CAR_DRIVE_BRAKE,
  CAR_DRIVE_COAST,
  CAR_DRIVE_CLOSED_LOOP
} CarDriveMode_t;

typedef enum
{
  CAR_MOTOR_MESSAGE_DRIVE = 0,
  CAR_MOTOR_MESSAGE_CLEAR_FAULT
} CarMotorMessageType_t;

typedef struct
{
  CarMotorMessageType_t type;
  CarDriveMode_t mode;
  int32_t target_rpm[4];
  uint8_t obstacle_retreat;
} CarMotorMessage_t;

typedef struct
{
  float integral;
  float previous_error;
  int32_t previous_target_rpm;
  uint8_t initialized;
} CarPidState_t;

typedef enum
{
  CAR_IMU_OFFLINE = 0,
  CAR_IMU_INITIALIZING,
  CAR_IMU_CALIBRATING,
  CAR_IMU_READY,
  CAR_IMU_ERROR
} CarImuState_t;

typedef struct
{
  MPU6050_Data_t data;
  CarImuState_t state;
  uint8_t who_am_i;
  uint32_t sample_tick_ms;
  uint32_t sample_count;
  uint32_t error_count;
} CarImuSnapshot_t;

typedef struct
{
  uint16_t length;
  /* The USART2 TX task serializes both binary ROVER frames and the former
     USART3 diagnostic text.  This prevents two tasks from interleaving bytes
     on PA2 while the Orange Pi is receiving a control acknowledgement. */
  uint8_t bytes[384U];
} CarUartTxMessage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAR_DEFAULT_TARGET_RPM     80U
#define CAR_TARGET_RPM_STEP        20U
#define CAR_COMMAND_IGNORED        0U
#define CAR_COMMAND_ACCEPTED       1U
#define CAR_COMMAND_HELP           2U
#define CAR_COMMAND_BUSY           3U
#define CAR_MOTOR_QUEUE_LENGTH     16U
#define CAR_PROTOCOL_TX_QUEUE_LENGTH 16U
#define CAR_UART2_TX_MESSAGE_SIZE    384U
#define CAR_TELEMETRY_PERIOD_MS    250U
#define CAR_UART_RX_TIMEOUT_MS     0U
#define CAR_UART2_DMA_BUFFER_SIZE  128U
#define CAR_UART2_STREAM_SIZE      512U
#define CAR_UART2_READ_CHUNK_SIZE  128U
#define CAR_UART2_RX_RETRY_MS      20U
#define CAR_UART2_TX_TIMEOUT_MS    100U
#define CAR_PROTOCOL_TX_DONE_FLAG  (1UL << 0U)
#define CAR_PROTOCOL_TX_ERROR_FLAG (1UL << 1U)
#define CAR_RPM_FILTER_DIVISOR     4L
#define CAR_ENCODER_DIR_MIN_RPM_X10 50L
#define CAR_ENCODER_POLARITY_CONFIRM_SAMPLES 3U
#define CAR_ENCODER_DIR_GRACE_SAMPLES 50U
#define CAR_ENCODER_DIR_FAULT_SAMPLES 10U
#define CAR_IMU_FLAG_DATA_READY    (1UL << 0U)
#define CAR_IMU_FLAG_DMA_DONE      (1UL << 1U)
#define CAR_IMU_FLAG_DMA_ERROR     (1UL << 2U)
#define CAR_ULTRASONIC_FLAG_CAPTURE_DONE (1UL << 0U)
#define CAR_IMU_DMA_TIMEOUT_MS     20U
#define CAR_IMU_RETRY_DELAY_MS     1000U
#define CAR_IMU_MAX_CONSECUTIVE_ERRORS 5U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static volatile CarDriveMode_t car_drive_mode = CAR_DRIVE_DISABLED;
static volatile int32_t car_target_rpm[4] = {0, 0, 0, 0};
static volatile int32_t car_measured_rpm_x10[4] = {0, 0, 0, 0};
static volatile int16_t car_encoder_delta[4] = {0, 0, 0, 0};
static volatile int32_t car_encoder_total[4] = {0, 0, 0, 0};
static volatile int16_t car_pwm_command[4] = {0, 0, 0, 0};
static volatile uint8_t car_encoder_fault_wheel = 0U;
static volatile uint8_t car_estop_latched = 0U;
static volatile uint8_t car_comm_timeout = 0U;
static volatile uint8_t car_uart_rx_fault = 0U;
/* Block motion at boot until the safety task obtains reliable free space. */
static volatile uint8_t car_obstacle_fault = AVG_FAULT_ULTRASONIC_UNAVAILABLE;
/* Default on at every MCU boot. Remote changes are accepted only at rest. */
static volatile uint8_t car_obstacle_guard_enabled = 1U;
static volatile uint8_t car_drive_is_obstacle_retreat = 0U;
static volatile uint8_t car_uart_rx_restart_requested = 0U;
static volatile uint8_t car_pid_reset_request = 0U;
static volatile uint32_t car_last_valid_drive_tick = 0U;
static uint32_t car_status_sequence = 0U;

static const uint16_t car_motor_start_command[4] =
{
  CAR_MOTOR_M1_START_COMMAND,
  CAR_MOTOR_M2_START_COMMAND,
  CAR_MOTOR_M3_START_COMMAND,
  CAR_MOTOR_M4_START_COMMAND
};

/*
 * The compile-time signs are only safe defaults.  The first commanded motion
 * after each boot confirms each encoder independently from its raw count
 * direction.  Direction-fault protection is enabled for a wheel only after
 * that confirmation, so an uncalibrated default cannot shut down all motors.
 */
static volatile int8_t car_encoder_forward_sign[4] =
{
  AX_ENCODER_M1_FORWARD_SIGN,
  AX_ENCODER_M2_FORWARD_SIGN,
  AX_ENCODER_M3_FORWARD_SIGN,
  AX_ENCODER_M4_FORWARD_SIGN
};
static volatile uint8_t car_encoder_polarity_locked[4] = {0U, 0U, 0U, 0U};
static CarImuSnapshot_t car_imu_snapshot = {
  .state = CAR_IMU_OFFLINE
};
static uint8_t car_uart2_dma_buffer[CAR_UART2_DMA_BUFFER_SIZE];
static volatile uint16_t car_uart2_dma_position = 0U;
static uint8_t car_uart2_stream_storage[CAR_UART2_STREAM_SIZE];
static StaticStreamBuffer_t car_uart2_stream_control;
static StreamBufferHandle_t car_uart2_stream_handle;

osMessageQueueId_t motorCommandQueueHandle;
const osMessageQueueAttr_t motorCommandQueue_attributes = {
  .name = "motorCommandQueue"
};

osMessageQueueId_t protocolTxQueueHandle;
const osMessageQueueAttr_t protocolTxQueue_attributes = {
  .name = "protocolTxQueue"
};

osMutexId_t vinMutexHandle;
const osMutexAttr_t vinMutex_attributes = {
  .name = "vinMutex",
  .attr_bits = osMutexPrioInherit
};

osThreadId_t motorControlTaskHandle;
const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t ultrasonicTaskHandle;
const osThreadAttr_t ultrasonicTask_attributes = {
  .name = "ultrasonicTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t uartRxTaskHandle;
const osThreadAttr_t uartRxTask_attributes = {
  .name = "uartRxTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t safetyTaskHandle;
const osThreadAttr_t safetyTask_attributes = {
  .name = "safetyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t protocolTxTaskHandle;
const osThreadAttr_t protocolTxTask_attributes = {
  .name = "protocolTxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 320 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE END Variables */
/* Definitions for uartCommandTask */
osThreadId_t uartCommandTaskHandle;
const osThreadAttr_t uartCommandTask_attributes = {
  .name = "uartCommandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static uint8_t CAR_QueueMotion(CarMotion_t motion, uint16_t target_rpm);
static void CAR_RefreshDebugDriveWatchdog(CarMotion_t motion);
static uint8_t CAR_HandleCommand(uint8_t command, CarMotion_t *motion,
                                 uint16_t *target_rpm);
static const char *CAR_GetMotionName(CarMotion_t motion);
static void CAR_SendText(const char *text);
static void CAR_SendStatus(CarMotion_t motion, uint16_t target_rpm);
static void CAR_SendCommandAck(uint8_t command, CarMotion_t motion,
                               uint16_t target_rpm);
static void CAR_SendCommandNack(uint8_t command);
static void CAR_SendCommandBusy(uint8_t command);
static void CAR_SendHelp(void);
static void CAR_SendTelemetry(void);
static void CAR_SendImuTelemetry(void);
static void CAR_SendUltrasonicTelemetry(void);
static void CAR_MotorControlTask(void *argument);
static void CAR_ImuTask(void *argument);
static void CAR_UltrasonicTask(void *argument);
static void CAR_PublishImu(const MPU6050_Data_t *imu, CarImuState_t state,
                           uint8_t who_am_i, uint32_t sample_tick_ms,
                           uint32_t sample_count, uint32_t error_count);
static const char *CAR_GetImuStateName(CarImuState_t state);
static int32_t CAR_FloatToScaled(float value, float scale);
static uint8_t CAR_QueueDrive(CarDriveMode_t mode,
                              const int32_t target_rpm[4],
                              uint8_t obstacle_retreat);
static uint8_t CAR_QueueClearEncoderFault(void);
static void CAR_ApplyDriveInMotorTask(CarDriveMode_t mode,
                                      const int32_t target_rpm[4],
                                      uint8_t obstacle_retreat);
static void CAR_ClearEncoderFaultInMotorTask(void);
static void CAR_UartRxTask(void *argument);
static void CAR_SafetyTask(void *argument);
static void CAR_ProtocolTxTask(void *argument);
static void CAR_TelemetryTask(void *argument);
static void CAR_HandleProtocolFrame(const AvgProtocolFrame_t *frame,
                                    void *context);
static void CAR_SendProtocolAck(uint8_t acked_type, AvgAckResult_t result,
                                uint32_t transport_sequence);
static uint8_t CAR_QueueProtocolFrame(uint8_t message_type,
                                      uint32_t transport_sequence,
                                      const uint8_t *payload,
                                      uint16_t payload_len);
static uint8_t CAR_QueueProtocolFrameWithFlags(uint8_t message_type,
                                               uint8_t flags,
                                               uint32_t transport_sequence,
                                               const uint8_t *payload,
                                               uint16_t payload_len);
static uint8_t CAR_QueueRawUart2(const uint8_t *bytes, uint16_t length);
static uint8_t CAR_QueueVelocityCommand(float linear_mps, float angular_rps);
static uint8_t CAR_QueueHolonomicVelocityCommand(float linear_mps,
                                                  float lateral_mps,
                                                  float angular_rps);
static void CAR_RequestSafeStop(void);
static uint8_t CAR_OutputIsInhibited(void);
static uint8_t CAR_HasHardwareFault(void);
static uint8_t CAR_ApplyObstacleMotionPolicy(float *linear_mps,
                                              float lateral_mps,
                                              float angular_rps);
static uint8_t CAR_GetFaultCode(void);
static uint8_t CAR_OutputsAreZero(void);
static uint8_t CAR_IsStationaryForImuBias(void);
static uint16_t CAR_ReadVinX100(void);
static void CAR_SendProtocolStatus(void);
static void CAR_ResetUart2Reception(void);

/* USER CODE END FunctionPrototypes */

void StartUartCommandTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  AX_MOTOR_Init();
  AX_ENCODER_A_Init();
  AX_ENCODER_B_Init();
  AX_ENCODER_C_Init();
  AX_ENCODER_D_Init();
  AX_VIN_Init();
  AX_MOTOR_Disable();
  car_last_valid_drive_tick = HAL_GetTick();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  vinMutexHandle = osMutexNew(&vinMutex_attributes);
  if (vinMutexHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  motorCommandQueueHandle = osMessageQueueNew(CAR_MOTOR_QUEUE_LENGTH,
                                               sizeof(CarMotorMessage_t),
                                               &motorCommandQueue_attributes);
  if (motorCommandQueueHandle == NULL)
  {
    Error_Handler();
  }

  protocolTxQueueHandle = osMessageQueueNew(CAR_PROTOCOL_TX_QUEUE_LENGTH,
                                             sizeof(CarUartTxMessage_t),
                                             &protocolTxQueue_attributes);
  if (protocolTxQueueHandle == NULL)
  {
    Error_Handler();
  }

  car_uart2_stream_handle = xStreamBufferCreateStatic(
      CAR_UART2_STREAM_SIZE, 1U, car_uart2_stream_storage,
      &car_uart2_stream_control);
  if (car_uart2_stream_handle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of uartCommandTask */
  uartCommandTaskHandle = osThreadNew(StartUartCommandTask, NULL,
                                      &uartCommandTask_attributes);
  if (uartCommandTaskHandle == NULL)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN RTOS_THREADS */
  motorControlTaskHandle = osThreadNew(CAR_MotorControlTask, NULL,
                                       &motorControlTask_attributes);
  if (motorControlTaskHandle == NULL)
  {
    Error_Handler();
  }

  imuTaskHandle = osThreadNew(CAR_ImuTask, NULL, &imuTask_attributes);
  if (imuTaskHandle == NULL)
  {
    Error_Handler();
  }

  ultrasonicTaskHandle = osThreadNew(CAR_UltrasonicTask, NULL,
                                     &ultrasonicTask_attributes);
  if (ultrasonicTaskHandle == NULL)
  {
    Error_Handler();
  }

  safetyTaskHandle = osThreadNew(CAR_SafetyTask, NULL,
                                 &safetyTask_attributes);
  if (safetyTaskHandle == NULL)
  {
    Error_Handler();
  }

  protocolTxTaskHandle = osThreadNew(CAR_ProtocolTxTask, NULL,
                                     &protocolTxTask_attributes);
  if (protocolTxTaskHandle == NULL)
  {
    Error_Handler();
  }

  uartRxTaskHandle = osThreadNew(CAR_UartRxTask, NULL,
                                 &uartRxTask_attributes);
  if (uartRxTaskHandle == NULL)
  {
    Error_Handler();
  }

  telemetryTaskHandle = osThreadNew(CAR_TelemetryTask, NULL,
                                    &telemetryTask_attributes);
  if (telemetryTaskHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartUartCommandTask */
/**
  * @brief  USART3 command-input task. Its diagnostic output is serialized on
  *         USART2/PA2 so the Orange Pi link is the only external TX path.
 *         Motor hardware is owned by
  *         CAR_MotorControlTask and is reached only through the command queue.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUartCommandTask */
void StartUartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartUartCommandTask */
  CarMotion_t motion;
  uint16_t target_rpm;
  uint8_t command;
  uint8_t command_result;
  uint32_t now;
  uint32_t last_telemetry_tick;

  (void)argument;

  motion = CAR_MOTION_DISABLED;
  target_rpm = CAR_DEFAULT_TARGET_RPM;

#if (AX_MOTOR_ENABLE_STRAFE == 1)
  CAR_SendText("AVG_V1 motor control ready (115200 8N1)\r\n"
               "F/B/L/R/S: move, Q/E: mecanum strafe, C: coast, !: disable\r\n"
               "1..9: target 20..180 RPM, K: clear encoder-direction fault\r\n"
               "Encoder polarity auto-calibrates on the first sustained motion.\r\n");
#else
  CAR_SendText("AVG_V1 motor control ready (115200 8N1)\r\n"
               "F/B/L/R/S: move, C: coast, !: disable\r\n"
               "1..9: target 20..180 RPM, K: clear encoder-direction fault\r\n"
               "Encoder polarity auto-calibrates on the first sustained motion.\r\n");
#endif
  CAR_SendText("RTOS:motorControlTask + imuTask + ultrasonicTask; diagnostic output is on USART2/PA2\r\n");
  CAR_SendText("IMU:I2C1 PB8/PB9, DATA_READY PB5, 100Hz; keep still during calibration.\r\n");
  CAR_SendText("HC-SR04:TRIG PE6, ECHO PE5/TIM9_CH1, 60ms cycle.\r\n");
  CAR_SendText("SAFE START: PWM=0 and STBY=OFF; repeat F/B/L/R within 300ms while testing.\r\n");
  CAR_SendStatus(motion, target_rpm);

  last_telemetry_tick = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {
    now = HAL_GetTick();

    if (HAL_UART_Receive(&huart3, &command, 1U,
                         CAR_UART_RX_TIMEOUT_MS) == HAL_OK)
    {
      command_result = CAR_HandleCommand(command, &motion, &target_rpm);
      if (command_result == CAR_COMMAND_ACCEPTED)
      {
        CAR_SendCommandAck(command, motion, target_rpm);
      }
      else if (command_result == CAR_COMMAND_HELP)
      {
        CAR_SendHelp();
      }
      else if (command_result == CAR_COMMAND_BUSY)
      {
        CAR_SendCommandBusy(command);
      }
      else if ((command != (uint8_t)'\r') &&
               (command != (uint8_t)'\n') &&
               (command != (uint8_t)' ') &&
               (command != (uint8_t)'\t'))
      {
        CAR_SendCommandNack(command);
      }
    }

    if ((uint32_t)(now - last_telemetry_tick) >= CAR_TELEMETRY_PERIOD_MS)
    {
      last_telemetry_tick = now;
      CAR_SendTelemetry();
    }

    osDelay(1U);
  }
  /* USER CODE END StartUartCommandTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static uint8_t CAR_OutputIsInhibited(void)
{
  uint8_t inhibited;
  uint8_t obstacle_fault;

  taskENTER_CRITICAL();
  obstacle_fault = ((car_obstacle_guard_enabled != 0U) &&
                    (car_obstacle_fault ==
                     AVG_FAULT_ULTRASONIC_UNAVAILABLE)) ? 1U : 0U;
  inhibited = ((car_estop_latched != 0U) ||
               (car_comm_timeout != 0U) ||
               (car_encoder_fault_wheel != 0U) ||
               (obstacle_fault != 0U)) ? 1U : 0U;
  taskEXIT_CRITICAL();

  if (CAR_PHYSICAL_ESTOP_RELEASED() == 0U)
  {
    inhibited = 1U;
  }
  return inhibited;
}

static uint8_t CAR_HasHardwareFault(void)
{
  uint8_t encoder_fault;
  uint8_t obstacle_fault;

  taskENTER_CRITICAL();
  encoder_fault = car_encoder_fault_wheel;
  obstacle_fault = ((car_obstacle_guard_enabled != 0U) &&
                    (car_obstacle_fault ==
                     AVG_FAULT_ULTRASONIC_UNAVAILABLE)) ? 1U : 0U;
  taskEXIT_CRITICAL();
  return ((encoder_fault != 0U) ||
          (obstacle_fault != 0U) ||
          (CAR_PHYSICAL_ESTOP_RELEASED() == 0U)) ? 1U : 0U;
}

static uint8_t CAR_ApplyObstacleMotionPolicy(float *linear_mps,
                                              float lateral_mps,
                                              float angular_rps)
{
  uint8_t guard_enabled;
  uint8_t obstacle_fault;

  taskENTER_CRITICAL();
  guard_enabled = car_obstacle_guard_enabled;
  obstacle_fault = car_obstacle_fault;
  taskEXIT_CRITICAL();
  return ObstacleMotionGuard_Apply(guard_enabled, obstacle_fault, linear_mps,
                                   lateral_mps, angular_rps);
}

static uint8_t CAR_GetFaultCode(void)
{
  uint8_t encoder_fault;
  uint8_t comm_timeout;
  uint8_t uart_fault;
  uint8_t obstacle_fault;

  if (CAR_PHYSICAL_ESTOP_RELEASED() == 0U)
  {
    return AVG_FAULT_PHYSICAL_ESTOP;
  }

  taskENTER_CRITICAL();
  encoder_fault = car_encoder_fault_wheel;
  comm_timeout = car_comm_timeout;
  uart_fault = car_uart_rx_fault;
  obstacle_fault = ((car_obstacle_guard_enabled != 0U) &&
                    (car_obstacle_fault != AVG_FAULT_NONE)) ? 1U : 0U;
  taskEXIT_CRITICAL();

  if ((encoder_fault >= 1U) && (encoder_fault <= 4U))
  {
    return (uint8_t)(AVG_FAULT_ENCODER_FRONT_LEFT + encoder_fault - 1U);
  }
  if (obstacle_fault != 0U)
  {
    return car_obstacle_fault;
  }
  if (comm_timeout != 0U)
  {
    return AVG_FAULT_COMM_TIMEOUT;
  }
  if (uart_fault != 0U)
  {
    return AVG_FAULT_UART_RX;
  }
  return AVG_FAULT_NONE;
}

static uint8_t CAR_OutputsAreZero(void)
{
  uint8_t index;
  uint8_t are_zero;

  are_zero = 1U;
  taskENTER_CRITICAL();
  if (car_drive_mode != CAR_DRIVE_DISABLED)
  {
    are_zero = 0U;
  }
  for (index = 0U; index < 4U; ++index)
  {
    if ((car_pwm_command[index] != 0) || (car_target_rpm[index] != 0))
    {
      are_zero = 0U;
    }
  }
  taskEXIT_CRITICAL();
  return are_zero;
}

static uint8_t CAR_IsStationaryForImuBias(void)
{
  uint8_t index;
  uint8_t stationary;

  stationary = (CAR_OutputsAreZero() != 0U) ? 1U : 0U;
  if (stationary == 0U)
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  for (index = 0U; index < 4U; ++index)
  {
    if ((car_measured_rpm_x10[index] > CAR_IMU_STATIONARY_RPM_X10) ||
        (car_measured_rpm_x10[index] < -CAR_IMU_STATIONARY_RPM_X10))
    {
      stationary = 0U;
      break;
    }
  }
  taskEXIT_CRITICAL();
  return stationary;
}

static uint16_t CAR_ReadVinX100(void)
{
  uint16_t value;

  if (osMutexAcquire(vinMutexHandle, 20U) != osOK)
  {
    return 0U;
  }
  value = AX_VIN_GetVol_X100();
  (void)osMutexRelease(vinMutexHandle);
  return value;
}

static void CAR_RequestSafeStop(void)
{
  uint8_t index;

  taskENTER_CRITICAL();
  car_drive_mode = CAR_DRIVE_DISABLED;
  car_drive_is_obstacle_retreat = 0U;
  car_pid_reset_request = 1U;
  for (index = 0U; index < 4U; ++index)
  {
    car_target_rpm[index] = 0;
    car_pwm_command[index] = 0;
  }
  taskEXIT_CRITICAL();

  /* STBY is the asynchronous hardware gate. The motor task clears all four
     compare registers and PID state on its next (at most 10 ms) cycle. */
  AX_MOTOR_SetStandby(0U);
  if (motorCommandQueueHandle != NULL)
  {
    (void)osMessageQueueReset(motorCommandQueueHandle);
  }
}

static int32_t CAR_RoundFloatToInt32(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint8_t CAR_QueueVelocityCommand(float linear_mps, float angular_rps)
{
  float left_mps;
  float right_mps;
  float left_rpm;
  float right_rpm;
  float largest_rpm;
  float scale;
  float rpm_per_mps;
  int32_t target_rpm[4];
  CarDriveMode_t mode;

  left_mps = linear_mps - (0.5f * CAR_WHEEL_TRACK_M * angular_rps);
  right_mps = linear_mps + (0.5f * CAR_WHEEL_TRACK_M * angular_rps);
  rpm_per_mps = 60.0f / (6.28318530718f * CAR_WHEEL_RADIUS_M);
  left_rpm = left_mps * rpm_per_mps;
  right_rpm = right_mps * rpm_per_mps;

  largest_rpm = (left_rpm < 0.0f) ? -left_rpm : left_rpm;
  if (((right_rpm < 0.0f) ? -right_rpm : right_rpm) > largest_rpm)
  {
    largest_rpm = (right_rpm < 0.0f) ? -right_rpm : right_rpm;
  }
  if (largest_rpm > CAR_MAX_WHEEL_RPM)
  {
    scale = CAR_MAX_WHEEL_RPM / largest_rpm;
    left_rpm *= scale;
    right_rpm *= scale;
  }

  target_rpm[0] = CAR_RoundFloatToInt32(left_rpm);
  target_rpm[1] = CAR_RoundFloatToInt32(right_rpm);
  target_rpm[2] = target_rpm[0];
  target_rpm[3] = target_rpm[1];
  mode = ((target_rpm[0] == 0) && (target_rpm[1] == 0)) ?
         CAR_DRIVE_DISABLED : CAR_DRIVE_CLOSED_LOOP;
  return CAR_QueueDrive(mode, target_rpm,
      (uint8_t)((linear_mps < 0.0f) && (angular_rps == 0.0f)));
}

static uint8_t CAR_QueueHolonomicVelocityCommand(float linear_mps,
                                                  float lateral_mps,
                                                  float angular_rps)
{
  float wheel_mps[4];
  float wheel_rpm[4];
  float rpm_per_mps;
  float largest_rpm;
  float absolute_rpm;
  float scale;
  int32_t target_rpm[4];
  CarDriveMode_t mode;
  uint8_t index;

  /* Positive lateral velocity is the Android/UI command for right strafe.
     The calibrated four-wheel sequence lives in mecanum_mixer.h so protocol,
     UART debug and direct motor paths cannot drift apart. */
  CAR_MecanumMixVelocity(linear_mps, lateral_mps, angular_rps, wheel_mps);
  rpm_per_mps = 60.0f / (6.28318530718f * CAR_WHEEL_RADIUS_M);
  largest_rpm = 0.0f;
  for (index = 0U; index < 4U; ++index)
  {
    wheel_rpm[index] = wheel_mps[index] * rpm_per_mps;
    absolute_rpm = (wheel_rpm[index] < 0.0f) ?
                   -wheel_rpm[index] : wheel_rpm[index];
    if (absolute_rpm > largest_rpm)
    {
      largest_rpm = absolute_rpm;
    }
  }
  scale = (largest_rpm > CAR_MAX_WHEEL_RPM) ?
          (CAR_MAX_WHEEL_RPM / largest_rpm) : 1.0f;
  mode = CAR_DRIVE_DISABLED;
  for (index = 0U; index < 4U; ++index)
  {
    target_rpm[index] = CAR_RoundFloatToInt32(wheel_rpm[index] * scale);
    if (target_rpm[index] != 0)
    {
      mode = CAR_DRIVE_CLOSED_LOOP;
    }
  }
  return CAR_QueueDrive(mode, target_rpm,
      (uint8_t)((linear_mps < 0.0f) && (lateral_mps == 0.0f) &&
                (angular_rps == 0.0f)));
}

static uint8_t CAR_QueueProtocolFrame(uint8_t message_type,
                                      uint32_t transport_sequence,
                                      const uint8_t *payload,
                                      uint16_t payload_len)
{
  return CAR_QueueProtocolFrameWithFlags(message_type, 0U,
                                         transport_sequence,
                                         payload, payload_len);
}

static uint8_t CAR_QueueProtocolFrameWithFlags(uint8_t message_type,
                                               uint8_t flags,
                                               uint32_t transport_sequence,
                                               const uint8_t *payload,
                                               uint16_t payload_len)
{
  CarUartTxMessage_t message;

  message.length = AVG_ProtocolBuildFrame(message.bytes,
                                           sizeof(message.bytes),
                                           message_type, flags,
                                           transport_sequence,
                                           payload, payload_len);
  if ((message.length == 0U) || (protocolTxQueueHandle == NULL))
  {
    return 0U;
  }
  return (osMessageQueuePut(protocolTxQueueHandle, &message, 0U, 0U) ==
          osOK) ? 1U : 0U;
}

/* Queue former USART3 text on USART2/PA2 without interleaving it with an
 * acknowledgement or a binary STATUS frame from the Orange Pi protocol. */
static uint8_t CAR_QueueRawUart2(const uint8_t *bytes, uint16_t length)
{
  CarUartTxMessage_t message;

  if ((bytes == NULL) || (length == 0U) ||
      (length > CAR_UART2_TX_MESSAGE_SIZE) ||
      (protocolTxQueueHandle == NULL))
  {
    return 0U;
  }

  (void)memcpy(message.bytes, bytes, length);
  message.length = length;
  return (osMessageQueuePut(protocolTxQueueHandle, &message, 0U, 0U) ==
          osOK) ? 1U : 0U;
}

static void CAR_SendProtocolAck(uint8_t acked_type, AvgAckResult_t result,
                                uint32_t transport_sequence)
{
  uint8_t payload[sizeof(AvgAckPayload_t)];

  (void)AVG_ProtocolEncodeAckPayload(payload, acked_type, result);
  (void)CAR_QueueProtocolFrame(AVG_MESSAGE_ACK, transport_sequence,
                               payload, sizeof(payload));
}

static void CAR_HandleProtocolFrame(const AvgProtocolFrame_t *frame,
                                    void *context)
{
  float linear_mps;
  float lateral_mps;
  float angular_rps;
  uint8_t valid;
  uint8_t comm_timeout;
  AvgAckResult_t result;

  (void)context;
  result = AVG_ACK_UNSUPPORTED;

  switch (frame->message_type)
  {
    case AVG_MESSAGE_DRIVE:
      valid = AVG_ProtocolDecodeDrive(frame, &linear_mps, &angular_rps);
      if (valid == 0U)
      {
        result = AVG_ACK_INVALID_PAYLOAD;
        break;
      }

      if (linear_mps < CAR_LINEAR_MIN_MPS)
      {
        linear_mps = CAR_LINEAR_MIN_MPS;
      }
      else if (linear_mps > CAR_LINEAR_MAX_MPS)
      {
        linear_mps = CAR_LINEAR_MAX_MPS;
      }
      if (angular_rps < CAR_ANGULAR_MIN_RPS)
      {
        angular_rps = CAR_ANGULAR_MIN_RPS;
      }
      else if (angular_rps > CAR_ANGULAR_MAX_RPS)
      {
        angular_rps = CAR_ANGULAR_MAX_RPS;
      }

      taskENTER_CRITICAL();
      car_last_valid_drive_tick = HAL_GetTick();
      car_comm_timeout = 0U;
      car_uart_rx_fault = 0U;
      taskEXIT_CRITICAL();

      if (CAR_HasHardwareFault() != 0U)
      {
        result = AVG_ACK_HARDWARE_FAULT;
      }
      else if (car_estop_latched != 0U)
      {
        result = AVG_ACK_ESTOP_LATCHED;
      }
      else if (CAR_ApplyObstacleMotionPolicy(&linear_mps, 0.0f,
                                              angular_rps) == 0U)
      {
        CAR_RequestSafeStop();
        result = AVG_ACK_HARDWARE_FAULT;
      }
      else if (CAR_QueueVelocityCommand(linear_mps, angular_rps) == 0U)
      {
        result = AVG_ACK_HARDWARE_FAULT;
      }
      else
      {
        result = AVG_ACK_OK;
      }
      break;

    case AVG_MESSAGE_DRIVE_HOLONOMIC:
      valid = AVG_ProtocolDecodeHolonomicDrive(frame, &linear_mps,
                                                &lateral_mps, &angular_rps);
      if (valid == 0U)
      {
        result = AVG_ACK_INVALID_PAYLOAD;
        break;
      }

      if (linear_mps < CAR_LINEAR_MIN_MPS) linear_mps = CAR_LINEAR_MIN_MPS;
      else if (linear_mps > CAR_LINEAR_MAX_MPS) linear_mps = CAR_LINEAR_MAX_MPS;
      if (lateral_mps < CAR_LATERAL_MIN_MPS) lateral_mps = CAR_LATERAL_MIN_MPS;
      else if (lateral_mps > CAR_LATERAL_MAX_MPS) lateral_mps = CAR_LATERAL_MAX_MPS;
      if (angular_rps < CAR_ANGULAR_MIN_RPS) angular_rps = CAR_ANGULAR_MIN_RPS;
      else if (angular_rps > CAR_ANGULAR_MAX_RPS) angular_rps = CAR_ANGULAR_MAX_RPS;

      taskENTER_CRITICAL();
      car_last_valid_drive_tick = HAL_GetTick();
      car_comm_timeout = 0U;
      car_uart_rx_fault = 0U;
      taskEXIT_CRITICAL();

      if (CAR_HasHardwareFault() != 0U) result = AVG_ACK_HARDWARE_FAULT;
      else if (car_estop_latched != 0U) result = AVG_ACK_ESTOP_LATCHED;
      else if (CAR_ApplyObstacleMotionPolicy(&linear_mps, lateral_mps,
                                              angular_rps) == 0U)
      {
        CAR_RequestSafeStop();
        result = AVG_ACK_HARDWARE_FAULT;
      }
      else if (CAR_QueueHolonomicVelocityCommand(linear_mps, lateral_mps,
                                                  angular_rps) == 0U)
        result = AVG_ACK_HARDWARE_FAULT;
      else result = AVG_ACK_OK;
      break;

    case AVG_MESSAGE_WHEEL_TEST:
    {
      uint8_t wheel_index;
      int16_t target_rpm;
      int32_t wheel_target[4] = {0, 0, 0, 0};

      valid = AVG_ProtocolDecodeWheelTest(frame, &wheel_index, &target_rpm);
      if ((valid == 0U) || (wheel_index >= 4U) || (target_rpm == 0) ||
          (target_rpm > CAR_WHEEL_TEST_MAX_RPM) ||
          (target_rpm < -CAR_WHEEL_TEST_MAX_RPM))
      {
        result = AVG_ACK_INVALID_PAYLOAD;
        break;
      }
      taskENTER_CRITICAL();
      car_last_valid_drive_tick = HAL_GetTick();
      car_comm_timeout = 0U;
      car_uart_rx_fault = 0U;
      taskEXIT_CRITICAL();
      wheel_target[wheel_index] = target_rpm;
      if (CAR_HasHardwareFault() != 0U) result = AVG_ACK_HARDWARE_FAULT;
      else if (car_estop_latched != 0U) result = AVG_ACK_ESTOP_LATCHED;
      else if (CAR_QueueDrive(CAR_DRIVE_CLOSED_LOOP, wheel_target, 0U) == 0U)
        result = AVG_ACK_HARDWARE_FAULT;
      else result = AVG_ACK_OK;
      break;
    }

    case AVG_MESSAGE_ESTOP:
      if ((frame->flags != 0U) || (frame->payload_len != 1U) ||
          (frame->payload[0] != 0x01U))
      {
        result = AVG_ACK_INVALID_PAYLOAD;
        break;
      }
      taskENTER_CRITICAL();
      car_estop_latched = 1U;
      taskEXIT_CRITICAL();
      CAR_RequestSafeStop();
      result = AVG_ACK_OK;
      break;

    case AVG_MESSAGE_CLEAR_ESTOP:
      if ((frame->flags != 0U) || (frame->payload_len != 1U) ||
          (frame->payload[0] != 0xA5U))
      {
        result = AVG_ACK_INVALID_PAYLOAD;
        break;
      }

      taskENTER_CRITICAL();
      comm_timeout = car_comm_timeout;
      taskEXIT_CRITICAL();
      if ((CAR_HasHardwareFault() != 0U) || (comm_timeout != 0U) ||
          (CAR_OutputsAreZero() == 0U))
      {
        result = AVG_ACK_HARDWARE_FAULT;
        break;
      }

      taskENTER_CRITICAL();
      car_estop_latched = 0U;
      taskEXIT_CRITICAL();
      CAR_RequestSafeStop();
      result = AVG_ACK_OK;
      break;

    case AVG_MESSAGE_SET_OBSTACLE_GUARD:
      if ((frame->flags != 0U) || (frame->payload_len != 1U) ||
          (frame->payload[0] > 1U))
      {
        result = AVG_ACK_INVALID_PAYLOAD;
        break;
      }
      if (CAR_OutputsAreZero() == 0U)
      {
        result = AVG_ACK_HARDWARE_FAULT;
        break;
      }
      taskENTER_CRITICAL();
      car_obstacle_guard_enabled = frame->payload[0];
      if (car_obstacle_guard_enabled == 0U)
      {
        /* Keep any existing ESTOP latched; user must explicitly clear it. */
        car_obstacle_fault = AVG_FAULT_NONE;
      }
      taskEXIT_CRITICAL();
      CAR_RequestSafeStop();
      result = AVG_ACK_OK;
      break;

    default:
      result = AVG_ACK_UNSUPPORTED;
      break;
  }

  CAR_SendProtocolAck(frame->message_type, result,
                      frame->transport_sequence);
}

static void CAR_ResetUart2Reception(void)
{
  (void)HAL_UART_AbortReceive(&huart2);
  xStreamBufferReset(car_uart2_stream_handle);
  car_uart2_dma_position = 0U;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, car_uart2_dma_buffer,
                                   sizeof(car_uart2_dma_buffer)) == HAL_OK)
  {
    car_uart_rx_restart_requested = 0U;
  }
  else
  {
    car_uart_rx_fault = 1U;
    car_uart_rx_restart_requested = 1U;
  }
}

static void CAR_UartRxTask(void *argument)
{
  AvgProtocolParser_t parser;
  uint8_t chunk[CAR_UART2_READ_CHUNK_SIZE];
  size_t received;

  (void)argument;
  AVG_ProtocolParserInit(&parser);
  CAR_ResetUart2Reception();

  for (;;)
  {
    received = xStreamBufferReceive(car_uart2_stream_handle, chunk,
                                    sizeof(chunk),
                                    pdMS_TO_TICKS(CAR_UART2_RX_RETRY_MS));
    if (received != 0U)
    {
      AVG_ProtocolParserFeed(&parser, chunk, received,
                             CAR_HandleProtocolFrame, NULL);
    }

    if (car_uart_rx_restart_requested != 0U)
    {
      AVG_ProtocolParserInit(&parser);
      CAR_ResetUart2Reception();
    }
  }
}

static void CAR_SafetyTask(void *argument)
{
  TickType_t last_wake;
  uint32_t now;
  uint32_t last_drive;
  uint8_t timed_out;
  HC_SR04_Snapshot_t range;
  ObstacleGuard_t obstacle = {0U, 0U, AVG_FAULT_ULTRASONIC_UNAVAILABLE};
  uint32_t last_valid_tick = 0U;
  uint32_t last_sample_count = 0U;
  uint16_t last_distance_mm = HC_SR04_UNAVAILABLE_MM;
  uint8_t have_valid = 0U;
  uint8_t range_fault;
  uint8_t active_range_fault;
  uint8_t previous_range_fault = AVG_FAULT_NONE;

  (void)argument;
  last_wake = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAR_SAFETY_PERIOD_MS));

    now = HAL_GetTick();
    HC_SR04_GetSnapshot(&range);
    if ((range.state == HC_SR04_STATE_VALID) &&
        (range.sample_count != last_sample_count) &&
        (range.distance_mm != HC_SR04_UNAVAILABLE_MM))
    {
      last_sample_count = range.sample_count;
      last_valid_tick = range.sample_tick_ms;
      last_distance_mm = range.distance_mm;
      have_valid = 1U;
    }
    range_fault = ObstacleGuard_Update(&obstacle, now,
        (uint8_t)((have_valid != 0U) &&
                  ((uint32_t)(now - last_valid_tick) <= CAR_ULTRASONIC_STALE_MS)),
        last_distance_mm);
    taskENTER_CRITICAL();
    if (car_obstacle_guard_enabled != 0U)
    {
      car_obstacle_fault = range_fault;
      active_range_fault = range_fault;
      if (range_fault == AVG_FAULT_ULTRASONIC_UNAVAILABLE)
      {
        /* Unknown front clearance remains a hard, explicitly cleared fault. */
        car_estop_latched = 1U;
      }
    }
    else
    {
      car_obstacle_fault = AVG_FAULT_NONE;
      active_range_fault = AVG_FAULT_NONE;
    }
    taskEXIT_CRITICAL();
    if (active_range_fault != previous_range_fault)
    {
      if (active_range_fault != AVG_FAULT_NONE)
      {
        CAR_RequestSafeStop();
      }
      previous_range_fault = active_range_fault;
    }

    if (CAR_PHYSICAL_ESTOP_RELEASED() == 0U)
    {
      taskENTER_CRITICAL();
      car_estop_latched = 1U;
      taskEXIT_CRITICAL();
      CAR_RequestSafeStop();
      continue;
    }

    now = HAL_GetTick();
    taskENTER_CRITICAL();
    last_drive = car_last_valid_drive_tick;
    timed_out = car_comm_timeout;
    taskEXIT_CRITICAL();
    if (((uint32_t)(now - last_drive) > CAR_COMM_TIMEOUT_MS) &&
        (timed_out == 0U))
    {
      taskENTER_CRITICAL();
      car_comm_timeout = 1U;
      taskEXIT_CRITICAL();
      CAR_RequestSafeStop();
    }
  }
}

static void CAR_ProtocolTxTask(void *argument)
{
  CarUartTxMessage_t message;
  uint32_t flags;

  (void)argument;
  for (;;)
  {
    if (osMessageQueueGet(protocolTxQueueHandle, &message, NULL,
                          osWaitForever) != osOK)
    {
      continue;
    }

    (void)osThreadFlagsClear(CAR_PROTOCOL_TX_DONE_FLAG |
                             CAR_PROTOCOL_TX_ERROR_FLAG);
    if (HAL_UART_Transmit_IT(&huart2, message.bytes,
                            message.length) != HAL_OK)
    {
      car_uart_rx_fault = 1U;
      continue;
    }

    flags = osThreadFlagsWait(CAR_PROTOCOL_TX_DONE_FLAG |
                              CAR_PROTOCOL_TX_ERROR_FLAG,
                              osFlagsWaitAny, CAR_UART2_TX_TIMEOUT_MS);
    if (((flags & osFlagsError) != 0U) ||
        ((flags & CAR_PROTOCOL_TX_ERROR_FLAG) != 0U))
    {
      (void)HAL_UART_AbortTransmit(&huart2);
      car_uart_rx_fault = 1U;
    }
  }
}

static void CAR_SendProtocolStatus(void)
{
  AvgStatusPayload_t status;
  CarImuSnapshot_t imu;
  HC_SR04_Snapshot_t ultrasonic;
  uint32_t ultrasonic_age_ms;
  int32_t rpm_x10[4];
  float left_mps;
  float right_mps;
  float metres_per_rpm;
  uint8_t payload[sizeof(AvgStatusPayload_t)];
  uint8_t status_flags;
  uint8_t pairing_pressed;
  uint8_t shutdown_pressed;
  uint8_t index;
  uint32_t now_ms;
  static uint32_t pairing_press_started_ms = 0U;
  static uint32_t pairing_signal_until_ms = 0U;
  static uint8_t pairing_request_consumed = 0U;
  static uint32_t shutdown_press_started_ms = 0U;
  static uint8_t shutdown_request_consumed = 0U;
  static uint8_t reset_shutdown_request_consumed = 0U;

  now_ms = HAL_GetTick();
  pairing_pressed = CAR_BLE_PAIR_BUTTON_PRESSED() ? 1U : 0U;
  if ((pairing_pressed != 0U) && (CAR_OutputsAreZero() != 0U))
  {
    if ((pairing_press_started_ms == 0U) &&
        (pairing_request_consumed == 0U))
    {
      pairing_press_started_ms = now_ms;
    }
    else if ((pairing_request_consumed == 0U) &&
             ((uint32_t)(now_ms - pairing_press_started_ms) >=
              CAR_BLE_PAIR_HOLD_MS))
    {
      pairing_request_consumed = 1U;
      pairing_signal_until_ms = now_ms + CAR_BLE_PAIR_SIGNAL_MS;
    }
  }
  else
  {
    pairing_press_started_ms = 0U;
    if (pairing_pressed == 0U)
    {
      pairing_request_consumed = 0U;
    }
  }
  if ((CAR_OPI_SHUTDOWN_ON_NRST_ENABLED == 1U) &&
      (reset_shutdown_request_consumed == 0U) &&
      (CAR_BootWasExternalReset() != 0U) &&
      (now_ms >= CAR_OPI_SHUTDOWN_ON_NRST_DELAY_MS))
  {
    reset_shutdown_request_consumed = 1U;
    /* NRST restarts the MCU; wait for UART initialization before notifying Pi. */
    (void)CAR_QueueProtocolFrame(AVG_MESSAGE_ORANGE_PI_SHUTDOWN_REQUEST,
                                 car_status_sequence++, NULL, 0U);
  }

  shutdown_pressed = CAR_OPI_SHUTDOWN_BUTTON_PRESSED() ? 1U : 0U;
  if ((shutdown_pressed != 0U) && (CAR_OutputsAreZero() != 0U))
  {
    if ((shutdown_press_started_ms == 0U) &&
        (shutdown_request_consumed == 0U))
    {
      shutdown_press_started_ms = now_ms;
    }
    else if ((shutdown_request_consumed == 0U) &&
             ((uint32_t)(now_ms - shutdown_press_started_ms) >=
              CAR_OPI_SHUTDOWN_HOLD_MS))
    {
      shutdown_request_consumed = 1U;
      /* The Orange Pi gateway latches ESTOP before issuing system poweroff. */
      (void)CAR_QueueProtocolFrame(AVG_MESSAGE_ORANGE_PI_SHUTDOWN_REQUEST,
                                   car_status_sequence++, NULL, 0U);
    }
  }
  else
  {
    shutdown_press_started_ms = 0U;
    if (shutdown_pressed == 0U)
    {
      shutdown_request_consumed = 0U;
    }
  }
  status_flags = (((int32_t)(pairing_signal_until_ms - now_ms) > 0) ?
                  AVG_STATUS_FLAG_BLE_PAIR_REQUEST : 0U) |
                 AVG_STATUS_FLAG_OBSTACLE_GUARD_CAPABLE;

  taskENTER_CRITICAL();
  status.encoder_fl = car_encoder_total[0];
  status.encoder_fr = car_encoder_total[1];
  status.encoder_rl = car_encoder_total[2];
  status.encoder_rr = car_encoder_total[3];
  for (index = 0U; index < 4U; ++index)
  {
    rpm_x10[index] = car_measured_rpm_x10[index];
  }
  imu = car_imu_snapshot;
  status.estop = car_estop_latched;
  if (car_obstacle_guard_enabled != 0U)
  {
    status_flags |= AVG_STATUS_FLAG_OBSTACLE_GUARD_ENABLED;
  }
  taskEXIT_CRITICAL();

  if (CAR_PHYSICAL_ESTOP_RELEASED() == 0U)
  {
    status.estop = 1U;
  }
  status.battery_mv = (uint16_t)(CAR_ReadVinX100() * 10U);
  HC_SR04_GetSnapshot(&ultrasonic);
  ultrasonic_age_ms = (ultrasonic.sample_tick_ms == 0U) ? 0xFFFFFFFFUL :
      (uint32_t)(HAL_GetTick() - ultrasonic.sample_tick_ms);
  status.ultrasonic_mm =
      ((ultrasonic.state == HC_SR04_STATE_VALID) &&
       (ultrasonic_age_ms <= HC_SR04_STALE_MS))
          ? ultrasonic.distance_mm : HC_SR04_UNAVAILABLE_MM;
  metres_per_rpm = (6.28318530718f * CAR_WHEEL_RADIUS_M) / 60.0f;
  left_mps = (((float)rpm_x10[0] + (float)rpm_x10[2]) / 20.0f) *
             metres_per_rpm;
  right_mps = (((float)rpm_x10[1] + (float)rpm_x10[3]) / 20.0f) *
              metres_per_rpm;
  status.measured_linear_mps = (left_mps + right_mps) * 0.5f;
  status.measured_angular_rps = (right_mps - left_mps) / CAR_WHEEL_TRACK_M;
  status.imu_yaw_rad = imu.data.yaw_deg * 0.01745329252f;
  status.fault_code = CAR_GetFaultCode();

  (void)AVG_ProtocolEncodeStatusPayload(payload, &status);
  (void)CAR_QueueProtocolFrameWithFlags(AVG_MESSAGE_STATUS, status_flags,
                                        car_status_sequence++, payload,
                                        sizeof(payload));
}

static void CAR_TelemetryTask(void *argument)
{
  TickType_t last_wake;

  (void)argument;
  last_wake = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAR_STATUS_PERIOD_MS));
    CAR_SendProtocolStatus();
  }
}

static void CAR_RefreshDebugDriveWatchdog(CarMotion_t motion)
{
  if ((motion == CAR_MOTION_FORWARD) ||
      (motion == CAR_MOTION_BACKWARD) ||
      (motion == CAR_MOTION_TURN_LEFT) ||
      (motion == CAR_MOTION_TURN_RIGHT) ||
      (motion == CAR_MOTION_STRAFE_LEFT) ||
      (motion == CAR_MOTION_STRAFE_RIGHT))
  {
    /*
     * USART3 is a local wheels-up test interface. Treat each accepted motion
     * byte as one watchdog heartbeat, but keep the same 300 ms fail-safe used
     * by the Orange Pi link. If the terminal stops sending, safetyTask still
     * disables STBY and clears PWM.
     */
    taskENTER_CRITICAL();
    car_last_valid_drive_tick = HAL_GetTick();
    car_comm_timeout = 0U;
    taskEXIT_CRITICAL();
  }
}

static uint8_t CAR_QueueMotion(CarMotion_t motion, uint16_t target_rpm)
{
  int32_t wheel_target[4] = {0, 0, 0, 0};
  CarDriveMode_t drive_mode;

  /*
   * Refresh before queueing. motorControlTask has a higher priority and may
   * consume the message immediately after osMessageQueuePut() succeeds.
   */
  CAR_RefreshDebugDriveWatchdog(motion);
  drive_mode = CAR_DRIVE_BRAKE;
  switch (motion)
  {
    case CAR_MOTION_FORWARD:
      wheel_target[0] = target_rpm;
      wheel_target[1] = target_rpm;
      wheel_target[2] = target_rpm;
      wheel_target[3] = target_rpm;
      drive_mode = CAR_DRIVE_CLOSED_LOOP;
      break;

    case CAR_MOTION_BACKWARD:
      wheel_target[0] = -(int32_t)target_rpm;
      wheel_target[1] = -(int32_t)target_rpm;
      wheel_target[2] = -(int32_t)target_rpm;
      wheel_target[3] = -(int32_t)target_rpm;
      drive_mode = CAR_DRIVE_CLOSED_LOOP;
      break;

    case CAR_MOTION_TURN_LEFT:
      wheel_target[0] = -(int32_t)target_rpm;
      wheel_target[1] = target_rpm;
      wheel_target[2] = -(int32_t)target_rpm;
      wheel_target[3] = target_rpm;
      drive_mode = CAR_DRIVE_CLOSED_LOOP;
      break;

    case CAR_MOTION_TURN_RIGHT:
      wheel_target[0] = target_rpm;
      wheel_target[1] = -(int32_t)target_rpm;
      wheel_target[2] = target_rpm;
      wheel_target[3] = -(int32_t)target_rpm;
      drive_mode = CAR_DRIVE_CLOSED_LOOP;
      break;

    case CAR_MOTION_STRAFE_LEFT:
      CAR_MecanumMixCommand(0, -(int32_t)target_rpm, 0, wheel_target);
      drive_mode = CAR_DRIVE_CLOSED_LOOP;
      break;

    case CAR_MOTION_STRAFE_RIGHT:
      CAR_MecanumMixCommand(0, (int32_t)target_rpm, 0, wheel_target);
      drive_mode = CAR_DRIVE_CLOSED_LOOP;
      break;

    case CAR_MOTION_COAST:
      drive_mode = CAR_DRIVE_COAST;
      break;

    case CAR_MOTION_DISABLED:
      drive_mode = CAR_DRIVE_DISABLED;
      break;

    case CAR_MOTION_BRAKE:
    default:
      drive_mode = CAR_DRIVE_BRAKE;
      break;
  }

  return CAR_QueueDrive(drive_mode, wheel_target, 0U);
}

static int32_t CAR_Abs32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static void CAR_ReadEncoderCounters(int16_t counter[4])
{
  /* Convert physical C/A/B/D into logical FL/FR/RL/RR order. */
  counter[0] = AX_ENCODER_C_GetCounter();
  counter[1] = AX_ENCODER_A_GetCounter();
  counter[2] = AX_ENCODER_B_GetCounter();
  counter[3] = AX_ENCODER_D_GetCounter();
}

static void CAR_ResetPid(CarPidState_t *pid)
{
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
  pid->previous_target_rpm = 0;
  pid->initialized = 0U;
}

static int16_t CAR_ComputePid(CarPidState_t *pid, int32_t target_rpm,
                              int32_t measured_rpm_x10, uint32_t dt_ms)
{
  float dt;
  float measured_rpm;
  float error;
  float derivative;
  float output;

  if ((target_rpm == 0) || (dt_ms == 0U))
  {
    CAR_ResetPid(pid);
    return 0;
  }

  if ((pid->previous_target_rpm == 0) ||
      ((pid->previous_target_rpm > 0) != (target_rpm > 0)))
  {
    CAR_ResetPid(pid);
  }

  dt = (float)dt_ms / 1000.0f;
  measured_rpm = (float)measured_rpm_x10 / 10.0f;
  error = (float)target_rpm - measured_rpm;

  pid->integral += error * dt;
  if (pid->integral > CAR_PID_INTEGRAL_LIMIT)
  {
    pid->integral = CAR_PID_INTEGRAL_LIMIT;
  }
  else if (pid->integral < -CAR_PID_INTEGRAL_LIMIT)
  {
    pid->integral = -CAR_PID_INTEGRAL_LIMIT;
  }

  derivative = 0.0f;
  if (pid->initialized != 0U)
  {
    derivative = (error - pid->previous_error) / dt;
  }

  output = (CAR_PID_KFF * (float)target_rpm) +
           (CAR_PID_KP * error) +
           (CAR_PID_KI * pid->integral) +
           (CAR_PID_KD * derivative);

  /* A speed loop may reduce PWM to zero, but never reverse the bridge merely
     to correct overspeed.  Direction changes only come from a new target. */
  if (target_rpm > 0)
  {
    if (output < 0.0f)
    {
      output = 0.0f;
    }
    else if (output > (float)AX_MOTOR_MAX_COMMAND)
    {
      output = (float)AX_MOTOR_MAX_COMMAND;
    }
  }
  else
  {
    if (output > 0.0f)
    {
      output = 0.0f;
    }
    else if (output < -(float)AX_MOTOR_MAX_COMMAND)
    {
      output = -(float)AX_MOTOR_MAX_COMMAND;
    }
  }

  pid->previous_error = error;
  pid->previous_target_rpm = target_rpm;
  pid->initialized = 1U;
  return (int16_t)output;
}

static uint8_t CAR_QueueDrive(CarDriveMode_t mode,
                              const int32_t target_rpm[4],
                              uint8_t obstacle_retreat)
{
  CarMotorMessage_t message;
  uint8_t index;

  message.type = CAR_MOTOR_MESSAGE_DRIVE;
  message.mode = mode;
  message.obstacle_retreat = obstacle_retreat;
  for (index = 0U; index < 4U; ++index)
  {
    message.target_rpm[index] = target_rpm[index];
  }

  return (osMessageQueuePut(motorCommandQueueHandle, &message, 0U, 0U) ==
          osOK) ? 1U : 0U;
}

static uint8_t CAR_QueueClearEncoderFault(void)
{
  CarMotorMessage_t message = {0};

  message.type = CAR_MOTOR_MESSAGE_CLEAR_FAULT;
  message.mode = CAR_DRIVE_BRAKE;
  return (osMessageQueuePut(motorCommandQueueHandle, &message, 0U, 0U) ==
          osOK) ? 1U : 0U;
}

static void CAR_ApplyDriveInMotorTask(CarDriveMode_t mode,
                                      const int32_t target_rpm[4],
                                      uint8_t obstacle_retreat)
{
  uint8_t index;
  uint8_t near_obstacle;

  taskENTER_CRITICAL();
  near_obstacle = ((car_obstacle_guard_enabled != 0U) &&
                   (car_obstacle_fault == AVG_FAULT_OBSTACLE)) ? 1U : 0U;
  taskEXIT_CRITICAL();
  if ((mode != CAR_DRIVE_DISABLED) &&
      ((CAR_OutputIsInhibited() != 0U) ||
       ((near_obstacle != 0U) && (obstacle_retreat == 0U))))
  {
    mode = CAR_DRIVE_DISABLED;
  }

  taskENTER_CRITICAL();
  car_drive_mode = mode;
  car_drive_is_obstacle_retreat =
      ((mode == CAR_DRIVE_CLOSED_LOOP) && (obstacle_retreat != 0U)) ? 1U : 0U;
  for (index = 0U; index < 4U; ++index)
  {
    car_target_rpm[index] = (mode == CAR_DRIVE_CLOSED_LOOP) ?
                            target_rpm[index] : 0;
  }
  taskEXIT_CRITICAL();
}

static void CAR_ClearEncoderFaultInMotorTask(void)
{
  int32_t zero_target[4] = {0, 0, 0, 0};

  taskENTER_CRITICAL();
  car_encoder_fault_wheel = 0U;
  taskEXIT_CRITICAL();
  CAR_ApplyDriveInMotorTask(CAR_DRIVE_BRAKE, zero_target, 0U);
}

static void CAR_LatchEncoderDirectionFault(uint8_t wheel)
{
  uint8_t index;

  taskENTER_CRITICAL();
  car_encoder_fault_wheel = (uint8_t)(wheel + 1U);
  car_drive_mode = CAR_DRIVE_DISABLED;
  car_drive_is_obstacle_retreat = 0U;
  for (index = 0U; index < 4U; ++index)
  {
    car_target_rpm[index] = 0;
    car_pwm_command[index] = 0;
  }
  taskEXIT_CRITICAL();

  AX_MOTOR_Disable();
}

static void CAR_MotorControlTask(void *argument)
{
  CarPidState_t pid[4];
  CarMotorMessage_t message;
  CarDriveMode_t mode;
  CarDriveMode_t previous_mode;
  int16_t previous_counter[4];
  int16_t current_counter[4];
  int16_t raw_delta[4];
  int16_t wheel_command[4];
  int32_t target_rpm[4];
  int32_t instant_rpm_x10;
  int32_t filtered_rpm_x10;
  int32_t logical_delta;
  int32_t direction_previous_target[4] = {0, 0, 0, 0};
  int8_t polarity_candidate[4] = {0, 0, 0, 0};
  uint8_t polarity_samples[4] = {0U, 0U, 0U, 0U};
  uint8_t direction_bad_samples[4] = {0U, 0U, 0U, 0U};
  uint8_t direction_grace_samples[4] = {0U, 0U, 0U, 0U};
  uint8_t index;
  uint8_t direction_fault;
  uint8_t reset_requested;
  uint8_t near_obstacle_now;
  uint8_t retreat_active;
  uint32_t now_ms;
  uint32_t previous_ms;
  uint32_t dt_ms;
  TickType_t last_wake_tick;

  (void)argument;
  for (index = 0U; index < 4U; ++index)
  {
    CAR_ResetPid(&pid[index]);
  }

  CAR_ReadEncoderCounters(previous_counter);
  previous_ms = HAL_GetTick();
  last_wake_tick = xTaskGetTickCount();
  previous_mode = (CarDriveMode_t)0xFF;

  for (;;)
  {
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(CAR_CONTROL_PERIOD_MS));

    now_ms = HAL_GetTick();
    dt_ms = (uint32_t)(now_ms - previous_ms);
    if (dt_ms == 0U)
    {
      continue;
    }
    previous_ms = now_ms;

    /* Drain all pending requests; only this task may change drive state or
       call the motor/PWM driver after the scheduler has started. */
    while (osMessageQueueGet(motorCommandQueueHandle, &message, NULL, 0U) ==
           osOK)
    {
      if (message.type == CAR_MOTOR_MESSAGE_CLEAR_FAULT)
      {
        CAR_ClearEncoderFaultInMotorTask();
      }
      else
      {
        CAR_ApplyDriveInMotorTask(message.mode, message.target_rpm,
                                  message.obstacle_retreat);
      }
    }

    taskENTER_CRITICAL();
    reset_requested = car_pid_reset_request;
    car_pid_reset_request = 0U;
    taskEXIT_CRITICAL();
    if (reset_requested != 0U)
    {
      for (index = 0U; index < 4U; ++index)
      {
        CAR_ResetPid(&pid[index]);
        polarity_candidate[index] = 0;
        polarity_samples[index] = 0U;
        direction_previous_target[index] = 0;
        direction_bad_samples[index] = 0U;
        direction_grace_samples[index] = 0U;
      }
      AX_MOTOR_Disable();
      previous_mode = CAR_DRIVE_DISABLED;
    }

    CAR_ReadEncoderCounters(current_counter);
    for (index = 0U; index < 4U; ++index)
    {
      raw_delta[index] = (int16_t)
          ((uint16_t)current_counter[index] - (uint16_t)previous_counter[index]);
      previous_counter[index] = current_counter[index];
      logical_delta = (int32_t)raw_delta[index] *
                      car_encoder_forward_sign[index];

      instant_rpm_x10 = (int32_t)(((int64_t)logical_delta * 600000LL) /
          ((int64_t)AX_ENCODER_COUNTS_PER_WHEEL_REV * (int64_t)dt_ms));
      filtered_rpm_x10 = car_measured_rpm_x10[index];
      filtered_rpm_x10 +=
          (instant_rpm_x10 - filtered_rpm_x10) / CAR_RPM_FILTER_DIVISOR;
      car_encoder_delta[index] = (int16_t)logical_delta;
      car_encoder_total[index] = (int32_t)
          ((uint32_t)car_encoder_total[index] + (uint32_t)logical_delta);
      car_measured_rpm_x10[index] = filtered_rpm_x10;
    }

    taskENTER_CRITICAL();
    mode = car_drive_mode;
    for (index = 0U; index < 4U; ++index)
    {
      target_rpm[index] = car_target_rpm[index];
    }
    taskEXIT_CRITICAL();

    direction_fault = 0U;
    if (mode == CAR_DRIVE_CLOSED_LOOP)
    {
      for (index = 0U; index < 4U; ++index)
      {
        /*
         * Learn the raw A/B polarity only from a real, sustained movement.
         * The target sign is the intended logical wheel direction; comparing
         * it with the raw timer delta yields the required encoder sign.
         */
        if (car_encoder_polarity_locked[index] == 0U)
        {
          int8_t observed_sign;

          if ((target_rpm[index] != 0) && (raw_delta[index] != 0) &&
              (CAR_Abs32(car_measured_rpm_x10[index]) >=
               CAR_ENCODER_DIR_MIN_RPM_X10))
          {
            observed_sign = ((target_rpm[index] > 0) ==
                             (raw_delta[index] > 0)) ? 1 : -1;
            if (polarity_candidate[index] == observed_sign)
            {
              if (polarity_samples[index] <
                  CAR_ENCODER_POLARITY_CONFIRM_SAMPLES)
              {
                ++polarity_samples[index];
              }
            }
            else
            {
              polarity_candidate[index] = observed_sign;
              polarity_samples[index] = 1U;
            }

            if (polarity_samples[index] >=
                CAR_ENCODER_POLARITY_CONFIRM_SAMPLES)
            {
              if (car_encoder_forward_sign[index] != observed_sign)
              {
                car_encoder_forward_sign[index] = observed_sign;
                car_encoder_delta[index] =
                    (int16_t)((int32_t)raw_delta[index] * observed_sign);
                car_encoder_total[index] = 0;
                car_measured_rpm_x10[index] =
                    (target_rpm[index] > 0) ?
                    CAR_Abs32(car_measured_rpm_x10[index]) :
                    -CAR_Abs32(car_measured_rpm_x10[index]);
                CAR_ResetPid(&pid[index]);
              }
              car_encoder_polarity_locked[index] = 1U;
            }
          }
          else
          {
            polarity_candidate[index] = 0;
            polarity_samples[index] = 0U;
          }
        }

        if ((direction_previous_target[index] == 0) ||
            ((direction_previous_target[index] > 0) != (target_rpm[index] > 0)))
        {
          direction_grace_samples[index] = CAR_ENCODER_DIR_GRACE_SAMPLES;
          direction_bad_samples[index] = 0U;
        }
        direction_previous_target[index] = target_rpm[index];

        if (direction_grace_samples[index] != 0U)
        {
          --direction_grace_samples[index];
        }
        else if ((car_encoder_polarity_locked[index] != 0U) &&
                 (target_rpm[index] != 0) &&
                 (CAR_Abs32(car_measured_rpm_x10[index]) >=
                  CAR_ENCODER_DIR_MIN_RPM_X10) &&
                 ((target_rpm[index] > 0) !=
                  (car_measured_rpm_x10[index] > 0)))
        {
          if (++direction_bad_samples[index] >=
              CAR_ENCODER_DIR_FAULT_SAMPLES)
          {
            CAR_LatchEncoderDirectionFault(index);
            direction_fault = 1U;
            break;
          }
        }
        else
        {
          direction_bad_samples[index] = 0U;
        }
      }
    }

    if (direction_fault != 0U)
    {
      previous_mode = CAR_DRIVE_DISABLED;
      continue;
    }

    if (mode != previous_mode)
    {
      if (mode == CAR_DRIVE_DISABLED)
      {
        AX_MOTOR_Disable();
      }
      else if (mode == CAR_DRIVE_BRAKE)
      {
        AX_MOTOR_SetStandby(1U);
        AX_MOTOR_BrakeAll();
      }
      else if (mode == CAR_DRIVE_COAST)
      {
        AX_MOTOR_SetStandby(1U);
        AX_MOTOR_CoastAll();
      }
      else
      {
        AX_MOTOR_SetStandby(1U);
      }
      previous_mode = mode;
    }

    if (mode == CAR_DRIVE_CLOSED_LOOP)
    {
      for (index = 0U; index < 4U; ++index)
      {
        wheel_command[index] = CAR_ComputePid(&pid[index], target_rpm[index],
                                              car_measured_rpm_x10[index], dt_ms);
        wheel_command[index] = MotorStartCompensate(
            target_rpm[index], car_measured_rpm_x10[index],
            wheel_command[index], CAR_MOTOR_START_MIN_TARGET_RPM,
            CAR_MOTOR_MOVING_RPM_X10, car_motor_start_command[index]);
        car_pwm_command[index] = wheel_command[index];
      }
      AX_MOTOR_SetWheelSpeeds(wheel_command[0], wheel_command[1],
                              wheel_command[2], wheel_command[3]);
    }
    else
    {
      for (index = 0U; index < 4U; ++index)
      {
        CAR_ResetPid(&pid[index]);
        polarity_candidate[index] = 0;
        polarity_samples[index] = 0U;
        direction_previous_target[index] = 0;
        direction_bad_samples[index] = 0U;
        direction_grace_samples[index] = 0U;
        car_pwm_command[index] = 0;
      }
    }

    /* Close the only remaining race: a high-priority safety event can occur
       after this cycle copied its mode. Re-check after all bridge writes. */
    taskENTER_CRITICAL();
    near_obstacle_now = ((car_obstacle_guard_enabled != 0U) &&
                         (car_obstacle_fault == AVG_FAULT_OBSTACLE)) ? 1U : 0U;
    retreat_active = car_drive_is_obstacle_retreat;
    taskEXIT_CRITICAL();
    if ((CAR_OutputIsInhibited() != 0U) ||
        ((near_obstacle_now != 0U) && (retreat_active == 0U)))
    {
      AX_MOTOR_Disable();
      previous_mode = CAR_DRIVE_DISABLED;
      taskENTER_CRITICAL();
      car_drive_mode = CAR_DRIVE_DISABLED;
      car_drive_is_obstacle_retreat = 0U;
      for (index = 0U; index < 4U; ++index)
      {
        car_target_rpm[index] = 0;
        car_pwm_command[index] = 0;
      }
      taskEXIT_CRITICAL();
    }
  }
}

static void CAR_PublishImu(const MPU6050_Data_t *imu, CarImuState_t state,
                           uint8_t who_am_i, uint32_t sample_tick_ms,
                           uint32_t sample_count, uint32_t error_count)
{
  taskENTER_CRITICAL();
  if (imu != NULL)
  {
    car_imu_snapshot.data = *imu;
  }
  car_imu_snapshot.state = state;
  car_imu_snapshot.who_am_i = who_am_i;
  car_imu_snapshot.sample_tick_ms = sample_tick_ms;
  car_imu_snapshot.sample_count = sample_count;
  car_imu_snapshot.error_count = error_count;
  taskEXIT_CRITICAL();
}

static void CAR_ImuTask(void *argument)
{
  MPU6050_Data_t imu;
  float gx_sum;
  float gy_sum;
  float gz_sum;
  float dt_s;
  uint32_t flags;
  uint32_t now_tick;
  uint32_t previous_tick;
  uint32_t sample_count;
  uint32_t error_count;
  uint8_t who_am_i;
  uint8_t calibration_ok;
  uint8_t consecutive_errors;
  uint16_t calibration_index;

  (void)argument;
  memset(&imu, 0, sizeof(imu));
  sample_count = 0U;
  error_count = 0U;
  who_am_i = 0U;
  CAR_PublishImu(&imu, CAR_IMU_OFFLINE, who_am_i, 0U,
                 sample_count, error_count);

  for (;;)
  {
    memset(&imu, 0, sizeof(imu));
    who_am_i = 0U;
    CAR_PublishImu(&imu, CAR_IMU_INITIALIZING, who_am_i, 0U,
                   sample_count, error_count);

    if (MPU6050_Init(&who_am_i) != HAL_OK)
    {
      ++error_count;
      CAR_PublishImu(&imu, CAR_IMU_ERROR, who_am_i, 0U,
                     sample_count, error_count);
      osDelay(CAR_IMU_RETRY_DELAY_MS);
      continue;
    }

    CAR_PublishImu(&imu, CAR_IMU_CALIBRATING, who_am_i, 0U,
                   sample_count, error_count);
    gx_sum = 0.0f;
    gy_sum = 0.0f;
    gz_sum = 0.0f;
    calibration_ok = 1U;
    for (calibration_index = 0U;
         calibration_index < MPU6050_CALIBRATION_SAMPLES;
         ++calibration_index)
    {
      if (MPU6050_ReadBlocking(&imu) != HAL_OK)
      {
        calibration_ok = 0U;
        ++error_count;
        break;
      }
      gx_sum += imu.gx_dps;
      gy_sum += imu.gy_dps;
      gz_sum += imu.gz_dps;
      osDelay(MPU6050_SAMPLE_PERIOD_MS);
    }

    if (calibration_ok == 0U)
    {
      CAR_PublishImu(&imu, CAR_IMU_ERROR, who_am_i, 0U,
                     sample_count, error_count);
      osDelay(CAR_IMU_RETRY_DELAY_MS);
      continue;
    }

    imu.gx_offset_dps = gx_sum / (float)MPU6050_CALIBRATION_SAMPLES;
    imu.gy_offset_dps = gy_sum / (float)MPU6050_CALIBRATION_SAMPLES;
    imu.gz_offset_dps = gz_sum / (float)MPU6050_CALIBRATION_SAMPLES;
    if (MPU6050_ReadBlocking(&imu) != HAL_OK)
    {
      ++error_count;
      CAR_PublishImu(&imu, CAR_IMU_ERROR, who_am_i, 0U,
                     sample_count, error_count);
      osDelay(CAR_IMU_RETRY_DELAY_MS);
      continue;
    }

    MPU6050_SetInitialAttitude(&imu);
    previous_tick = HAL_GetTick();
    consecutive_errors = 0U;
    (void)osThreadFlagsClear(CAR_IMU_FLAG_DATA_READY |
                             CAR_IMU_FLAG_DMA_DONE |
                             CAR_IMU_FLAG_DMA_ERROR);
    CAR_PublishImu(&imu, CAR_IMU_READY, who_am_i, previous_tick,
                   sample_count, error_count);

    while (consecutive_errors < CAR_IMU_MAX_CONSECUTIVE_ERRORS)
    {
      /* PB5 normally wakes the task every 10 ms. A timeout still starts one
         read, allowing operation if the sensor interrupt wire is absent. */
      (void)osThreadFlagsWait(CAR_IMU_FLAG_DATA_READY, osFlagsWaitAny,
                              MPU6050_SAMPLE_PERIOD_MS + 5U);
      (void)osThreadFlagsClear(CAR_IMU_FLAG_DMA_DONE |
                               CAR_IMU_FLAG_DMA_ERROR);

      if (MPU6050_StartReadDMA() != HAL_OK)
      {
        ++error_count;
        ++consecutive_errors;
        osDelay(MPU6050_SAMPLE_PERIOD_MS);
        continue;
      }

      flags = osThreadFlagsWait(CAR_IMU_FLAG_DMA_DONE |
                                CAR_IMU_FLAG_DMA_ERROR,
                                osFlagsWaitAny, CAR_IMU_DMA_TIMEOUT_MS);
      if (((flags & osFlagsError) != 0U) ||
          ((flags & CAR_IMU_FLAG_DMA_ERROR) != 0U))
      {
        ++error_count;
        ++consecutive_errors;
        continue;
      }

      now_tick = HAL_GetTick();
      dt_s = (float)((uint32_t)(now_tick - previous_tick)) / 1000.0f;
      previous_tick = now_tick;
      MPU6050_ProcessDmaSample(&imu, dt_s, CAR_IsStationaryForImuBias());
      ++sample_count;
      consecutive_errors = 0U;
      CAR_PublishImu(&imu, CAR_IMU_READY, who_am_i, now_tick,
                     sample_count, error_count);
    }

    CAR_PublishImu(&imu, CAR_IMU_ERROR, who_am_i, previous_tick,
                   sample_count, error_count);

    /* Recover only the I2C peripheral. Sensor failures never stop or block
       the independent motor-control task. */
    (void)HAL_I2C_DeInit(&hi2c1);
    (void)HAL_I2C_Init(&hi2c1);
    osDelay(CAR_IMU_RETRY_DELAY_MS);
  }
}

static void CAR_UltrasonicTask(void *argument)
{
  uint32_t cycle_start;
  uint32_t elapsed_ms;
  uint32_t flags;

  (void)argument;
  while (HC_SR04_Init() != HAL_OK)
  {
    osDelay(1000U);
  }

  for (;;)
  {
    cycle_start = HAL_GetTick();
    (void)osThreadFlagsClear(CAR_ULTRASONIC_FLAG_CAPTURE_DONE);
    if (HC_SR04_Trigger() == HAL_OK)
    {
      flags = osThreadFlagsWait(CAR_ULTRASONIC_FLAG_CAPTURE_DONE,
                                osFlagsWaitAny,
                                HC_SR04_ECHO_TIMEOUT_MS);
      if (((flags & osFlagsError) != 0U) ||
          ((flags & CAR_ULTRASONIC_FLAG_CAPTURE_DONE) == 0U))
      {
        HC_SR04_MarkTimeout();
      }
    }
    else
    {
      HC_SR04_MarkTimeout();
    }

    elapsed_ms = (uint32_t)(HAL_GetTick() - cycle_start);
    if (elapsed_ms < HC_SR04_TRIGGER_PERIOD_MS)
    {
      osDelay(HC_SR04_TRIGGER_PERIOD_MS - elapsed_ms);
    }
  }
}

static uint8_t CAR_HandleCommand(uint8_t command, CarMotion_t *motion,
                                 uint16_t *target_rpm)
{
  CarMotion_t requested_motion;
  uint16_t requested_target_rpm;

  requested_motion = *motion;
  requested_target_rpm = *target_rpm;

  if ((command >= (uint8_t)'1') && (command <= (uint8_t)'9'))
  {
    requested_target_rpm = (uint16_t)(CAR_TARGET_RPM_STEP *
                                      (command - (uint8_t)'0'));
    if (CAR_QueueMotion(requested_motion, requested_target_rpm) == 0U)
    {
      return CAR_COMMAND_BUSY;
    }
    *target_rpm = requested_target_rpm;
    return CAR_COMMAND_ACCEPTED;
  }

  switch (command)
  {
    case 'F':
    case 'f':
      requested_motion = CAR_MOTION_FORWARD;
      break;

    case 'B':
    case 'b':
      requested_motion = CAR_MOTION_BACKWARD;
      break;

    case 'L':
    case 'l':
      requested_motion = CAR_MOTION_TURN_LEFT;
      break;

    case 'R':
    case 'r':
      requested_motion = CAR_MOTION_TURN_RIGHT;
      break;

#if (AX_MOTOR_ENABLE_STRAFE == 1)
    case 'Q':
    case 'q':
      requested_motion = CAR_MOTION_STRAFE_LEFT;
      break;

    case 'E':
    case 'e':
      requested_motion = CAR_MOTION_STRAFE_RIGHT;
      break;
#endif

    case 'S':
    case 's':
    case 'X':
    case 'x':
      requested_motion = CAR_MOTION_BRAKE;
      break;

    case 'C':
    case 'c':
      requested_motion = CAR_MOTION_COAST;
      break;

    case '!':
    case '0':
      requested_motion = CAR_MOTION_DISABLED;
      break;

    case 'K':
    case 'k':
      if (CAR_QueueClearEncoderFault() == 0U)
      {
        return CAR_COMMAND_BUSY;
      }
      *motion = CAR_MOTION_BRAKE;
      return CAR_COMMAND_ACCEPTED;

    case '?':
      return CAR_COMMAND_HELP;

    default:
      return CAR_COMMAND_IGNORED;
  }

  if (CAR_QueueMotion(requested_motion, requested_target_rpm) == 0U)
  {
    return CAR_COMMAND_BUSY;
  }
  *motion = requested_motion;
  return CAR_COMMAND_ACCEPTED;
}

static const char *CAR_GetMotionName(CarMotion_t motion)
{
  switch (motion)
  {
    case CAR_MOTION_FORWARD:      return "FORWARD";
    case CAR_MOTION_BACKWARD:     return "BACKWARD";
    case CAR_MOTION_TURN_LEFT:    return "TURN_LEFT";
    case CAR_MOTION_TURN_RIGHT:   return "TURN_RIGHT";
    case CAR_MOTION_STRAFE_LEFT:  return "STRAFE_LEFT";
    case CAR_MOTION_STRAFE_RIGHT: return "STRAFE_RIGHT";
    case CAR_MOTION_COAST:        return "COAST";
    case CAR_MOTION_DISABLED:     return "DISABLED";
    case CAR_MOTION_BRAKE:
    default:                      return "BRAKE";
  }
}

static void CAR_SendText(const char *text)
{
  size_t length;

  length = strlen(text);
  if (length > 0xFFFFU)
  {
    length = 0xFFFFU;
  }

  if (length != 0U)
  {
    (void)CAR_QueueRawUart2((const uint8_t *)text, (uint16_t)length);
  }
}

static void CAR_SendStatus(CarMotion_t motion, uint16_t target_rpm)
{
  char buffer[96];
  int length;
  uint8_t fault_wheel;

  fault_wheel = car_encoder_fault_wheel;
  length = snprintf(buffer, sizeof(buffer),
                    "STATE:%s TARGET_RPM:%u ENC_DIR_FAULT:%u\r\n",
                    CAR_GetMotionName(motion), (unsigned int)target_rpm,
                    (unsigned int)fault_wheel);
  if (length > 0)
  {
    if ((size_t)length >= sizeof(buffer))
    {
      length = (int)(sizeof(buffer) - 1U);
    }
    (void)CAR_QueueRawUart2((const uint8_t *)buffer, (uint16_t)length);
  }
}

static void CAR_SendCommandAck(uint8_t command, CarMotion_t motion,
                               uint16_t target_rpm)
{
  char buffer[112];
  int length;
  uint8_t fault_wheel;

  fault_wheel = car_encoder_fault_wheel;
  length = snprintf(buffer, sizeof(buffer),
                    "ACK:%c STATE:%s TARGET_RPM:%u FAULT:%u\r\n",
                    (char)command, CAR_GetMotionName(motion),
                    (unsigned int)target_rpm, (unsigned int)fault_wheel);
  if (length > 0)
  {
    if ((size_t)length >= sizeof(buffer))
    {
      length = (int)(sizeof(buffer) - 1U);
    }
    (void)CAR_QueueRawUart2((const uint8_t *)buffer, (uint16_t)length);
  }
}

static void CAR_SendCommandNack(uint8_t command)
{
  char buffer[32];
  int length;

  length = snprintf(buffer, sizeof(buffer),
                    "NACK:0x%02X SEND:? FOR HELP\r\n",
                    (unsigned int)command);
  if (length > 0)
  {
    if ((size_t)length >= sizeof(buffer))
    {
      length = (int)(sizeof(buffer) - 1U);
    }
    (void)CAR_QueueRawUart2((const uint8_t *)buffer, (uint16_t)length);
  }
}

static void CAR_SendCommandBusy(uint8_t command)
{
  char buffer[40];
  int length;

  length = snprintf(buffer, sizeof(buffer),
                    "NACK:%c MOTOR_QUEUE_BUSY\r\n", (char)command);
  if (length > 0)
  {
    if ((size_t)length >= sizeof(buffer))
    {
      length = (int)(sizeof(buffer) - 1U);
    }
    (void)CAR_QueueRawUart2((const uint8_t *)buffer, (uint16_t)length);
  }
}

static void CAR_SendHelp(void)
{
#if (AX_MOTOR_ENABLE_STRAFE == 1)
  CAR_SendText("CMD:F=FORWARD B=BACKWARD L=LEFT R=RIGHT Q=STRAFE_LEFT "
               "E=STRAFE_RIGHT S/X=BRAKE C=COAST !=DISABLE "
               "1..9=20..180RPM K=CLEAR_FAULT ?=HELP\r\n");
#else
  CAR_SendText("CMD:F=FORWARD B=BACKWARD L=LEFT R=RIGHT "
               "S/X=BRAKE C=COAST !=DISABLE "
               "1..9=20..180RPM K=CLEAR_FAULT ?=HELP\r\n");
#endif
}

static void CAR_SendTelemetry(void)
{
  int16_t encoder_delta[4];
  int32_t encoder_total[4];
  int16_t pwm[4];
  int32_t rpm_x10[4];
  int32_t rpm_abs_x10[4];
  int32_t target[4];
  int8_t encoder_sign[4];
  uint8_t encoder_locked[4];
  uint16_t vin_x100;
  uint8_t fault_wheel;
  char buffer[320];
  int length;
  uint8_t index;

  taskENTER_CRITICAL();
  for (index = 0U; index < 4U; ++index)
  {
    encoder_delta[index] = car_encoder_delta[index];
    encoder_total[index] = car_encoder_total[index];
    rpm_x10[index] = car_measured_rpm_x10[index];
    rpm_abs_x10[index] = CAR_Abs32(rpm_x10[index]);
    target[index] = car_target_rpm[index];
    pwm[index] = car_pwm_command[index];
    encoder_sign[index] = car_encoder_forward_sign[index];
    encoder_locked[index] = car_encoder_polarity_locked[index];
  }
  fault_wheel = car_encoder_fault_wheel;
  taskEXIT_CRITICAL();

  vin_x100 = CAR_ReadVinX100();
  length = snprintf(buffer, sizeof(buffer),
                    "ENC_D:%d,%d,%d,%d ENC_T:%ld,%ld,%ld,%ld "
                    "RPM:%s%ld.%ld,%s%ld.%ld,%s%ld.%ld,%s%ld.%ld "
                    "TARGET:%ld,%ld,%ld,%ld PWM:%d,%d,%d,%d "
                    "ESIGN:%d,%d,%d,%d ECAL:%u,%u,%u,%u FAULT:%u "
                    "VIN:%u.%02uV\r\n",
                    encoder_delta[0], encoder_delta[1],
                    encoder_delta[2], encoder_delta[3],
                    (long)encoder_total[0], (long)encoder_total[1],
                    (long)encoder_total[2], (long)encoder_total[3],
                    (rpm_x10[0] < 0) ? "-" : "",
                    (long)(rpm_abs_x10[0] / 10), (long)(rpm_abs_x10[0] % 10),
                    (rpm_x10[1] < 0) ? "-" : "",
                    (long)(rpm_abs_x10[1] / 10), (long)(rpm_abs_x10[1] % 10),
                    (rpm_x10[2] < 0) ? "-" : "",
                    (long)(rpm_abs_x10[2] / 10), (long)(rpm_abs_x10[2] % 10),
                    (rpm_x10[3] < 0) ? "-" : "",
                    (long)(rpm_abs_x10[3] / 10), (long)(rpm_abs_x10[3] % 10),
                    (long)target[0], (long)target[1],
                    (long)target[2], (long)target[3],
                    pwm[0], pwm[1], pwm[2], pwm[3],
                    encoder_sign[0], encoder_sign[1],
                    encoder_sign[2], encoder_sign[3],
                    (unsigned int)encoder_locked[0],
                    (unsigned int)encoder_locked[1],
                    (unsigned int)encoder_locked[2],
                    (unsigned int)encoder_locked[3],
                    (unsigned int)fault_wheel,
                    (unsigned int)(vin_x100 / 100U),
                    (unsigned int)(vin_x100 % 100U));
  if (length > 0)
  {
    if ((size_t)length >= sizeof(buffer))
    {
      length = (int)(sizeof(buffer) - 1U);
    }
    (void)CAR_QueueRawUart2((const uint8_t *)buffer, (uint16_t)length);
  }

  CAR_SendImuTelemetry();
  CAR_SendUltrasonicTelemetry();
}

static const char *CAR_GetImuStateName(CarImuState_t state)
{
  switch (state)
  {
    case CAR_IMU_INITIALIZING: return "INIT";
    case CAR_IMU_CALIBRATING:  return "CAL";
    case CAR_IMU_READY:        return "READY";
    case CAR_IMU_ERROR:        return "ERROR";
    case CAR_IMU_OFFLINE:
    default:                   return "OFFLINE";
  }
}

static int32_t CAR_FloatToScaled(float value, float scale)
{
  float scaled;

  scaled = value * scale;
  return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void CAR_SendImuTelemetry(void)
{
  CarImuSnapshot_t snapshot;
  uint32_t age_ms;
  int32_t ax_mg;
  int32_t ay_mg;
  int32_t az_mg;
  int32_t gx_mdps;
  int32_t gy_mdps;
  int32_t gz_mdps;
  int32_t roll_cdeg;
  int32_t pitch_cdeg;
  int32_t yaw_cdeg;
  static char buffer[288];
  int length;

  taskENTER_CRITICAL();
  snapshot = car_imu_snapshot;
  taskEXIT_CRITICAL();

  age_ms = (snapshot.sample_tick_ms == 0U) ? 0U :
           (uint32_t)(HAL_GetTick() - snapshot.sample_tick_ms);
  ax_mg = CAR_FloatToScaled(snapshot.data.ax_g, 1000.0f);
  ay_mg = CAR_FloatToScaled(snapshot.data.ay_g, 1000.0f);
  az_mg = CAR_FloatToScaled(snapshot.data.az_g, 1000.0f);
  gx_mdps = CAR_FloatToScaled(snapshot.data.gx_dps, 1000.0f);
  gy_mdps = CAR_FloatToScaled(snapshot.data.gy_dps, 1000.0f);
  gz_mdps = CAR_FloatToScaled(snapshot.data.gz_dps, 1000.0f);
  roll_cdeg = CAR_FloatToScaled(snapshot.data.roll_deg, 100.0f);
  pitch_cdeg = CAR_FloatToScaled(snapshot.data.pitch_deg, 100.0f);
  yaw_cdeg = CAR_FloatToScaled(snapshot.data.yaw_deg, 100.0f);

  length = snprintf(buffer, sizeof(buffer),
                    "IMU:%s WHO:0x%02X SAMPLE:%lu AGE:%lums ERR:%lu "
                    "ACC_mg:%ld,%ld,%ld GYRO_mdps:%ld,%ld,%ld "
                    "ANGLE_cdeg:%ld,%ld,%ld\r\n",
                    CAR_GetImuStateName(snapshot.state),
                    (unsigned int)snapshot.who_am_i,
                    (unsigned long)snapshot.sample_count,
                    (unsigned long)age_ms,
                    (unsigned long)snapshot.error_count,
                    (long)ax_mg, (long)ay_mg, (long)az_mg,
                    (long)gx_mdps, (long)gy_mdps, (long)gz_mdps,
                    (long)roll_cdeg, (long)pitch_cdeg, (long)yaw_cdeg);
  if (length > 0)
  {
    if ((size_t)length >= sizeof(buffer))
    {
      length = (int)(sizeof(buffer) - 1U);
    }
    (void)CAR_QueueRawUart2((const uint8_t *)buffer, (uint16_t)length);
  }
}

static void CAR_SendUltrasonicTelemetry(void)
{
  HC_SR04_Snapshot_t snapshot;
  uint32_t age_ms;
  char buffer[80];
  int length;

  HC_SR04_GetSnapshot(&snapshot);
  age_ms = (snapshot.sample_tick_ms == 0U) ? 0xFFFFFFFFUL :
           (uint32_t)(HAL_GetTick() - snapshot.sample_tick_ms);

  if ((snapshot.state == HC_SR04_STATE_VALID) &&
      (age_ms <= HC_SR04_STALE_MS))
  {
    length = snprintf(buffer, sizeof(buffer),
                      "Distance: %lu.%lu cm\r\n",
                      (unsigned long)(snapshot.distance_mm / 10U),
                      (unsigned long)(snapshot.distance_mm % 10U));
  }
  else if (snapshot.state == HC_SR04_STATE_TIMEOUT)
  {
    length = snprintf(buffer, sizeof(buffer), "Distance: timeout\r\n");
  }
  else
  {
    length = snprintf(buffer, sizeof(buffer), "Distance: waiting\r\n");
  }

  if (length > 0)
  {
    if ((size_t)length >= sizeof(buffer))
    {
      length = (int)(sizeof(buffer) - 1U);
    }
    (void)CAR_QueueRawUart2((const uint8_t *)buffer, (uint16_t)length);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  BaseType_t higher_priority_task_woken;
  size_t sent;
  uint16_t previous;

  if ((huart != &huart2) || (car_uart2_stream_handle == NULL))
  {
    return;
  }

  if (size > CAR_UART2_DMA_BUFFER_SIZE)
  {
    car_uart_rx_fault = 1U;
    car_uart_rx_restart_requested = 1U;
    return;
  }

  higher_priority_task_woken = pdFALSE;
  previous = car_uart2_dma_position;
  if (size > previous)
  {
    sent = xStreamBufferSendFromISR(car_uart2_stream_handle,
                                    &car_uart2_dma_buffer[previous],
                                    (size_t)(size - previous),
                                    &higher_priority_task_woken);
    if (sent != (size_t)(size - previous))
    {
      car_uart_rx_fault = 1U;
      car_uart_rx_restart_requested = 1U;
    }
  }
  else if (size < previous)
  {
    sent = xStreamBufferSendFromISR(car_uart2_stream_handle,
                                    &car_uart2_dma_buffer[previous],
                                    (size_t)(CAR_UART2_DMA_BUFFER_SIZE -
                                             previous),
                                    &higher_priority_task_woken);
    if (sent != (size_t)(CAR_UART2_DMA_BUFFER_SIZE - previous))
    {
      car_uart_rx_fault = 1U;
      car_uart_rx_restart_requested = 1U;
    }
    if (size != 0U)
    {
      sent = xStreamBufferSendFromISR(car_uart2_stream_handle,
                                      car_uart2_dma_buffer, size,
                                      &higher_priority_task_woken);
      if (sent != size)
      {
        car_uart_rx_fault = 1U;
        car_uart_rx_restart_requested = 1U;
      }
    }
  }

  car_uart2_dma_position =
      (size == CAR_UART2_DMA_BUFFER_SIZE) ? 0U : size;
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart == &huart2) && (protocolTxTaskHandle != NULL))
  {
    (void)osThreadFlagsSet(protocolTxTaskHandle,
                           CAR_PROTOCOL_TX_DONE_FLAG);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    car_uart_rx_fault = 1U;
    car_uart_rx_restart_requested = 1U;
    if (protocolTxTaskHandle != NULL)
    {
      (void)osThreadFlagsSet(protocolTxTaskHandle,
                             CAR_PROTOCOL_TX_ERROR_FLAG);
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == MPU_INT_Pin) && (imuTaskHandle != NULL))
  {
    (void)osThreadFlagsSet(imuTaskHandle, CAR_IMU_FLAG_DATA_READY);
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((HC_SR04_HandleCaptureInterrupt(htim) != 0U) &&
      (ultrasonicTaskHandle != NULL))
  {
    (void)osThreadFlagsSet(ultrasonicTaskHandle,
                           CAR_ULTRASONIC_FLAG_CAPTURE_DONE);
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if ((hi2c == &hi2c1) && (imuTaskHandle != NULL))
  {
    (void)osThreadFlagsSet(imuTaskHandle, CAR_IMU_FLAG_DMA_DONE);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if ((hi2c == &hi2c1) && (imuTaskHandle != NULL))
  {
    (void)osThreadFlagsSet(imuTaskHandle, CAR_IMU_FLAG_DMA_ERROR);
  }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  (void)task;
  (void)task_name;
  Emergency_MotorStop();
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationMallocFailedHook(void)
{
  Emergency_MotorStop();
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

/* USER CODE END Application */


