#include "ax_motor.h"
#include "mecanum_mixer.h"
#include "tim.h"

#if ((AX_MOTOR_M1_FORWARD_SIGN != 1) && (AX_MOTOR_M1_FORWARD_SIGN != -1)) || \
    ((AX_MOTOR_M2_FORWARD_SIGN != 1) && (AX_MOTOR_M2_FORWARD_SIGN != -1)) || \
    ((AX_MOTOR_M3_FORWARD_SIGN != 1) && (AX_MOTOR_M3_FORWARD_SIGN != -1)) || \
    ((AX_MOTOR_M4_FORWARD_SIGN != 1) && (AX_MOTOR_M4_FORWARD_SIGN != -1))
#error "Each AX_MOTOR_Mx_FORWARD_SIGN must be 1 or -1"
#endif

#if ((AX_MOTOR_ENABLE_STRAFE != 0) && (AX_MOTOR_ENABLE_STRAFE != 1))
#error "AX_MOTOR_ENABLE_STRAFE must be 0 or 1"
#endif

typedef struct
{
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
  uint32_t pwm_channel;
} AX_MotorConfig;

static const AX_MotorConfig motor_config[4] =
{
  {M1_IN1_GPIO_Port, M1_IN1_Pin, M1_IN2_GPIO_Port, M1_IN2_Pin, TIM_CHANNEL_1},
  {M2_IN1_GPIO_Port, M2_IN1_Pin, M2_IN2_GPIO_Port, M2_IN2_Pin, TIM_CHANNEL_2},
  {M3_IN1_GPIO_Port, M3_IN1_Pin, M3_IN2_GPIO_Port, M3_IN2_Pin, TIM_CHANNEL_3},
  {M4_IN1_GPIO_Port, M4_IN1_Pin, M4_IN2_GPIO_Port, M4_IN2_Pin, TIM_CHANNEL_4}
};

static const int8_t motor_forward_sign[4] =
{
  AX_MOTOR_M1_FORWARD_SIGN,
  AX_MOTOR_M2_FORWARD_SIGN,
  AX_MOTOR_M3_FORWARD_SIGN,
  AX_MOTOR_M4_FORWARD_SIGN
};

static int8_t motor_direction[4] = {0, 0, 0, 0};

static int16_t AX_MOTOR_ClampCommand(int32_t command)
{
  if (command > AX_MOTOR_MAX_COMMAND)
  {
    command = AX_MOTOR_MAX_COMMAND;
  }
  else if (command < -AX_MOTOR_MAX_COMMAND)
  {
    command = -AX_MOTOR_MAX_COMMAND;
  }

  return (int16_t)command;
}

static int16_t AX_MOTOR_ClampMagnitude(uint16_t speed)
{
  if (speed > AX_MOTOR_MAX_COMMAND)
  {
    speed = AX_MOTOR_MAX_COMMAND;
  }

  return (int16_t)speed;
}

static uint32_t AX_MOTOR_GetPwmPeriod(void)
{
  return __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
}

static void AX_MOTOR_SetPins(const AX_MotorConfig *motor,
                             GPIO_PinState in1, GPIO_PinState in2)
{
  HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, in1);
  HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, in2);
}

static void AX_MOTOR_WaitForPwmPeriod(void)
{
  if ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0U)
  {
    return;
  }

  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  while (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) == RESET)
  {
    /* TIM1 runs at 20 kHz, so the maximum wait is one 50 us PWM period. */
  }
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
}

static void AX_MOTOR_SetSpeed(uint8_t index, int16_t speed)
{
  uint32_t magnitude;
  uint32_t compare;
  uint32_t pwm_period;
  int8_t requested_direction;
  const AX_MotorConfig *motor;

  if (index >= 4U)
  {
    return;
  }

  motor = &motor_config[index];

  if (speed > 0)
  {
    requested_direction = 1;
  }
  else if (speed < 0)
  {
    requested_direction = -1;
  }
  else
  {
    requested_direction = 0;
  }

  /*
   * CCR writes are preloaded.  Before starting from a stopped/unknown state or
   * performing a true reversal, force the zero compare into hardware and keep
   * the bridge braked for one complete PWM period.  This also prevents the
   * full-duty compare used by CoastAll from leaking into the next drive state.
   */
  __HAL_TIM_SET_COMPARE(&htim1, motor->pwm_channel, 0U);
  if ((requested_direction != 0) &&
      (motor_direction[index] != requested_direction))
  {
    (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    AX_MOTOR_SetPins(motor, GPIO_PIN_SET, GPIO_PIN_SET);
    AX_MOTOR_WaitForPwmPeriod();
  }

  if (speed > 0)
  {
    AX_MOTOR_SetPins(motor, GPIO_PIN_RESET, GPIO_PIN_SET);
    magnitude = (uint32_t)speed;
  }
  else if (speed < 0)
  {
    AX_MOTOR_SetPins(motor, GPIO_PIN_SET, GPIO_PIN_RESET);
    magnitude = (uint32_t)(-(int32_t)speed);
  }
  else
  {
    AX_MOTOR_SetPins(motor, GPIO_PIN_SET, GPIO_PIN_SET);
    motor_direction[index] = 0;
    return;
  }

  if (magnitude > AX_MOTOR_MAX_COMMAND)
  {
    magnitude = AX_MOTOR_MAX_COMMAND;
  }

  pwm_period = AX_MOTOR_GetPwmPeriod();
  compare = (magnitude * pwm_period) / AX_MOTOR_MAX_COMMAND;
  __HAL_TIM_SET_COMPARE(&htim1, motor->pwm_channel, compare);
  motor_direction[index] = requested_direction;
}

void AX_MOTOR_Init(void)
{
  AX_MOTOR_SetStandby(0U);
  AX_MOTOR_BrakeAll();

  if ((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK))
  {
    Error_Handler();
  }

  AX_MOTOR_BrakeAll();
  /* Remain in hardware standby until an explicit motion command is received. */
  AX_MOTOR_SetStandby(0U);
}

void AX_MOTOR_SetStandby(uint8_t enable)
{
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin,
                    (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void AX_MOTOR_StopAll(void)
{
  AX_MOTOR_BrakeAll();
}

void AX_MOTOR_BrakeAll(void)
{
  uint8_t index;

  for (index = 0U; index < 4U; ++index)
  {
    __HAL_TIM_SET_COMPARE(&htim1, motor_config[index].pwm_channel, 0U);
    AX_MOTOR_SetPins(&motor_config[index], GPIO_PIN_SET, GPIO_PIN_SET);
    motor_direction[index] = 0;
  }


  if ((htim1.Instance != NULL) &&
      ((htim1.Instance->CR1 & TIM_CR1_CEN) != 0U))
  {
    (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
  }
}

void AX_MOTOR_CoastAll(void)
{
  uint8_t index;
  uint32_t pwm_period;

  pwm_period = AX_MOTOR_GetPwmPeriod();

  /* Brake first so no channel receives a full-duty drive pulse during changeover. */
  AX_MOTOR_BrakeAll();
  for (index = 0U; index < 4U; ++index)
  {
    AX_MOTOR_SetPins(&motor_config[index], GPIO_PIN_RESET, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, motor_config[index].pwm_channel, pwm_period);
  }
}

void AX_MOTOR_Disable(void)
{
  AX_MOTOR_BrakeAll();
  AX_MOTOR_SetStandby(0U);
}

void AX_MOTOR_A_SetSpeed(int16_t speed)
{
  AX_MOTOR_SetSpeed(0U, speed);
}

void AX_MOTOR_B_SetSpeed(int16_t speed)
{
  AX_MOTOR_SetSpeed(1U, speed);
}

void AX_MOTOR_C_SetSpeed(int16_t speed)
{
  AX_MOTOR_SetSpeed(2U, speed);
}

void AX_MOTOR_D_SetSpeed(int16_t speed)
{
  AX_MOTOR_SetSpeed(3U, speed);
}

void AX_MOTOR_SetWheelSpeeds(int16_t front_left, int16_t front_right,
                             int16_t rear_left, int16_t rear_right)
{
  int16_t wheel_speed[4];
  int8_t requested_direction[4];
  uint32_t compare[4];
  uint32_t magnitude;
  uint32_t pwm_period;
  uint8_t direction_change;
  uint8_t index;

  /* Physical driver channels: A=front-right, B=rear-left, C=front-left, D=rear-right. */
  wheel_speed[0] = front_right;
  wheel_speed[1] = rear_left;
  wheel_speed[2] = front_left;
  wheel_speed[3] = rear_right;

  pwm_period = AX_MOTOR_GetPwmPeriod();
  direction_change = 0U;
  for (index = 0U; index < 4U; ++index)
  {
    wheel_speed[index] = AX_MOTOR_ClampCommand(
        (int32_t)wheel_speed[index] * motor_forward_sign[index]);
    requested_direction[index] = (wheel_speed[index] > 0) ? 1 :
                                 ((wheel_speed[index] < 0) ? -1 : 0);
    if (requested_direction[index] != motor_direction[index])
    {
      direction_change = 1U;
    }
    magnitude = (wheel_speed[index] < 0) ?
                (uint32_t)(-(int32_t)wheel_speed[index]) :
                (uint32_t)wheel_speed[index];
    compare[index] = (magnitude * pwm_period) / AX_MOTOR_MAX_COMMAND;
  }

  /*
   * TIM1 preload makes compare values simultaneous only if no intermediate
   * update event is generated. Batch every wheel transition so a strafe does
   * not momentarily apply one diagonal before the other. A direction change
   * first commits zero duty to all four channels and observes one dead period.
   */
  if (direction_change != 0U)
  {
    for (index = 0U; index < 4U; ++index)
    {
      __HAL_TIM_SET_COMPARE(&htim1, motor_config[index].pwm_channel, 0U);
      AX_MOTOR_SetPins(&motor_config[index], GPIO_PIN_SET, GPIO_PIN_SET);
    }
    (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    AX_MOTOR_WaitForPwmPeriod();
  }

  for (index = 0U; index < 4U; ++index)
  {
    if (requested_direction[index] > 0)
    {
      AX_MOTOR_SetPins(&motor_config[index], GPIO_PIN_RESET, GPIO_PIN_SET);
    }
    else if (requested_direction[index] < 0)
    {
      AX_MOTOR_SetPins(&motor_config[index], GPIO_PIN_SET, GPIO_PIN_RESET);
    }
    else
    {
      AX_MOTOR_SetPins(&motor_config[index], GPIO_PIN_SET, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, motor_config[index].pwm_channel,
                          compare[index]);
    motor_direction[index] = requested_direction[index];
  }
  (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
}

void AX_MOTOR_SetDifferential(int16_t left, int16_t right)
{
  AX_MOTOR_SetWheelSpeeds(left, right, left, right);
}

void AX_MOTOR_SetMotion(int16_t forward, int16_t strafe_right,
                        int16_t rotate_clockwise)
{
  int32_t wheel[4];
  int32_t magnitude;
  int32_t max_magnitude;
  uint8_t index;

#if (AX_MOTOR_ENABLE_STRAFE == 0)
  strafe_right = 0;
#endif

  CAR_MecanumMixCommand((int32_t)forward, (int32_t)strafe_right,
                         (int32_t)rotate_clockwise, wheel);

  max_magnitude = 0;
  for (index = 0U; index < 4U; ++index)
  {
    magnitude = (wheel[index] < 0) ? -wheel[index] : wheel[index];
    if (magnitude > max_magnitude)
    {
      max_magnitude = magnitude;
    }
  }

  /* Preserve the requested motion ratio while keeping every wheel in range. */
  if (max_magnitude > AX_MOTOR_MAX_COMMAND)
  {
    for (index = 0U; index < 4U; ++index)
    {
      wheel[index] = (wheel[index] * AX_MOTOR_MAX_COMMAND) / max_magnitude;
    }
  }

  AX_MOTOR_SetWheelSpeeds((int16_t)wheel[0], (int16_t)wheel[1],
                           (int16_t)wheel[2], (int16_t)wheel[3]);
}

void AX_MOTOR_Forward(uint16_t speed)
{
  int16_t command;

  command = AX_MOTOR_ClampMagnitude(speed);
  AX_MOTOR_SetDifferential(command, command);
}

void AX_MOTOR_Backward(uint16_t speed)
{
  int16_t command;

  command = AX_MOTOR_ClampMagnitude(speed);
  AX_MOTOR_SetDifferential((int16_t)-command, (int16_t)-command);
}

void AX_MOTOR_TurnLeft(uint16_t speed)
{
  int16_t command;

  command = AX_MOTOR_ClampMagnitude(speed);
  AX_MOTOR_SetDifferential((int16_t)-command, command);
}

void AX_MOTOR_TurnRight(uint16_t speed)
{
  int16_t command;

  command = AX_MOTOR_ClampMagnitude(speed);
  AX_MOTOR_SetDifferential(command, (int16_t)-command);
}

void AX_MOTOR_StrafeLeft(uint16_t speed)
{
  int16_t command;

  command = AX_MOTOR_ClampMagnitude(speed);
  AX_MOTOR_SetMotion(0, (int16_t)-command, 0);
}

void AX_MOTOR_StrafeRight(uint16_t speed)
{
  int16_t command;

  command = AX_MOTOR_ClampMagnitude(speed);
  AX_MOTOR_SetMotion(0, command, 0);
}

