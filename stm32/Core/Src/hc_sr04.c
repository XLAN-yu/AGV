#include "hc_sr04.h"

#include "main.h"

/* TIM9 is configured by AVG_V1.ioc for 168 MHz / 168 = 1 MHz. */
static volatile uint16_t hc_sr04_rising_edge = 0U;
static volatile uint8_t hc_sr04_waiting_for_falling_edge = 0U;
static volatile uint8_t hc_sr04_measurement_active = 0U;
static volatile HC_SR04_Snapshot_t hc_sr04_snapshot = {
  .state = HC_SR04_STATE_OFFLINE,
  .distance_mm = HC_SR04_UNAVAILABLE_MM
};

static void HC_SR04_DelayUs(uint16_t delay_us)
{
  uint16_t start;

  start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim9);
  while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim9) - start) <
         delay_us)
  {
  }
}

static void HC_SR04_SelectRisingEdge(void)
{
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim9, TIM_CHANNEL_1,
                                TIM_INPUTCHANNELPOLARITY_RISING);
}

HAL_StatusTypeDef HC_SR04_Init(void)
{
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
  hc_sr04_waiting_for_falling_edge = 0U;
  hc_sr04_measurement_active = 0U;
  hc_sr04_snapshot.state = HC_SR04_STATE_OFFLINE;
  hc_sr04_snapshot.distance_mm = HC_SR04_UNAVAILABLE_MM;
  hc_sr04_snapshot.pulse_us = 0U;
  hc_sr04_snapshot.sample_tick_ms = 0U;
  hc_sr04_snapshot.sample_count = 0U;
  hc_sr04_snapshot.timeout_count = 0U;
  HC_SR04_SelectRisingEdge();
  __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_CC1);
  return HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
}

HAL_StatusTypeDef HC_SR04_Trigger(void)
{
  if (hc_sr04_measurement_active != 0U)
  {
    return HAL_BUSY;
  }

  __HAL_TIM_DISABLE_IT(&htim9, TIM_IT_CC1);
  hc_sr04_waiting_for_falling_edge = 0U;
  HC_SR04_SelectRisingEdge();
  __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_CC1);
  if (hc_sr04_snapshot.state == HC_SR04_STATE_OFFLINE)
  {
    hc_sr04_snapshot.state = HC_SR04_STATE_WAITING;
  }
  hc_sr04_measurement_active = 1U;
  __HAL_TIM_ENABLE_IT(&htim9, TIM_IT_CC1);

  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
  HC_SR04_DelayUs(2U);
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
  HC_SR04_DelayUs(10U);
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
  return HAL_OK;
}

void HC_SR04_MarkTimeout(void)
{
  __HAL_TIM_DISABLE_IT(&htim9, TIM_IT_CC1);
  hc_sr04_measurement_active = 0U;
  hc_sr04_waiting_for_falling_edge = 0U;
  HC_SR04_SelectRisingEdge();
  __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_CC1);
  hc_sr04_snapshot.state = HC_SR04_STATE_TIMEOUT;
  hc_sr04_snapshot.distance_mm = HC_SR04_UNAVAILABLE_MM;
  hc_sr04_snapshot.pulse_us = 0U;
  hc_sr04_snapshot.sample_tick_ms = HAL_GetTick();
  ++hc_sr04_snapshot.timeout_count;
  __HAL_TIM_ENABLE_IT(&htim9, TIM_IT_CC1);
}

uint8_t HC_SR04_HandleCaptureInterrupt(TIM_HandleTypeDef *htim)
{
  uint16_t captured_count;
  uint32_t pulse_us;
  uint32_t distance_mm;

  if ((htim == NULL) || (htim->Instance != TIM9) ||
      (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) ||
      (hc_sr04_measurement_active == 0U))
  {
    return 0U;
  }

  captured_count = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
  if (hc_sr04_waiting_for_falling_edge == 0U)
  {
    hc_sr04_rising_edge = captured_count;
    hc_sr04_waiting_for_falling_edge = 1U;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                  TIM_INPUTCHANNELPOLARITY_FALLING);
    return 0U;
  }

  /* Unsigned subtraction handles one 16-bit TIM9 counter wrap. */
  pulse_us = (uint16_t)(captured_count - hc_sr04_rising_edge);
  distance_mm = (pulse_us * 10U + 29U) / 58U;
  if (distance_mm >= HC_SR04_UNAVAILABLE_MM)
  {
    distance_mm = HC_SR04_UNAVAILABLE_MM - 1U;
  }

  hc_sr04_waiting_for_falling_edge = 0U;
  hc_sr04_measurement_active = 0U;
  HC_SR04_SelectRisingEdge();
  hc_sr04_snapshot.pulse_us = pulse_us;
  hc_sr04_snapshot.distance_mm = (uint16_t)distance_mm;
  hc_sr04_snapshot.sample_tick_ms = HAL_GetTick();
  ++hc_sr04_snapshot.sample_count;
  hc_sr04_snapshot.state = HC_SR04_STATE_VALID;
  return 1U;
}

void HC_SR04_GetSnapshot(HC_SR04_Snapshot_t *snapshot)
{
  uint32_t interrupt_mask;

  if (snapshot == NULL)
  {
    return;
  }

  interrupt_mask = __get_PRIMASK();
  __disable_irq();
  snapshot->state = hc_sr04_snapshot.state;
  snapshot->distance_mm = hc_sr04_snapshot.distance_mm;
  snapshot->pulse_us = hc_sr04_snapshot.pulse_us;
  snapshot->sample_tick_ms = hc_sr04_snapshot.sample_tick_ms;
  snapshot->sample_count = hc_sr04_snapshot.sample_count;
  snapshot->timeout_count = hc_sr04_snapshot.timeout_count;
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}
