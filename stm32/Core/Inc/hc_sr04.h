#ifndef HC_SR04_H
#define HC_SR04_H

#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HC_SR04_TRIGGER_PERIOD_MS  60U
#define HC_SR04_ECHO_TIMEOUT_MS    40U
#define HC_SR04_STALE_MS           250U
#define HC_SR04_UNAVAILABLE_MM     0xFFFFU

typedef enum
{
  HC_SR04_STATE_OFFLINE = 0,
  HC_SR04_STATE_WAITING,
  HC_SR04_STATE_VALID,
  HC_SR04_STATE_TIMEOUT
} HC_SR04_State_t;

typedef struct
{
  HC_SR04_State_t state;
  uint16_t distance_mm;
  uint32_t pulse_us;
  uint32_t sample_tick_ms;
  uint32_t sample_count;
  uint32_t timeout_count;
} HC_SR04_Snapshot_t;

HAL_StatusTypeDef HC_SR04_Init(void);
HAL_StatusTypeDef HC_SR04_Trigger(void);
void HC_SR04_MarkTimeout(void);
uint8_t HC_SR04_HandleCaptureInterrupt(TIM_HandleTypeDef *htim);
void HC_SR04_GetSnapshot(HC_SR04_Snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* HC_SR04_H */
