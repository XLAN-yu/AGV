#ifndef OBSTACLE_GUARD_H
#define OBSTACLE_GUARD_H

#include "car_config.h"
#include "avg_protocol.h"

/* Pure state machine, owned by the safety task. No motor or GPIO access. */
typedef struct
{
  uint32_t clear_since;
  uint8_t clearing;
  uint8_t fault;
} ObstacleGuard_t;

static inline uint8_t ObstacleGuard_Update(ObstacleGuard_t *guard,
                                          uint32_t now,
                                          uint8_t fresh_valid,
                                          uint16_t distance_mm)
{
  if (fresh_valid == 0U)
  {
    guard->fault = AVG_FAULT_ULTRASONIC_UNAVAILABLE;
    guard->clearing = 0U;
  }
  else if (distance_mm <= CAR_ULTRASONIC_STOP_MM)
  {
    guard->fault = AVG_FAULT_OBSTACLE;
    guard->clearing = 0U;
  }
  else if (guard->fault != AVG_FAULT_NONE)
  {
    if (distance_mm < CAR_ULTRASONIC_RELEASE_MM)
    {
      guard->clearing = 0U;
    }
    else if (guard->clearing == 0U)
    {
      guard->clearing = 1U;
      guard->clear_since = now;
    }
    else if ((uint32_t)(now - guard->clear_since) >=
             CAR_ULTRASONIC_RELEASE_HOLD_MS)
    {
      guard->fault = AVG_FAULT_NONE;
      guard->clearing = 0U;
    }
  }
  return guard->fault;
}

#endif
