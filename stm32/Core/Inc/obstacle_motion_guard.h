#ifndef OBSTACLE_MOTION_GUARD_H
#define OBSTACLE_MOTION_GUARD_H

#include <stdint.h>

#include "avg_protocol.h"
#include "car_config.h"

/*
 * Directional policy for a front-only sensor.  A near obstacle permits only a
 * slow, straight retreat.  An unavailable sensor remains a hard interlock.
 */
static inline uint8_t ObstacleMotionGuard_Apply(uint8_t guard_enabled,
                                                uint8_t obstacle_fault,
                                                float *linear_mps,
                                                float lateral_mps,
                                                float angular_rps)
{
  if ((linear_mps == NULL) || (guard_enabled == 0U) ||
      (obstacle_fault == AVG_FAULT_NONE))
  {
    return (linear_mps != NULL) ? 1U : 0U;
  }
  if (obstacle_fault != AVG_FAULT_OBSTACLE)
  {
    return 0U;
  }
  if ((*linear_mps == 0.0f) && (lateral_mps == 0.0f) &&
      (angular_rps == 0.0f))
  {
    return 1U;
  }
  if ((*linear_mps < 0.0f) && (lateral_mps == 0.0f) &&
      (angular_rps == 0.0f))
  {
    if (*linear_mps < -CAR_OBSTACLE_RETREAT_MAX_MPS)
    {
      *linear_mps = -CAR_OBSTACLE_RETREAT_MAX_MPS;
    }
    return 1U;
  }
  return 0U;
}

#endif
