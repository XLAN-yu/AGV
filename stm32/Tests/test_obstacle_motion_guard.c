#include "obstacle_motion_guard.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
  float linear;

  linear = 0.4f;
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_NONE, &linear,
                                   0.3f, 1.0f) == 1U);
  linear = -0.4f;
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_OBSTACLE, &linear,
                                   0.0f, 0.0f) == 1U);
  assert(linear == -CAR_OBSTACLE_RETREAT_MAX_MPS);
  linear = -0.1f;
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_OBSTACLE, &linear,
                                   0.0f, 0.0f) == 1U);
  assert(linear == -0.1f);
  linear = 0.0f;
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_OBSTACLE, &linear,
                                   0.0f, 0.0f) == 1U);
  linear = 0.1f;
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_OBSTACLE, &linear,
                                   0.0f, 0.0f) == 0U);
  linear = -0.1f;
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_OBSTACLE, &linear,
                                   0.01f, 0.0f) == 0U);
  linear = -0.1f;
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_OBSTACLE, &linear,
                                   0.0f, 0.01f) == 0U);
  linear = -0.1f;
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_ULTRASONIC_UNAVAILABLE,
                                   &linear, 0.0f, 0.0f) == 0U);
  linear = 0.4f;
  assert(ObstacleMotionGuard_Apply(0U, AVG_FAULT_OBSTACLE, &linear,
                                   0.2f, 1.0f) == 1U);
  assert(ObstacleMotionGuard_Apply(1U, AVG_FAULT_OBSTACLE, NULL,
                                   0.0f, 0.0f) == 0U);
  puts("obstacle motion guard: retreat-only policy passed");
  return 0;
}
