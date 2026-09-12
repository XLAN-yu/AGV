#include "obstacle_guard.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
  ObstacleGuard_t g = {0U, 0U, AVG_FAULT_ULTRASONIC_UNAVAILABLE};
  assert(ObstacleGuard_Update(&g, 0U, 0U, 65535U) == 0x31U);
  /* Startup needs 500 ms of fresh readings >= 300 mm. */
  assert(ObstacleGuard_Update(&g, 10U, 1U, 300U) == 0x31U);
  assert(ObstacleGuard_Update(&g, 509U, 1U, 300U) == 0x31U);
  assert(ObstacleGuard_Update(&g, 510U, 1U, 300U) == 0U);
  assert(ObstacleGuard_Update(&g, 520U, 1U, 201U) == 0U);
  assert(ObstacleGuard_Update(&g, 530U, 1U, 200U) == 0x30U);
  /* Hysteresis and interrupted clearance must not release the interlock. */
  assert(ObstacleGuard_Update(&g, 540U, 1U, 299U) == 0x30U);
  assert(ObstacleGuard_Update(&g, 550U, 1U, 300U) == 0x30U);
  assert(ObstacleGuard_Update(&g, 900U, 1U, 250U) == 0x30U);
  assert(ObstacleGuard_Update(&g, 1000U, 1U, 400U) == 0x30U);
  assert(ObstacleGuard_Update(&g, 1499U, 1U, 400U) == 0x30U);
  assert(ObstacleGuard_Update(&g, 1500U, 1U, 400U) == 0U);
  /* Missing/stale echo means unknown, never free space. */
  assert(ObstacleGuard_Update(&g, 1510U, 0U, 400U) == 0x31U);
  assert(ObstacleGuard_Update(&g, 1520U, 1U, 400U) == 0x31U);
  assert(ObstacleGuard_Update(&g, 1600U, 0U, 400U) == 0x31U);
  assert(ObstacleGuard_Update(&g, 1700U, 1U, 400U) == 0x31U);
  assert(ObstacleGuard_Update(&g, 2200U, 1U, 400U) == 0U);
  assert(ObstacleGuard_Update(&g, 2210U, 1U, 0U) == 0x30U);
  /* Millisecond wrap-around must not release early or hang. */
  assert(ObstacleGuard_Update(&g, 0xFFFFFF00U, 1U, 400U) == 0x30U);
  assert(ObstacleGuard_Update(&g, 243U, 1U, 400U) == 0x30U);
  assert(ObstacleGuard_Update(&g, 244U, 1U, 400U) == 0U);
  puts("obstacle guard: threshold, hysteresis, loss, startup, wrap tests passed");
  return 0;
}
