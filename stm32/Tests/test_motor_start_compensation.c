#include "motor_start_compensation.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
  assert(MotorStartCompensate(20, 0, 400, 15, 50, 1200) == 1200);
  assert(MotorStartCompensate(-20, 0, -400, 15, 50, 1200) == -1200);
  assert(MotorStartCompensate(14, 0, 400, 15, 50, 1200) == 400);
  assert(MotorStartCompensate(20, 50, 400, 15, 50, 1200) == 400);
  assert(MotorStartCompensate(20, 0, 1300, 15, 50, 1200) == 1300);
  assert(MotorStartCompensate(-20, 0, -1300, 15, 50, 1200) == -1300);
  puts("motor start compensation tests passed");
  return 0;
}
