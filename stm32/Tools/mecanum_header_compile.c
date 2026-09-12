#include <assert.h>
#include <stdint.h>
#include "mecanum_mixer.h"

int main(void)
{
  int32_t wheel[4] = {0, 0, 0, 0};
  float velocity[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  CAR_MecanumMixCommand(0, 100, 0, wheel);
  assert(wheel[0] == 100 && wheel[1] == -100 &&
         wheel[2] == -100 && wheel[3] == 100);
  CAR_MecanumMixCommand(0, -100, 0, wheel);
  assert(wheel[0] == -100 && wheel[1] == 100 &&
         wheel[2] == 100 && wheel[3] == -100);
  CAR_MecanumMixVelocity(0.0f, 0.2f, 0.0f, velocity);
  assert(velocity[0] > 0.0f && velocity[1] < 0.0f &&
         velocity[2] < 0.0f && velocity[3] > 0.0f);
  return 0;
}
