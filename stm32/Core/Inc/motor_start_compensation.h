#ifndef MOTOR_START_COMPENSATION_H
#define MOTOR_START_COMPENSATION_H

#include <stdint.h>

/*
 * Raise only the initial low-speed command above static friction. Once the
 * encoder confirms motion, the normal PID regains full control. The target
 * threshold prevents tiny joystick values from producing an unexpected kick.
 */
static inline int16_t MotorStartCompensate(int32_t target_rpm,
                                           int32_t measured_rpm_x10,
                                           int16_t command,
                                           uint16_t minimum_target_rpm,
                                           uint16_t moving_rpm_x10,
                                           uint16_t minimum_command)
{
  int32_t absolute_target = (target_rpm < 0) ? -target_rpm : target_rpm;
  int32_t absolute_measured = (measured_rpm_x10 < 0) ?
                              -measured_rpm_x10 : measured_rpm_x10;
  int32_t absolute_command = (command < 0) ? -(int32_t)command : command;

  if ((absolute_target < (int32_t)minimum_target_rpm) ||
      (absolute_measured >= (int32_t)moving_rpm_x10) ||
      (absolute_command >= (int32_t)minimum_command))
  {
    return command;
  }

  return (target_rpm > 0) ? (int16_t)minimum_command :
                            (int16_t)(-(int32_t)minimum_command);
}

#endif /* MOTOR_START_COMPENSATION_H */
