#ifndef AX_MOTOR_H
#define AX_MOTOR_H

#include "main.h"
#include "car_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AX_MOTOR_MAX_COMMAND CAR_PWM_MAX_COMMAND

/* Enable only after confirming a mecanum chassis and the wheel layout below. */
#ifndef AX_MOTOR_ENABLE_STRAFE
#define AX_MOTOR_ENABLE_STRAFE 0
#endif

/*
 * Logical wheel layout used by the chassis helpers:
 *
 *       M1 (front left)       M2 (front right)
 *       M3 (rear left)        M4 (rear right)
 *
 * Vehicle calibration (2026-08-27): all four installed motors use positive
 * electrical polarity for logical vehicle-forward motion.
 * Change only the sign for a wheel that runs backwards during a wheels-off-
 * ground check; do not change the chassis mixing equations.
 */
#ifndef AX_MOTOR_M1_FORWARD_SIGN
#define AX_MOTOR_M1_FORWARD_SIGN CAR_MOTOR_M1_FORWARD_SIGN
#endif
#ifndef AX_MOTOR_M2_FORWARD_SIGN
#define AX_MOTOR_M2_FORWARD_SIGN CAR_MOTOR_M2_FORWARD_SIGN
#endif
#ifndef AX_MOTOR_M3_FORWARD_SIGN
#define AX_MOTOR_M3_FORWARD_SIGN CAR_MOTOR_M3_FORWARD_SIGN
#endif
#ifndef AX_MOTOR_M4_FORWARD_SIGN
#define AX_MOTOR_M4_FORWARD_SIGN CAR_MOTOR_M4_FORWARD_SIGN
#endif

void AX_MOTOR_Init(void);
void AX_MOTOR_SetStandby(uint8_t enable);
void AX_MOTOR_StopAll(void);
void AX_MOTOR_BrakeAll(void);
void AX_MOTOR_CoastAll(void);
void AX_MOTOR_Disable(void);
void AX_MOTOR_A_SetSpeed(int16_t speed);
void AX_MOTOR_B_SetSpeed(int16_t speed);
void AX_MOTOR_C_SetSpeed(int16_t speed);
void AX_MOTOR_D_SetSpeed(int16_t speed);

/* Chassis-level commands.  Positive wheel speed always means vehicle-forward. */
void AX_MOTOR_SetWheelSpeeds(int16_t front_left, int16_t front_right,
                             int16_t rear_left, int16_t rear_right);
void AX_MOTOR_SetDifferential(int16_t left, int16_t right);

/*
 * Mixer inputs use: forward +, strafe-right +, rotate-clockwise +.  Strafe is
 * ignored unless AX_MOTOR_ENABLE_STRAFE is 1, so a normal four-wheel chassis
 * can safely use forward and rotate mixing.
 */
void AX_MOTOR_SetMotion(int16_t forward, int16_t strafe_right,
                        int16_t rotate_clockwise);
void AX_MOTOR_Forward(uint16_t speed);
void AX_MOTOR_Backward(uint16_t speed);
void AX_MOTOR_TurnLeft(uint16_t speed);
void AX_MOTOR_TurnRight(uint16_t speed);
void AX_MOTOR_StrafeLeft(uint16_t speed);
void AX_MOTOR_StrafeRight(uint16_t speed);

#ifdef __cplusplus
}
#endif

#endif
