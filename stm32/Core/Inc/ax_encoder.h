#ifndef AX_ENCODER_H
#define AX_ENCODER_H

#include "main.h"
#include "car_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Encoder calibration used by the wheel-speed controller.
 *
 * COUNTS_PER_WHEEL_REV is the TIM counter increment for one complete wheel
 * revolution.  TIM_ENCODERMODE_TI12 counts all four A/B edges, so include the
 * quadrature x4 factor and the gearbox ratio.  The supplied 1560-count value
 * matches a 13-PPR encoder with a 30:1 gearbox (13 * 4 * 30); change it to the
 * measured value when different hardware is fitted.
 *
 * A positive count must mean vehicle-forward for every wheel.  Change only
 * the corresponding sign after a wheels-off-ground direction check.
 */
#ifndef AX_ENCODER_COUNTS_PER_WHEEL_REV
#define AX_ENCODER_COUNTS_PER_WHEEL_REV CAR_ENCODER_COUNTS_PER_WHEEL_REV
#endif

#ifndef AX_ENCODER_M1_FORWARD_SIGN
#define AX_ENCODER_M1_FORWARD_SIGN CAR_ENCODER_M1_FORWARD_SIGN
#endif
#ifndef AX_ENCODER_M2_FORWARD_SIGN
#define AX_ENCODER_M2_FORWARD_SIGN CAR_ENCODER_M2_FORWARD_SIGN
#endif
#ifndef AX_ENCODER_M3_FORWARD_SIGN
#define AX_ENCODER_M3_FORWARD_SIGN CAR_ENCODER_M3_FORWARD_SIGN
#endif
#ifndef AX_ENCODER_M4_FORWARD_SIGN
#define AX_ENCODER_M4_FORWARD_SIGN CAR_ENCODER_M4_FORWARD_SIGN
#endif

void AX_ENCODER_A_Init(void);
int16_t AX_ENCODER_A_GetCounter(void);
void AX_ENCODER_A_SetCounter(int16_t count);

void AX_ENCODER_B_Init(void);
int16_t AX_ENCODER_B_GetCounter(void);
void AX_ENCODER_B_SetCounter(int16_t count);

void AX_ENCODER_C_Init(void);
int16_t AX_ENCODER_C_GetCounter(void);
void AX_ENCODER_C_SetCounter(int16_t count);

void AX_ENCODER_D_Init(void);
int16_t AX_ENCODER_D_GetCounter(void);
void AX_ENCODER_D_SetCounter(int16_t count);

#ifdef __cplusplus
}
#endif

#endif
