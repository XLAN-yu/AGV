#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MPU6500 is register-compatible with this driver and reports WHO_AM_I 0x70. */
#define MPU6050_SAMPLE_PERIOD_MS 10U
#define MPU6050_CALIBRATION_SAMPLES 200U

/* Correct temperature-induced residual gyro bias only while the chassis is
   independently known to be stationary. At 100 Hz this is a gentle 2 s
   adaptation, not a substitute for a magnetometer. */
#define MPU6050_STATIONARY_BIAS_GAIN 0.005f
#define MPU6050_STATIONARY_GYRO_LIMIT_DPS 1.5f

typedef struct
{
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  float gx_offset_dps;
  float gy_offset_dps;
  float gz_offset_dps;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float roll_reference_deg;
  float pitch_reference_deg;
  float yaw_reference_deg;
} MPU6050_Data_t;

HAL_StatusTypeDef MPU6050_Init(uint8_t *who_am_i);
HAL_StatusTypeDef MPU6050_ReadBlocking(MPU6050_Data_t *imu);
HAL_StatusTypeDef MPU6050_StartReadDMA(void);
void MPU6050_ProcessDmaSample(MPU6050_Data_t *imu, float dt_s,
                              uint8_t stationary);
void MPU6050_SetInitialAttitude(MPU6050_Data_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
