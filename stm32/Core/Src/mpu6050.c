#include "mpu6050.h"
#include "i2c.h"
#include <math.h>

/* AD0 is connected to GND, so the 7-bit I2C address is 0x68. */
#define MPU6050_ADDR                (0x68U << 1U)
#define MPU6050_I2C_TIMEOUT_MS      100U

#define MPU6050_WHO_AM_I            0x75U
#define MPU6050_SMPLRT_DIV          0x19U
#define MPU6050_CONFIG              0x1AU
#define MPU6050_GYRO_CONFIG         0x1BU
#define MPU6050_ACCEL_CONFIG        0x1CU
#define MPU6050_ACCEL_XOUT_H        0x3BU
#define MPU6050_INT_PIN_CFG         0x37U
#define MPU6050_INT_ENABLE          0x38U
#define MPU6050_PWR_MGMT_1          0x6BU
#define MPU6050_PWR_MGMT_2          0x6CU

#define ACCEL_LSB_PER_G             16384.0f
#define GYRO_LSB_PER_DPS            131.0f
#define RAD_TO_DEG                  57.2957795f
#define COMPLEMENTARY_FILTER_ALPHA  0.98f

/* The DMA destination must remain valid until HAL_I2C_MemRxCpltCallback. */
static uint8_t mpu6050_dma_buffer[14];

static int16_t MPU6050_BytesToInt16(uint8_t high_byte, uint8_t low_byte)
{
  return (int16_t)(((uint16_t)high_byte << 8U) | low_byte);
}

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           MPU6050_I2C_TIMEOUT_MS);
}

static void MPU6050_ParseRawData(MPU6050_Data_t *imu, const uint8_t raw[14])
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;

  ax = MPU6050_BytesToInt16(raw[0], raw[1]);
  ay = MPU6050_BytesToInt16(raw[2], raw[3]);
  az = MPU6050_BytesToInt16(raw[4], raw[5]);
  gx = MPU6050_BytesToInt16(raw[8], raw[9]);
  gy = MPU6050_BytesToInt16(raw[10], raw[11]);
  gz = MPU6050_BytesToInt16(raw[12], raw[13]);

  imu->ax_g = (float)ax / ACCEL_LSB_PER_G;
  imu->ay_g = (float)ay / ACCEL_LSB_PER_G;
  imu->az_g = (float)az / ACCEL_LSB_PER_G;
  imu->gx_dps = ((float)gx / GYRO_LSB_PER_DPS) - imu->gx_offset_dps;
  imu->gy_dps = ((float)gy / GYRO_LSB_PER_DPS) - imu->gy_offset_dps;
  imu->gz_dps = ((float)gz / GYRO_LSB_PER_DPS) - imu->gz_offset_dps;
}

static void MPU6050_UpdateAttitude(MPU6050_Data_t *imu, float dt_s)
{
  float roll_acc_deg;
  float pitch_acc_deg;

  if (dt_s < 0.001f)
  {
    dt_s = 0.001f;
  }
  else if (dt_s > 0.050f)
  {
    dt_s = 0.050f;
  }

  roll_acc_deg = atan2f(imu->ay_g, imu->az_g) * RAD_TO_DEG;
  pitch_acc_deg = atan2f(-imu->ax_g,
                         sqrtf(imu->ay_g * imu->ay_g +
                               imu->az_g * imu->az_g)) * RAD_TO_DEG;

  imu->roll_deg = COMPLEMENTARY_FILTER_ALPHA *
                  (imu->roll_deg + imu->gx_dps * dt_s) +
                  (1.0f - COMPLEMENTARY_FILTER_ALPHA) * roll_acc_deg;
  imu->pitch_deg = COMPLEMENTARY_FILTER_ALPHA *
                   (imu->pitch_deg + imu->gy_dps * dt_s) +
                   (1.0f - COMPLEMENTARY_FILTER_ALPHA) * pitch_acc_deg;
  /* MPU6500 has no magnetometer; yaw is integrated and needs a stable bias. */
  imu->yaw_deg += imu->gz_dps * dt_s;
}

static void MPU6050_AdaptBiasWhileStationary(MPU6050_Data_t *imu)
{
  float correction;

  if ((fabsf(imu->gx_dps) > MPU6050_STATIONARY_GYRO_LIMIT_DPS) ||
      (fabsf(imu->gy_dps) > MPU6050_STATIONARY_GYRO_LIMIT_DPS) ||
      (fabsf(imu->gz_dps) > MPU6050_STATIONARY_GYRO_LIMIT_DPS))
  {
    return;
  }

  correction = imu->gx_dps * MPU6050_STATIONARY_BIAS_GAIN;
  imu->gx_offset_dps += correction;
  imu->gx_dps -= correction;
  correction = imu->gy_dps * MPU6050_STATIONARY_BIAS_GAIN;
  imu->gy_offset_dps += correction;
  imu->gy_dps -= correction;
  correction = imu->gz_dps * MPU6050_STATIONARY_BIAS_GAIN;
  imu->gz_offset_dps += correction;
  imu->gz_dps -= correction;
}

HAL_StatusTypeDef MPU6050_Init(uint8_t *who_am_i)
{
  if ((who_am_i == NULL) ||
      (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3U,
                             MPU6050_I2C_TIMEOUT_MS) != HAL_OK) ||
      (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I,
                        I2C_MEMADD_SIZE_8BIT, who_am_i, 1U,
                        MPU6050_I2C_TIMEOUT_MS) != HAL_OK))
  {
    return HAL_ERROR;
  }

  if ((*who_am_i != 0x68U) && (*who_am_i != 0x69U) &&
      (*who_am_i != 0x70U))
  {
    return HAL_ERROR;
  }

  /* Wake the device and use the X gyro PLL as its clock reference. */
  if (MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x01U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(100U);

  if ((MPU6050_WriteRegister(MPU6050_PWR_MGMT_2, 0x00U) != HAL_OK) ||
      /* DLPF 44 Hz; 1 kHz / (1 + 9) = 100 Hz sample rate. */
      (MPU6050_WriteRegister(MPU6050_CONFIG, 0x03U) != HAL_OK) ||
      (MPU6050_WriteRegister(MPU6050_SMPLRT_DIV, 9U) != HAL_OK) ||
      /* Gyro +/-250 dps and accelerometer +/-2 g. */
      (MPU6050_WriteRegister(MPU6050_GYRO_CONFIG, 0x00U) != HAL_OK) ||
      (MPU6050_WriteRegister(MPU6050_ACCEL_CONFIG, 0x00U) != HAL_OK) ||
      /* Active-high push-pull data-ready interrupt; clear on data read. */
      (MPU6050_WriteRegister(MPU6050_INT_PIN_CFG, 0x10U) != HAL_OK) ||
      (MPU6050_WriteRegister(MPU6050_INT_ENABLE, 0x01U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadBlocking(MPU6050_Data_t *imu)
{
  uint8_t raw[14];

  if ((imu == NULL) ||
      (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                        I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw),
                        MPU6050_I2C_TIMEOUT_MS) != HAL_OK))
  {
    return HAL_ERROR;
  }

  MPU6050_ParseRawData(imu, raw);
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_StartReadDMA(void)
{
  return HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT, mpu6050_dma_buffer,
                              sizeof(mpu6050_dma_buffer));
}

void MPU6050_ProcessDmaSample(MPU6050_Data_t *imu, float dt_s,
                              uint8_t stationary)
{
  if (imu == NULL)
  {
    return;
  }

  MPU6050_ParseRawData(imu, mpu6050_dma_buffer);
  if (stationary != 0U)
  {
    MPU6050_AdaptBiasWhileStationary(imu);
  }
  MPU6050_UpdateAttitude(imu, dt_s);
}

void MPU6050_SetInitialAttitude(MPU6050_Data_t *imu)
{
  if (imu == NULL)
  {
    return;
  }

  imu->roll_deg = atan2f(imu->ay_g, imu->az_g) * RAD_TO_DEG;
  imu->pitch_deg = atan2f(-imu->ax_g,
                          sqrtf(imu->ay_g * imu->ay_g +
                                imu->az_g * imu->az_g)) * RAD_TO_DEG;
  imu->yaw_deg = 0.0f;
  imu->roll_reference_deg = imu->roll_deg;
  imu->pitch_reference_deg = imu->pitch_deg;
  imu->yaw_reference_deg = imu->yaw_deg;
}
