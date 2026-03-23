#pragma once

/* ============================================================
 *  imu.h — Драйвер MPU-6050 (I2C1)
 *
 *  Схема подключения:
 *    PB8 → SCL (I2C1, AF4)
 *    PB9 → SDA (I2C1, AF4)
 *    AD0 → GND → I2C addr = 0x68
 * ============================================================ */

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── Регистры MPU-6050 ──────────────────────────────────────*/
#define MPU_REG_SMPLRT_DIV      0x19U
#define MPU_REG_CONFIG          0x1AU
#define MPU_REG_GYRO_CONFIG     0x1BU
#define MPU_REG_ACCEL_CONFIG    0x1CU
#define MPU_REG_ACCEL_XOUT_H    0x3BU
#define MPU_REG_PWR_MGMT_1      0x6BU
#define MPU_REG_WHO_AM_I        0x75U
#define MPU_WHO_AM_I_EXPECTED   0x68U

#define MPU_ACCEL_SCALE     (9.80665f / 16384.0f)
#define MPU_GYRO_SCALE      (0.01745329f / 131.0f)

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float temp_c;
    uint32_t timestamp_ms;
    uint8_t  valid;
} IMU_Raw_t;

HAL_StatusTypeDef imu_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef imu_read(I2C_HandleTypeDef *hi2c, IMU_Raw_t *out);
