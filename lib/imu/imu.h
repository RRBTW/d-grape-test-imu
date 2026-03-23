#pragma once

/* ============================================================
 *  imu.h — Драйвер MPU6050 (I2C1)
 *
 *  Подключение (I2C1):
 *    PB6 → I2C1_SCL (AF4)
 *    PB7 → I2C1_SDA (AF4)
 *    AD0 → GND → адрес 0x68
 * ============================================================ */

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── I2C адрес (7-бит сдвинут влево для HAL) ────────────────*/
#define MPU6050_ADDR            (0x68U << 1)

/* ── Регистры ────────────────────────────────────────────────*/
#define MPU6050_WHO_AM_I        0x75U
#define MPU6050_WHO_AM_I_VAL    0x68U
#define MPU6050_PWR_MGMT_1      0x6BU
#define MPU6050_SMPLRT_DIV      0x19U
#define MPU6050_CONFIG          0x1AU
#define MPU6050_GYRO_CONFIG     0x1BU
#define MPU6050_ACCEL_CONFIG    0x1CU
#define MPU6050_ACCEL_XOUT_H    0x3BU
#define MPU6050_GYRO_XOUT_H     0x43U

typedef struct {
    float ax, ay, az;       /* м/с² */
    float gx, gy, gz;       /* рад/с */
    float temp_c;           /* °C */
    uint32_t timestamp_ms;
    uint8_t  valid;
} IMU_Raw_t;

HAL_StatusTypeDef imu_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef imu_read(I2C_HandleTypeDef *hi2c, IMU_Raw_t *out);
uint8_t           imu_whoami(I2C_HandleTypeDef *hi2c);
