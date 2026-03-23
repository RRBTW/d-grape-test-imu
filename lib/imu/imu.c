/* ============================================================
 *  imu.c — Драйвер MPU-6050 (bare metal версия, без FreeRTOS)
 *  Изменения относительно основного проекта:
 *    - #include "cmsis_os2.h" удалён
 *    - osDelay(ms) → HAL_Delay(ms)
 *    - osKernelGetTickCount() → HAL_GetTick()
 * ============================================================ */

#include "imu.h"
#include "robot_config.h"
#include <string.h>

static HAL_StatusTypeDef mpu_write(I2C_HandleTypeDef *hi2c,
                                   uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c,
                             IMU_I2C_ADDR,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1,
                             IMU_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef imu_init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Read(hi2c,
                          IMU_I2C_ADDR,
                          MPU_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                          &who, 1,
                          IMU_I2C_TIMEOUT_MS);
    if (st != HAL_OK || who != MPU_WHO_AM_I_EXPECTED)
        return HAL_ERROR;

    st = mpu_write(hi2c, MPU_REG_PWR_MGMT_1, 0x80U);
    if (st != HAL_OK) return st;
    HAL_Delay(100);  /* ждём завершения reset (было: osDelay) */

    st = mpu_write(hi2c, MPU_REG_PWR_MGMT_1, 0x01U);
    if (st != HAL_OK) return st;
    HAL_Delay(10);   /* (было: osDelay) */

    st = mpu_write(hi2c, MPU_REG_SMPLRT_DIV, (uint8_t)IMU_SMPLRT_DIV);
    if (st != HAL_OK) return st;

    st = mpu_write(hi2c, MPU_REG_CONFIG, (uint8_t)IMU_DLPF_CFG);
    if (st != HAL_OK) return st;

    st = mpu_write(hi2c, MPU_REG_GYRO_CONFIG, 0x00U);
    if (st != HAL_OK) return st;

    st = mpu_write(hi2c, MPU_REG_ACCEL_CONFIG, 0x00U);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

HAL_StatusTypeDef imu_read(I2C_HandleTypeDef *hi2c, IMU_Raw_t *out)
{
    uint8_t buf[14];

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c,
                                             IMU_I2C_ADDR,
                                             MPU_REG_ACCEL_XOUT_H,
                                             I2C_MEMADD_SIZE_8BIT,
                                             buf, sizeof(buf),
                                             IMU_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        out->valid = 0;
        return st;
    }

    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t raw_t  = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    out->ax = (float)raw_ax * MPU_ACCEL_SCALE;
    out->ay = (float)raw_ay * MPU_ACCEL_SCALE;
    out->az = (float)raw_az * MPU_ACCEL_SCALE;

    out->gx = (float)raw_gx * MPU_GYRO_SCALE;
    out->gy = (float)raw_gy * MPU_GYRO_SCALE;
    out->gz = (float)raw_gz * MPU_GYRO_SCALE;

    out->temp_c = (float)raw_t / 340.0f + 36.53f;

    out->timestamp_ms = HAL_GetTick();  /* было: osKernelGetTickCount() */
    out->valid = 1;

    return HAL_OK;
}
