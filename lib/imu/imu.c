/* ============================================================
 *  imu.c — Драйвер MPU6050 (I2C1)
 *  STM32F407VG Discovery
 * ============================================================ */

#include "imu.h"
#include <string.h>

/* ±2g  → м/с²:   1 LSB = 1/16384 g = 9.80665/16384 м/с² */
#define ACCEL_SCALE  (9.80665f / 16384.0f)
/* ±250°/с → рад/с: 1 LSB = 1/131 °/с → × π/180 */
#define GYRO_SCALE   (0.017453293f / 131.0f)

/* ── Низкоуровневые I2C функции ─────────────────────────────*/
static HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buf, 2, 10);
}

static HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg,
                                       uint8_t *buf, uint8_t len)
{
    HAL_StatusTypeDef st =
        HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, &reg, 1, 10);
    if (st != HAL_OK) return st;
    return HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, buf, len, 10);
}

/* ── Инициализация ──────────────────────────────────────────*/
HAL_StatusTypeDef imu_init(I2C_HandleTypeDef *hi2c)
{
    if (imu_whoami(hi2c) != MPU6050_WHO_AM_I_VAL) return HAL_ERROR;

    /* Выход из сна: SLEEP=0, CLKSEL=1 (PLL по гироскопу X) */
    if (i2c_write_reg(hi2c, MPU6050_PWR_MGMT_1, 0x01U) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    /* Sample Rate = 1000 / (1 + 9) = 100 Гц */
    if (i2c_write_reg(hi2c, MPU6050_SMPLRT_DIV, 9U) != HAL_OK)
        return HAL_ERROR;

    /* DLPF: полоса ~94 Гц */
    if (i2c_write_reg(hi2c, MPU6050_CONFIG, 0x02U) != HAL_OK)
        return HAL_ERROR;

    /* Акселерометр: ±2g (16384 LSB/g) */
    if (i2c_write_reg(hi2c, MPU6050_ACCEL_CONFIG, 0x00U) != HAL_OK)
        return HAL_ERROR;

    /* Гироскоп: ±250°/с (131 LSB/°/с) */
    if (i2c_write_reg(hi2c, MPU6050_GYRO_CONFIG, 0x00U) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);
    return HAL_OK;
}

uint8_t imu_whoami(I2C_HandleTypeDef *hi2c)
{
    uint8_t val = 0;
    i2c_read_regs(hi2c, MPU6050_WHO_AM_I, &val, 1);
    return val;
}

/* ── Чтение данных ──────────────────────────────────────────*/
HAL_StatusTypeDef imu_read(I2C_HandleTypeDef *hi2c, IMU_Raw_t *out)
{
    memset(out, 0, sizeof(*out));
    out->timestamp_ms = HAL_GetTick();

    /* Burst-чтение 14 байт: ACCEL(6) + TEMP(2) + GYRO(6) */
    uint8_t buf[14];
    HAL_StatusTypeDef st = i2c_read_regs(hi2c, MPU6050_ACCEL_XOUT_H, buf, 14);
    if (st != HAL_OK) return st;

    int16_t raw_ax = (int16_t)((uint16_t)(buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((uint16_t)(buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((uint16_t)(buf[4]  << 8) | buf[5]);
    int16_t raw_t  = (int16_t)((uint16_t)(buf[6]  << 8) | buf[7]);
    int16_t raw_gx = (int16_t)((uint16_t)(buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((uint16_t)(buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((uint16_t)(buf[12] << 8) | buf[13]);

    out->ax = (float)raw_ax * ACCEL_SCALE;
    out->ay = (float)raw_ay * ACCEL_SCALE;
    out->az = (float)raw_az * ACCEL_SCALE;
    out->gx = (float)raw_gx * GYRO_SCALE;
    out->gy = (float)raw_gy * GYRO_SCALE;
    out->gz = (float)raw_gz * GYRO_SCALE;
    out->temp_c = (float)raw_t / 340.0f + 36.53f;
    out->valid  = 1;
    return HAL_OK;
}
