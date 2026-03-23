/* ============================================================
 *  main.c — d-grape-test-imu
 *  Проверка MPU-6050: вывод ax/ay/az/gx/gy/gz/T в USB CDC
 *
 *  Ожидаемые значения в покое (плата горизонтально):
 *    az ≈ ±9.81 м/с²   (зависит от ориентации)
 *    gx,gy,gz ≈ 0      (небольшой шум)
 *    T ≈ 25-40 °C
 * ============================================================ */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "imu.h"
#include <string.h>
#include <stdio.h>

/* ── HAL handles ────────────────────────────────────────────*/
I2C_HandleTypeDef hi2c1;

/* ── Вспомогательные функции ────────────────────────────────*/
static void cdc_send(const char *s, uint16_t len)
{
    if (len == 0U || len > CDC_TX_BUF_SIZE) return;
    memcpy(UserTxBufferFS, s, len);
    uint32_t t = HAL_GetTick();
    while (CDC_Transmit_FS(UserTxBufferFS, len) == USBD_BUSY) {
        if (HAL_GetTick() - t > 30U) break;
    }
}

/* ── Прототипы ──────────────────────────────────────────────*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

void Error_Handler(void)
{
    GPIOE->BSRR = GPIO_PIN_1;
    while (1) {}
}

/* ── Точка входа ────────────────────────────────────────────*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USB_DEVICE_Init();
    HAL_Delay(500U);

    if (imu_init(&hi2c1) != HAL_OK) {
        const char *err = "IMU init FAILED — проверь подключение MPU-6050\r\n";
        cdc_send(err, (uint16_t)strlen(err));
        GPIOE->BSRR = GPIO_PIN_1;  /* PE1 красный = ошибка */
        while (1) {}
    }

    const char *banner =
        "\r\n=== d-grape-test-imu: MPU-6050 ===\r\n"
        "ax[m/s2]  ay       az       gx[rad/s] gy       gz       T[C]\r\n"
        "--------------------------------------------------------------------\r\n";
    cdc_send(banner, (uint16_t)strlen(banner));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); /* зелёный = IMU OK */

    IMU_Raw_t imu = {0};
    uint32_t last_read  = HAL_GetTick() - 10U;
    uint32_t last_print = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();

        /* Опрос 100 Гц */
        if (now - last_read >= 10U) {
            imu_read(&hi2c1, &imu);
            last_read = now;
        }

        /* Вывод 10 Гц */
        if (now - last_print >= 100U) {
            last_print = now;
            char buf[128];
            int n = snprintf(buf, sizeof(buf),
                "%7.3f   %7.3f   %7.3f   %7.4f   %7.4f   %7.4f   %5.1f\r\n",
                (double)imu.ax, (double)imu.ay, (double)imu.az,
                (double)imu.gx, (double)imu.gy, (double)imu.gz,
                (double)imu.temp_c);
            if (n > 0) cdc_send(buf, (uint16_t)n);
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); /* зелёный heartbeat */
        }
    }
}

/* ── Тактирование 168 МГц ───────────────────────────────────*/
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM            = 8;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 7;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/* ── GPIO: LED + DIR pins ───────────────────────────────────*/
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);
    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &gpio);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
}

/* ── I2C1: MPU-6050, PB8/PB9, 400 кГц ─────────────────────*/
static void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}
