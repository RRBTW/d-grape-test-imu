/* ============================================================
 *  main.c — d-grape-test-imu
 *  MPU6050 через I2C1, вывод данных через USB CDC
 *
 *  Подключение:
 *    ST-Link (CN1, Mini-USB)  → прошивка
 *    USB OTG (CN5, Micro-USB) → COM-порт (CDC)
 *    PB6 → I2C1_SCL
 *    PB7 → I2C1_SDA
 *    AD0 → GND (адрес 0x68)
 *
 *  Ожидаемые значения в покое (плата горизонтально):
 *    az ≈ ±9.81 м/с², ax/ay ≈ 0, gx/gy/gz ≈ 0
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
    /* PD14 = красный LED на Discovery */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
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
    HAL_Delay(2000U);   /* ждём энумерацию USB */

    if (imu_init(&hi2c1) != HAL_OK) {
        uint8_t who = imu_whoami(&hi2c1);
        char err[80];
        int n = snprintf(err, sizeof(err),
            "IMU init FAILED — WHO_AM_I = 0x%02X (ожидалось 0x68)\r\n",
            (unsigned)who);
        if (n > 0) cdc_send(err, (uint16_t)n);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); /* красный */
        while (1) {}
    }

    const char *banner =
        "\r\n=== d-grape-test-imu: MPU6050 ===\r\n"
        " ax[m/s2]   ay[m/s2]   az[m/s2]"
        "   gx[r/s]   gy[r/s]   gz[r/s]"
        "   t[C]\r\n"
        "-----------------------------------------------------------\r\n";
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
            char buf[96];
            int n = snprintf(buf, sizeof(buf),
                "%9.4f  %9.4f  %9.4f  %8.4f  %8.4f  %8.4f  %6.2f\r\n",
                (double)imu.ax, (double)imu.ay, (double)imu.az,
                (double)imu.gx, (double)imu.gy, (double)imu.gz,
                (double)imu.temp_c);
            if (n > 0) cdc_send(buf, (uint16_t)n);
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); /* зелёный heartbeat */
        }
    }
}

/* ── Тактирование 168 МГц (HSE 8 МГц) ──────────────────────*/
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8;
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;   /* SYSCLK = 168 МГц */
    osc.PLL.PLLQ       = 7;               /* USB = 48 МГц     */
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;   /* PCLK1 = 42 МГц */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;   /* PCLK2 = 84 МГц */
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/* ── GPIO: LEDs (PD12-PD15) ─────────────────────────────────*/
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    /* PD12=зелёный, PD13=оранжевый, PD14=красный, PD15=синий */
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);
    HAL_GPIO_WritePin(GPIOD,
        GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
        GPIO_PIN_RESET);
}

/* ── I2C1: PB6=SCL, PB7=SDA, Fast Mode 400 кГц ─────────────*/
static void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000U;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}
