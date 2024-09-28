/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>

#include "dma_printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define radians(x) ((x) * 3.14159265358979323846 / 180.0)

#define CS_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define AS5048A_ANGLE 0x3FFF

uint8_t spiCalcEvenParity(uint16_t value) {
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (value & 0x1) {
            cnt++;
        }
        value >>= 1;
    }
    return cnt & 0x1;
}

uint16_t AS5048A_read(uint16_t registerAddress) {
    uint16_t command = 0x4000 | registerAddress;

    // パリティビットの追加
    command |= spiCalcEvenParity(command) << 15;

    uint16_t response = 0;

    // SPI通信開始
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&command, 1, HAL_MAX_DELAY);
    CS_HIGH();

    // データ受信
    CS_LOW();
    HAL_SPI_Receive(&hspi1, (uint8_t *)&response, 1, HAL_MAX_DELAY);
    CS_HIGH();

    // エラービットの確認（必要に応じて実装）
    if (response & 0x4000) {
        // エラー処理
    }

    return response & 0x3FFF;  // データ部分のみ返す
}

int16_t getRotation() {
    uint16_t data = AS5048A_read(AS5048A_ANGLE);
    int16_t rotation = (int16_t)data;
    return rotation;
}

double cos_table[360];
void setPhase(uint16_t phase, uint16_t pwm_pwr) {
    uint16_t sinwave[3];

    sinwave[0] = pwm_pwr + pwm_pwr * cos_table[(phase + 0) % 360];
    sinwave[1] = pwm_pwr + pwm_pwr * cos_table[(phase + 120) % 360];
    sinwave[2] = pwm_pwr + pwm_pwr * cos_table[(phase + 240) % 360];
    // A相
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sinwave[0]);

    // B相
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, sinwave[1]);

    // C相
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, sinwave[2]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, sinwave[2]);
}

char *uint2char(uint16_t num) {
    static char str[5];
    str[0] = num / 1000 + '0';
    str[1] = (num % 1000) / 100 + '0';
    str[2] = (num % 100) / 10 + '0';
    str[3] = num % 10 + '0';
    str[4] = '\0';

    return str;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM16_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    setbuf(stdin, NULL);
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    dma_printf_init(&huart1);
    dma_scanf_init(&huart1);

    // A相
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // H
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // L

    // B相
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);     // H
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);  // L

    // C相
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);     // H
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  // L

    // デバッグ用LED
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // タイマー用
    HAL_TIM_Base_Start(&htim6);

    // DRVOFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    for (int i = 0; i < 360; i++) {
        cos_table[i] = cos(radians(i));
    }

    dma_printf_puts("BLDRV Gen2\r\n");
    dma_printf_puts("Calibration Start\r\n");
    setPhase(0, 50);
    HAL_Delay(500);

    int offset = getRotation();
    offset = getRotation();
    dma_printf_puts("Offset: ");
    dma_printf_puts(uint2char(offset));
    dma_printf_puts("\r\n");
    dma_printf_puts("Calibration End\r\n");

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        uint16_t sinwave[3];
        uint16_t pwm_pwr = 400;

        static int phase = 0;

        uint32_t rotation = getRotation() - offset;
        rotation += 8192;
        rotation %= 8192;

        // 7回で一回転
        rotation %= 1170;
        rotation *= 360;
        rotation /= 1170;
        rotation %= 360;

        rotation += 90;
        rotation %= 360;


        setPhase(rotation, pwm_pwr);

        // dma_printf_puts("Angle: ");
        // dma_printf_puts(uint2char(rotation));
        // dma_printf_puts("\r\n");
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_TIM1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    dma_printf_send_it(huart);
}
int __io_putchar(int ch) {
    dma_printf_putc(ch & 0xFF);
    return ch;
}

int __io_getchar(void) {
    return dma_scanf_getc_blocking();
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
