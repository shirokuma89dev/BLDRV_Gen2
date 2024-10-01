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
#include "adc.h"
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

int offset = 0;
int16_t getRotation() {
    uint16_t data = AS5048A_read(AS5048A_ANGLE);
    int16_t rotation = (int16_t)data;
    return rotation;
}

// 電気角
uint16_t electrical_angle(int16_t real_angle) {
    uint32_t rotation = real_angle;

    // 7回で一回転
    rotation %= 1170;

    return (uint16_t)rotation;
}

float cos_table[1170];

void setPhase(uint16_t phase, uint16_t pwm_pwr) {
    uint16_t sinwave[3];

    pwm_pwr = pwm_pwr > 320 ? 320 : pwm_pwr;

    sinwave[0] = pwm_pwr + pwm_pwr * cos_table[(phase + 0) % 1170];
    sinwave[1] = pwm_pwr + pwm_pwr * cos_table[(phase + 780) % 1170];
    sinwave[2] = pwm_pwr + pwm_pwr * cos_table[(phase + 390) % 1170];

    // A相
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sinwave[0]);

    // B相
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, sinwave[1]);

    // C相
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, sinwave[2]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, sinwave[2]);
}

void setVoltage(float a, float b, float c) {
    float batt_voltage = 17;  // バッテリー電圧

    // 電圧値を 0-1023 の範囲にマッピングする
    // 電圧 a, b, c をバッテリー電圧で正規化し、0〜1023の範囲にスケーリング
    int int_a = (int)((a / batt_voltage) * 1023) + 512;
    int int_b = (int)((b / batt_voltage) * 1023) + 512;
    int int_c = (int)((c / batt_voltage) * 1023) + 512;

    // clip [ 100, 900 ]
    int clip_low = 200;
    int clip_high = 800;

    int_a = int_a < clip_low ? clip_low : int_a;
    int_a = int_a > clip_high ? clip_high : int_a;

    int_b = int_b < clip_low ? clip_low : int_b;
    int_b = int_b > clip_high ? clip_high : int_b;

    int_c = int_c < clip_low ? clip_low : int_c;
    int_c = int_c > clip_high ? clip_high : int_c;

    // PWM信号を各チャネルに設定
    // A相
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, int_a);

    // B相
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, int_b);

    // C相
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, int_c);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, int_c);
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

uint16_t adcBuffer[3];  // PA0, PA1, PA2の3つのチャンネル分のデータ
uint16_t adcOffset[3];  // オフセット値

float sense[3];
void senseOut() {
    float _sense[3];
    _sense[0] = 0.995832 * (adcBuffer[0] - 2048) -
                0.028199 * (adcBuffer[1] - 2048) -
                0.014988 * (adcBuffer[2] - 2048);

    _sense[1] = 0.037737 * (adcBuffer[0] - 2048) +
                1.007723 * (adcBuffer[1] - 2048) -
                0.033757 * (adcBuffer[2] - 2048);

    _sense[2] = -0.014988 * (adcBuffer[0] - 2048) -
                0.028199 * (adcBuffer[1] - 2048) +
                0.995832 * (adcBuffer[2] - 2048);

    float gain = 1.2;
    float scaleFactor = 3.3 / 4096.0;

    _sense[0] = (_sense[0] * scaleFactor) / gain;
    _sense[1] = (_sense[1] * scaleFactor) / gain;
    _sense[2] = (_sense[2] * scaleFactor) / gain;

    // 　ローパスフィルタ
    const float alpha = 1.0;
    // sense[0] = (1.0 - alpha) * sense[0] + alpha * _sense[0];
    // sense[1] = (1.0 - alpha) * sense[1] + alpha * _sense[1];
    // sense[2] = (1.0 - alpha) * sense[2] + alpha * _sense[2];

    sense[0] = _sense[0];
    sense[1] = _sense[1];
    sense[2] = _sense[2];

    // sense_A += 0.5;
    // sense_B += 0.45;
    // sense_C += 0.33;
}

short velocity = 0;
const uint8_t pre_scaler = 3;
int16_t velocity_ref = 0;

int16_t pwm = 0;

const uint8_t id = 1;
uint8_t isReleased = 1;
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
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */
    setbuf(stdin, NULL);
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    dma_printf_init(&huart1);
    // dma_scanf_init(&huart1);

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
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // タイマー用

    // DRVOFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 3) == HAL_OK)
        printf("Start ADC Successfully\n\r");
    else
        printf("Start ADC failed\n\r");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    for (int i = 0; i < 1170; i++) {
        cos_table[i] = cos(radians(i * 360.0 / 1170.0));
    }

    dma_printf_puts("BLDRV Gen2\r\n");
    dma_printf_puts("Calibration Start\r\n");
    setPhase(0, 100);
    HAL_Delay(1000);

    offset = getRotation();  // なんか3回いる
    offset = getRotation();
    dma_printf_puts("Offset: ");
    dma_printf_puts(uint2char(offset));
    dma_printf_puts("\r\n");
    dma_printf_puts("Calibration End\r\n");

    // offset = 7199;

    HAL_TIM_Base_Start_IT(&htim2);

    uint8_t ring_buf[2] = {0};
    ring_buf[0] = 0B01000000;
    ring_buf[1] = 0B11000000;

    HAL_UART_Receive_DMA(&huart1, ring_buf, 2);

    ring_buf[0] = 0B01000000;
    ring_buf[1] = 0B11000000;

    while (!(ring_buf[0] == 0B11111111 && ring_buf[1] == 0B11111111)) {
        printf("waiting\t%d\t%d\n", ring_buf[0], ring_buf[1]);
    }

    printf("start\n");

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        printf("ref,pwm,real:%d\t%d\t%d\n", velocity_ref, pwm, velocity);

        uint8_t index = 0;
        for (int i = 0; i < 2; i++) {
            if (ring_buf[i] >> 7 == id) {
                index = i;
                break;
            }
        }

        if (ring_buf[index] == 0B11111111) {
            continue;
        }

        int8_t _ref = ((ring_buf[index] & 0x7F) << 1);

        if (_ref < -120) {
            isReleased = 1;
        } else {
            isReleased = 0;
        }

        velocity_ref = _ref * 2;

        // printf("%d\t%d\t%d\t%d\n", ring_buf[0], ring_buf[1], _ref,
        // velocity_ref); printf("%d\n", _ref);
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        short real_angle = getRotation() - offset;
        real_angle += 8192;
        real_angle %= 8192;
        uint16_t rotation = electrical_angle(real_angle);

        static uint8_t count = 0;
        if (32 / (pre_scaler + 1) == count) {
            count = 0;

            static short old_real_angle = 0;

            velocity = real_angle - old_real_angle;
            if (velocity > 4096) {
                velocity -= 8192;
            } else if (velocity < -4096) {
                velocity += 8192;
            }

            old_real_angle = real_angle;
        }
        count++;

        static int32_t err_vel = 0;
        static int32_t int_err_vel = 0;

        int32_t err = velocity_ref - velocity;
        int_err_vel += err;

        const float kp = 1.2;
        const float ki = 0.04;

        pwm = kp * err + ki * int_err_vel;

        const int max_int_err_vel = 7000;
        if (int_err_vel > max_int_err_vel) {
            int_err_vel = max_int_err_vel;
        } else if (int_err_vel < -max_int_err_vel) {
            int_err_vel = -max_int_err_vel;
        }

        const int max_pwm = 320;
        if (pwm > max_pwm) {
            pwm = max_pwm;
        } else if (pwm < -max_pwm) {
            pwm = -max_pwm;
        }

        if (pwm < 10 && pwm > -10) {
            pwm = 0;
        }

        if (isReleased) {
            pwm = 0;
        }

        if (pwm > 0) {
            setPhase((rotation + 292) % 1170, pwm);
        } else {
            setPhase((rotation + 877) % 1170, -pwm);
        }
    }
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
