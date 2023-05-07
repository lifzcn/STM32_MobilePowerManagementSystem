/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "mlx90614.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Pushdown_Key_1 HAL_GPIO_ReadPin(Key_1_GPIO_Port, Key_1_Pin) == GPIO_PIN_RESET
#define Pushdown_Key_2 HAL_GPIO_ReadPin(Key_2_GPIO_Port, Key_2_Pin) == GPIO_PIN_RESET
#define Pushdown_Key_3 HAL_GPIO_ReadPin(Key_3_GPIO_Port, Key_3_Pin) == GPIO_PIN_RESET
#define Pushdown_Key_4 HAL_GPIO_ReadPin(Key_4_GPIO_Port, Key_4_Pin) == GPIO_PIN_RESET
#define NotPushdown_Key_1 HAL_GPIO_ReadPin(Key_1_GPIO_Port, Key_1_Pin) == GPIO_PIN_SET
#define NotPushdown_Key_2 HAL_GPIO_ReadPin(Key_2_GPIO_Port, Key_2_Pin) == GPIO_PIN_SET
#define NotPushdown_Key_3 HAL_GPIO_ReadPin(Key_3_GPIO_Port, Key_3_Pin) == GPIO_PIN_SET
#define NotPushdown_Key_4 HAL_GPIO_ReadPin(Key_4_GPIO_Port, Key_4_Pin) == GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t noInputDelayTimeCounter = 0;
uint16_t lowVoltageDelayTimeCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t x = 0;
  uint8_t y = 0;
  float temperatureValue = 0;
  uint8_t temperatureIntegerValue = 0;
  uint8_t temperatureDecimalValue = 0;
  uint16_t adcValue = 0;
  float voltageValue = 0;
  uint8_t voltageIntegerValue = 0;
  uint8_t voltageDecimalValue_Unit = 0;
  uint8_t voltageDecimalValue_Ten = 0;
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;
  uint8_t yearValue = 0;
  uint8_t monthValue = 0;
  uint8_t dateValue = 0;
  uint8_t hourValue = 0;
  uint8_t minuteValue = 0;
  uint8_t secondValue = 0;
  uint8_t i = 0;
	uint8_t j = 0;
	float standardVoltage = 3.58;
	uint8_t standardTemperature = 30;
	uint8_t powerONTime_HourValue = 8;
	uint8_t powerONTime_MinuteValue = 0;
	uint8_t powerOFFTime_HourValue = 18;
	uint8_t powerOFFTime_MinuteValue = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    yearValue = sdatestructure.Year + 2000;
    monthValue = sdatestructure.Month;
    dateValue = sdatestructure.Date;
    hourValue = stimestructure.Hours;
    minuteValue = stimestructure.Minutes;
    secondValue = stimestructure.Seconds;

    if (Pushdown_Key_1)
    {
      HAL_Delay(50);
      i += 1;
    }
		
		switch(i%2)
		{
			case 0:
				printf("Power: OFF!\n");
				j = 0;
			break;
			case 1:
				printf("Power: ON!\n");
				j = 1;
			break;
			default:
				j = 1;
			break;
		}
		
		if (hourValue == powerOFFTime_HourValue)
		{
			if (minuteValue == powerOFFTime_MinuteValue)
			{
				j = 0;
			}
		}
		
		switch(j)
		{
			case 0:
				OLED_Clear();
			break;
			case 1:
				OLED_ShowChinese(x + 32 + 16 * 0, y + 2 * 0, 0);
				OLED_ShowChinese(x + 32 + 16 * 1, y + 2 * 0, 1);
				OLED_ShowChinese(x + 32 + 16 * 2, y + 2 * 0, 2);
				OLED_ShowChinese(x + 32 + 16 * 3, y + 2 * 0, 3);
				OLED_ShowChinese(x + 32 + 16 * 0, y + 2 * 1, 4);
				OLED_ShowChinese(x + 32 + 16 * 1, y + 2 * 1, 5);
				OLED_ShowChinese(x + 32 + 16 * 2, y + 2 * 1, 6);
				OLED_ShowChinese(x + 32 + 16 * 3, y + 2 * 1, 7);
				OLED_ShowChinese(x + 16 * 0, y + 2 * 2, 8);
				OLED_ShowChinese(x + 16 * 1, y + 2 * 2, 9);
				OLED_ShowChinese(x + 16 * 2, y + 2 * 2, 10);
				OLED_ShowChinese(x + 16 * 3, y + 2 * 2, 11);
				OLED_ShowChar(x + 16 * 4 + 8 * 0, y + 2 * 2, ':', 16);
				OLED_ShowChinese(x + 16 * 0, y + 2 * 3, 8);
				OLED_ShowChinese(x + 16 * 1, y + 2 * 3, 9);
				OLED_ShowChinese(x + 16 * 2, y + 2 * 3, 12);
				OLED_ShowChinese(x + 16 * 3, y + 2 * 3, 13);
				OLED_ShowChar(x + 16 * 4 + 8 * 0, y + 2 * 3, ':', 16);
				
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 100);
				adcValue = HAL_ADC_GetValue(&hadc1);
				voltageValue = (float)adcValue / 4096 * 3.3;
				voltageIntegerValue = (int)voltageValue;
				voltageDecimalValue_Unit = (int)((voltageValue - voltageIntegerValue) * 10);
				voltageDecimalValue_Ten = (int)(((voltageValue - voltageIntegerValue) * 10 - voltageDecimalValue_Unit) * 10);
				OLED_ShowNum(x + 16 * 4 + 8 * 1, y + 2 * 2, voltageIntegerValue, 2, 16);
				OLED_ShowChar(x + 16 * 4 + 8 * 3, y + 2 * 2, '.', 16);
				OLED_ShowNum(x + 16 * 4 + 8 * 4, y + 2 * 2, voltageDecimalValue_Unit, 1, 16);
				OLED_ShowNum(x + 16 * 4 + 8 * 5, y + 2 * 2, voltageDecimalValue_Ten, 1, 16);
				OLED_ShowChar(x + 16 * 4 + 8 * 6, y + 2 * 2, 'V', 16);
				
				temperatureValue = MLX90614_GetTemperature();
				temperatureIntegerValue = (int)temperatureValue;
				temperatureDecimalValue = 10 * (temperatureValue - (int)temperatureValue);
				OLED_ShowNum(x + 16 * 4 + 8 * 1, y + 2 * 3, temperatureIntegerValue, 2, 16);
				OLED_ShowChar(x + 16 * 4 + 8 * 3, y + 2 * 3, '.', 16);
				OLED_ShowNum(x + 16 * 4 + 8 * 4, y + 2 * 3, temperatureDecimalValue, 1, 16);
				OLED_ShowChar(x + 16 * 4 + 8 * 5, y + 2 * 3, 'C', 16);
				
				if (Pushdown_Key_2)
				{
					HAL_Delay(50);
					if (Pushdown_Key_3)
					{
						HAL_Delay(50);
						powerOFFTime_HourValue += 1;
					}
					else if (Pushdown_Key_4)
					{
						HAL_Delay(50);
						powerOFFTime_HourValue -= 1;
					}
				}
				
				if (NotPushdown_Key_1 && NotPushdown_Key_2 && NotPushdown_Key_3 && NotPushdown_Key_4)
				{
					HAL_TIM_PeriodElapsedCallback(&htim1);
				}
				
				if (voltageValue < standardVoltage)
				{
					HAL_TIM_PeriodElapsedCallback(&htim2);
				}
				
				if (temperatureValue > standardTemperature)
				{
					HAL_GPIO_WritePin(LED_IO_GPIO_Port, LED_IO_Pin, GPIO_PIN_SET);
					HAL_Delay(250);
					HAL_GPIO_TogglePin(LED_IO_GPIO_Port, LED_IO_Pin);
				}
			break;
			default:
				;
			break;
		}
		
		HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim1.Instance)
	{
		noInputDelayTimeCounter++;
		if (noInputDelayTimeCounter == 5000)
		{
			OLED_Clear();
		}
	}
	if (htim->Instance == htim2.Instance)
	{
		lowVoltageDelayTimeCounter++;
		if (lowVoltageDelayTimeCounter == 4 * 5000)
		{
			OLED_Clear();
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
