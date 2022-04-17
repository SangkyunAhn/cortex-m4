/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "rtc.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SWA (!HAL_GPIO_ReadPin(GA_GPIO_Port, GA_Pin))
#define SWB (!HAL_GPIO_ReadPin(GB_GPIO_Port, GB_Pin))
#define KEY (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t enc = 0;
int alarm_hour = 0;
int alarm_minute = 0;
int alarm_second = 0;
char lcd_alarm_time[8] = {0};
char lcd_alarm_hour[2] = {0};
char lcd_alarm_minute[2] = {0};
char lcd_alarm_second[2] = {0};
int old_alarm_hour = 0;
int old_alarm_minute = 0;
int old_alarm_second = 0;
uint8_t MSG[30] = {'\0'};
uint8_t oldBt = 0;
char lcdMsg[30] = {'\0'};
char status[5][20] = {"Alarm time",
		"Change hour",
		"Change minute",
		"Change second",
		"Present time"};
RTC_TimeTypeDef sTime;
char showTime[30] = {0};
uint8_t sFlag = RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rotaryEnc(void)
{
	uint8_t nowEnc = 0;
	uint8_t returnEnc = 0;
	static uint8_t oldEnc = 0;

	nowEnc = (SWB << 1) + SWA;

	if (oldEnc == 0 && nowEnc == 1) {
		returnEnc = 1; // CW
	} else if (oldEnc == 0 && nowEnc == 2) {
		returnEnc = 2; // CCW
	}

	oldEnc = nowEnc;

	return returnEnc;
}

void get_time(void)
{
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	sprintf((char *)showTime, "%02d:%02d:%02d\r\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();

  LCD_SetCursor(0, 0);
  LCD_Puts(0, 0, status[4]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  enc = rotaryEnc();
	  static int index = 4;

	  if (index != 4) {
		if (index == 0) {
		  sprintf(lcd_alarm_time, "%02d:%02d:%02d", alarm_hour, alarm_minute, alarm_second);
		  LCD_Puts(0, 1, lcd_alarm_time);
		}

		if (enc == 1) {
			if (index == 1) {
				alarm_hour++;
				if (alarm_hour == 24)
					alarm_hour = 0;
			}
			if (index == 2) {
				alarm_minute++;
				if (alarm_minute == 60)
					alarm_minute = 0;
			}
			if (index == 3) {
				alarm_second++;
				if (alarm_second == 60)
					alarm_second = 0;
			}
		} else if (enc == 2) {
			if (index == 1) {
				alarm_hour--;
				if (alarm_hour == -1)
					alarm_hour = 23;
			}
			if (index == 2) {
				alarm_minute--;
				if (alarm_minute == -1)
					alarm_minute = 59;
			}
			if (index == 3) {
				alarm_second--;
				if (alarm_second == -1)
					alarm_second = 59;
			}
		}

		if (alarm_hour != old_alarm_hour) {
			sprintf(lcd_alarm_hour, "%02d", alarm_hour);
			LCD_Puts(0, 1, lcd_alarm_hour);
			old_alarm_hour = alarm_hour;
		}

		if (alarm_minute != old_alarm_minute) {
			sprintf(lcd_alarm_minute, ":%02d", alarm_minute);
			LCD_Puts(2, 1, lcd_alarm_minute);
			old_alarm_minute = alarm_minute;
		}

		if (alarm_second != old_alarm_second) {
			sprintf(lcd_alarm_second, ":%02d", alarm_second);
			LCD_Puts(5, 1, lcd_alarm_second);
			old_alarm_second = alarm_second;
		}
	  } else {
		  LCD_Puts(0, 1, showTime);
	  }

	  if (KEY != oldBt && KEY) {
		  index++;
		  if (index == 5) index = 0;

		  LCD_Puts(0, 0, "                ");
		  LCD_Puts(0, 0, status[index]);
	  }

	  oldBt = KEY;

	  get_time();

	  if (sTime.Hours == alarm_hour && sTime.Minutes == alarm_minute && sTime.Seconds == alarm_second) {
		  HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
	  } else {
		  HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);
	  }

	  HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

