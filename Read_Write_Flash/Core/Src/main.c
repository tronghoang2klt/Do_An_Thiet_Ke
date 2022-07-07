/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include "cJSON.h"
#include "DHT.h"
#include "stdio.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LENG	50
#define MAX		500
#define TIME	3 // 6*5=30min
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Task_Luu_Flash(void);
void Task_Gui_Tu_Flash(void);
void Task_Gui_Cho_Esp(void);
void Task_DHT11(void);
void Task_Tao_Data(float temperature, float humidity);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float temperature=1,humidity=1;
DHT_DataTypedef DHT11_Data;
char data_luu_json[MAX],JSON_Flash[MAX];
char str_temperature[LENG],str_humidity[LENG],JSON[LENG];
char Flash_To_ESP[LENG];
long long last_tick=0;
int flag_gui=1,flag_luu=0;
int count=0;
int so_lan_da_luu=0;
int flag_rx=0;
char luu_usrt[5];
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2,(uint8_t *)&luu_usrt,5);
	last_tick=HAL_GetTick();// lay uwTich cua stm32
	for(int i=0;i<MAX;i++)
				{
					data_luu_json[i]=0;
				}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//Task_DHT11();
		//----------nhap nhay led de bao van con hoat dong-------------//
		if (HAL_GetTick()-last_tick>=1000) 
		{
			
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			Task_DHT11();
			last_tick=HAL_GetTick();
		}
		
		//----------doc tin hieu yeu cau gui tu esp-------------------//
		if (flag_rx == 1)
		{
			//--------tu trang thai flash sang trang thai gui len esp----------------//
			if(flag_luu == 1 && flag_gui == 0)
			{
				Task_Gui_Tu_Flash();
				//HAL_UART_Transmit(&huart2,(uint8_t *)&JSON_Flash,MAX,1000);
			}
			//-------neu bat dau tu trang thai hoat dong esp----------//
			else
			{
				//---- xoa du lieu dat_json------///
				/*for(int i=0;i<MAX;i++)
				{
					data_luu_json[i]=0;
				}*/
				Task_Gui_Cho_Esp();
			}
			flag_luu=0;
			flag_gui=1;
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&luu_usrt,5);
		}
		//-------------neu khong co tin hieu yeu cau tu esp----------//
		//-------------luu vao flash---------------------------------//
		else 
		{
			Task_Luu_Flash();
			flag_gui=0;
			flag_luu=1;
			flag_rx=0;
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&luu_usrt,5);
		}
		//last_tick=HAL_GetTick();
	
		HAL_Delay(1000);//delay 5s(test)
  
  /* USER CODE END 3 */
}
	}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Task_DHT11(void)
{
		DHT_GetData(&DHT11_Data);
		temperature=DHT11_Data.Temperature;
		humidity=DHT11_Data.Humidity;
}
void Task_Tao_Data(float temperature,float humidity)
{
	//cau truc co ban cu json la:     {"temp":"giatri","humi":"giatri"}\n
	// data cuoi cung la 1 mang char

	//B1: xoa mang du lieu
	for (int i = 0;  i <LENG; i++) {
		str_temperature[i]=0;
		str_humidity[i]=0;
		JSON[i]=0;
	}

	//B2: chuyen du lieu ve char
	sprintf(str_temperature,"%.2f",temperature);
	sprintf(str_humidity,"%.2f",humidity);

	//B3: copy vao json
	strcat(JSON,"{\"temp\":\"");
	strcat(JSON,str_temperature);
	strcat(JSON,"\",");

	strcat(JSON,"\"humi\":\"");
	strcat(JSON,str_humidity);
	strcat(JSON,"\"}\n");

}
void Task_Luu_Flash(void)
{
	//---------sau 6 lan ghi vao data_luu_json moi ghi vao flash lai 1 lan---------//
	while(so_lan_da_luu < TIME)
	{
	
	Task_Tao_Data(temperature,humidity);
	so_lan_da_luu++;
	count++;
	
	//ghi du lieu vao data_luu_JSON
	strcat(data_luu_json,JSON);
	}
	Flash_Write_String((uint8_t *)&data_luu_json,_PAGE_118_,MAX);
		so_lan_da_luu=0;
		for (int i = 0;  i <MAX; i++) 
				{
					data_luu_json[i]=0;
				}
}
void Task_Gui_Tu_Flash(void)
{
				for (int i = 0;  i <MAX; i++) 
				{
					JSON_Flash[i]=0;
				}
				//doc theo byte
				Flash_Read_String((uint8_t *)&JSON_Flash,_PAGE_118_,MAX);// luu vao json_flash
				int t=0;
				int vitri=0;
				for(int i=0;i<TIME;i++)
				{
					for(int j=0;j<32;j++)
					{
						Flash_To_ESP[j]=JSON_Flash[t];
						t++;
					}
					HAL_UART_Transmit(&huart2,(uint8_t *)&Flash_To_ESP,32,1000);
				}
				count=0;
}
void Task_Gui_Cho_Esp(void)
{
	Task_Tao_Data(temperature,humidity);
	HAL_UART_Transmit(&huart2,(uint8_t *)&JSON,sizeof(JSON),1000);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	flag_rx++;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
