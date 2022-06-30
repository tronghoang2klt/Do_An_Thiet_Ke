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
#define LENG 5
#define address 0x800FC02
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
void Task_UART(void);
void Task_Luu_Flash(void);
void Task_DHT11(void);
void Received(void);
void ClearBufferEnd(void);
void XuLyJSON(char *DataJSON);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void SendData(unsigned int trangthaiden,unsigned int thongso1);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float temperature=1,humidity=1;
DHT_DataTypedef DHT11_Data;
char data_rx[LENG];
uint8_t flag_rx = 0; //tranh tinh trang o trong ngat qua lau
uint8_t flag_button = 0;
char data_tx[LENG];
int thongso1=0,trangthaiden=0;
char str_den[100],str_trangthaiden[100],JSON[100];
char rx_buffer[200];// luu cac ki tu doc duoc vao mang
uint8_t rx_data;//luu tung ki tu doc dc
unsigned int rx_index=0;
cJSON *Str_json,*Str_trangthaiden_json,*Str_den_json;
long long last_tick=0;
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
	HAL_UART_Receive_IT(&huart2,(uint8_t *)&data_rx,LENG);
	//Flash_Write_Uint(100,_PAGE_127_);
	uint32_t mil=HAL_GetTick();
	uint16_t i=0;
	last_tick=HAL_GetTick();// lay uwTich cua stm32
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  if (HAL_GetTick()-last_tick>=1000) {
		  SendData(trangthaiden, thongso1);
		last_tick=HAL_GetTick();
		}
		if( HAL_GetTick() - mil > 500)
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			mil=HAL_GetTick();
		}
		Task_DHT11();
		if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == 1)// nhan dc thong tin gui tu esp
		{
			Task_UART();
		}
		else // neu khong nhan dc thong tin gui
		{
			Task_Luu_Flash();
		}
		//ghi du lieu vao flash
		if(flag_rx == 1)
		{
			//Flash_Write_Uint(i,_PAGE_127_);
			Flash_Write_String((uint8_t *)&data_rx,_PAGE_127_,LENG);//luu vao page 127 de tranh tinh trang ghi de len flash
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&data_rx,LENG);
			flag_rx = 0;
		}
		
		
		/*if (flag_button == 1)
		{
			Flash_Read_String(data_tx,_PAGE_127_,LENG);// doc ra
			HAL_UART_Transmit(&huart2,data_tx,LENG,10);
			flag_button =0;
		}
		Flash_Read_String(data_tx,address,LENG);// doc ra
		*/
		//HAL_UART_Receive_IT(&huart2,data_rx,LENG);
		i++;
		//Flash_Write_Uint(100,_PAGE_127_);
		//uint32_t a= Flash_Read_Uint(_PAGE_127_);
		//sprintf(data_rx,"a:%d",a);
		//Flash_Write_String((uint8_t *)&data_rx,address,LENG);
		Flash_Read_String((uint8_t *)&data_tx,_PAGE_127_,LENG);
		HAL_UART_Transmit(&huart2,(uint8_t *)&data_tx,LENG,1000);
		HAL_Delay(1000);
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART2) {
		last_tick=HAL_GetTick();
		// cho bien nay vao tranh truong hop dang su ly data thi gui ve
		// tuc la tranh tinh trang dang doc du lieu thi gui ve
		// chuong trinh doc du lieu => mang data
		Received();
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}
void Received(void)
{
	if (rx_data!='\n') {
		rx_buffer[rx_index++]=rx_data;
	}
	else
	{
		//printf(" du lieu nhan dc : %s\n",rx_buffer);
		XuLyJSON(rx_buffer);
		ClearBufferEnd();
		//printf(" du lieu nhan dc : %d\n",rx_index);
	}

}
void ClearBufferEnd(void)
{
	rx_index=0;
	for(int i=0 ; i<200 ; i++)
	{
		rx_buffer[i]=0;
	}
	last_tick=HAL_GetTick();
}
void XuLyJSON(char *DataJSON)
{
	Str_json=cJSON_Parse(DataJSON);
	//kiem tra xem cai du lieu dua vao co la cau truc json hay ko
	// {"name":"giatri"}
	if(!Str_json)
	{
		printf("json error!\r\n");
		return;
	}
	else
	{
		printf("Json ok!\r\n");
		// {"name":"giatri"}
		if (cJSON_GetObjectItem(Str_json, "thongso1")) // neu name la thongso1
		{
			// lay du lieu khi nhap vao tu thong so 1
			thongso1=atoi(cJSON_GetObjectItem(Str_json, "thongso1")->valuestring);
			printf("thong so 1 thay doi la : %d\r\n",thongso1);
		}
		else if(cJSON_GetObjectItem(Str_json,"trangthaiden"))// neu name la trang thai den
		{
			// so sanh gia tri cua trang thai den voi 1
			if(strstr(cJSON_GetObjectItem(Str_json, "trangthaiden")->valuestring,"0")!= NULL)
			{
				printf("off led 1\r\n");
				trangthaiden=0;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,SET);
			}
				// so sanh gia tri cua trang thai den voi 1
			else if(strstr(cJSON_GetObjectItem(Str_json, "trangthaiden")->valuestring,"1")!= NULL)
				{
					printf("on led 1\r\n");
					trangthaiden=1;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,RESET);
				}
		}
	}
}
void SendData(unsigned int trangthaiden,unsigned int thongso1)
{
	//cau truc co ban cu json la: "ten1":"gia tri1","ten2":"gia tri2",...
	// data cuoi cung la 1 mang char

	//B1: xoa mang du lieu
	for (int i = 0;  i <100; i++) {
		str_den[i]=0;
		str_trangthaiden[i]=0;
		JSON[i]=0;
	}

	//B2: chuyen du lieu ve char
	sprintf(str_den,"%d",thongso1);
	sprintf(str_trangthaiden,"%d",trangthaiden);

	//B3: copy vao json
	strcat(JSON,"{\"trangthaiden\":\"");
	strcat(JSON,str_trangthaiden);
	strcat(JSON,"\",");

	strcat(JSON,"\"thongsoden\":\"");
	strcat(JSON,str_den);
	strcat(JSON,"\"}");

	printf("%s\n",JSON);
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
