/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdio.h"
#include <stdlib.h>
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_PACKET_LEN 255


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;

UARTStucrture UART2 =
{ 0 };

typedef enum{
	state_idle,
	state_start,
	state_mode,
	state_n_station,
	state_data_frame,
	state_check_sum,
	state_wait_for_ack1_1,
	state_wait_for_ack1_2
}uart_state;

int inputchar = -1;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void UARTInit(UARTStucrture *uart);

void UARTResetStart(UARTStucrture *uart);

uint32_t UARTGetRxHead(UARTStucrture *uart);

int16_t UARTReadChar(UARTStucrture *uart);

void UARTTxDumpBuffer(UARTStucrture *uart);

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);

void uart_protocal(int16_t input,UARTStucrture *uart);


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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  UART2.huart = &huart2;
  UART2.RxLen = 255;
  UART2.TxLen = 255;
  UARTInit(&UART2);
  UARTResetStart(&UART2);


  /* USER CODE END 2 */
  {
	  char temp[32] = {"Hello\r\n"};
	  UARTTxWrite(&UART2, (uint8_t*) temp, strlen(temp));
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int16_t inputChar = UARTReadChar(&UART2);
	  if(inputChar != -1){
		  /*char temp[32];
		  sprintf(temp, "%d", inputChar);
		  UARTTxWrite(&UART2, (uint8_t*) temp, strlen(temp));*/
		  uart_protocal(inputChar, &UART2);
		  inputchar = -1;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  UARTTxDumpBuffer(&UART2);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;

}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}
uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}
int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;

}
void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}
}
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffur?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;
	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);

}

uint8_t mcu_connect = 0,goals[512] = {0},go_now = 0,current_station = 0,enable_gripper = 0,enable_sethome = 0;
uint16_t n_goal = 0;
double max_velocity = 0,set_position = 0,current_position = 1.5634;
static uart_state state = state_idle;
static uint8_t sum = 0;
static uint8_t datas[256] = {0},data_ind = 0,n_data = 0;
static uint8_t mode = 0;

void uart_protocal(int16_t input,UARTStucrture *uart){

	switch (state) {
		case state_idle:
			sum = data_ind = 0;
			if(input >= 0b10010001 && input <= 0b10011110){
				mode = input & 0b1111;
				sum += input;
				switch (mode){
					case 1:n_data = 2;state = state_data_frame;break;
					case 2:state = state_check_sum;break;
					case 3:state = state_check_sum;break;
					case 4:n_data = 1;state = state_data_frame;break;
					case 5:n_data = 2;state = state_data_frame;break;
					case 6:n_data = 1;state = state_data_frame;break;
					case 7:state = state_n_station;break;
					case 8:state = state_check_sum;break;
					case 9:state = state_check_sum;break;
					case 10:state = state_check_sum;break;
					case 11:state = state_check_sum;break;
					case 12:state = state_check_sum;break;
					case 13:state = state_check_sum;break;
					case 14:state = state_check_sum;break;
				}
			}
			else{
				sum = n_data = data_ind = mode = 0;
			}
			break;
		case state_n_station:
			n_data = (input+1)/2 & 0xFF;
			sum+= input;
			state = state_data_frame;
			break;
		case state_data_frame:
			if(data_ind < n_data){
				datas[data_ind] = input;
				sum += datas[data_ind++];
			}
			if(data_ind == n_data){
				state = state_check_sum;
			}

		case state_check_sum:
			if(input == (uint8_t) ~sum){
				switch(mode){
					case 1:{
						uint8_t temp[] = { (0b1001<<4) & mode , datas[0] , datas[1] , (uint8_t)input};
						UARTTxWrite(&UART2, temp, 4);
						state = state_idle;
						break;
					}
					case 2:{
						mcu_connect = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 3:{
						mcu_connect = 0;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 4:{
						max_velocity = datas[0];
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 5:{
						set_position = (double)((uint16_t)(datas[0]<<8) + datas[1])/1e-4;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 6:{
						goals[0] = datas[0];
						n_goal = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 7:{
						n_goal = 0;
						for(int i = 0;i < n_data;i++){
							goals[2*i] = datas[i] & 0b1111;
							goals[2*i+1] = datas[i]>>4;
							n_goal += 2;
						}
						if(goals[n_goal-1] == 0){
							n_goal--;
						}
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 8:{
						go_now = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 9:{
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						uint8_t temp2[] = {0b10011001,current_station,~(0b10011001+current_station) & 0xFF};
						UARTTxWrite(&UART2, temp2, 3);
						state = state_wait_for_ack1_1;
						break;
					}
					case 10:{
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						uint16_t pos = (uint16_t)(current_position*1e4);
						uint8_t temp2[] = {0b10011010,pos >> 8,pos & 0xFF, ~(0b10011001+(pos >> 8)+ (pos & 0xFF)) & 0xFF};
						UARTTxWrite(&UART2, temp2, 4);
						state = state_wait_for_ack1_1;
						break;
					}
					case 11:{
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						uint8_t temp2[] = {0b10011011,(uint8_t)max_velocity,~(0b10011001+(uint8_t)max_velocity) & 0xFF};
						UARTTxWrite(&UART2, temp2, 3);
						state = state_wait_for_ack1_1;
						break;
					}
					case 12:{
						enable_gripper = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 13:{
						enable_gripper = 0;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 14:{
						enable_sethome = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
				}
			}
			else{
				//error check sum
			}
			break;
		case state_wait_for_ack1_1:{if(input == 0x58){state = state_wait_for_ack1_2;}break;}
		case state_wait_for_ack1_2:{if(input == 0b01110101){state = state_idle;}break;}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
