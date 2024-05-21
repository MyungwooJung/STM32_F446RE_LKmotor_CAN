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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CW 0x00
#define CCW 0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//CAN variables
CAN_RxHeaderTypeDef canRxHeader;
CAN_TxHeaderTypeDef canTxHeader;
CAN_FilterTypeDef canFilter1;
uint32_t TxMailBox;
uint8_t can1Rx0Data[8];
uint8_t can1Tx0Data[8];

uint8_t datacheck = 0;
uint8_t mode = 0;

//position, speed, torque, temperature variables
int16_t iqControl;  //closed loop torque
int32_t speedControl; //closed loop speed-0.01dps/LSB
uint8_t spinDirection; //cw(0x00), ccw(0x01)
uint32_t angleControl; //single loop angle control
int32_t multiAngleControl; // multi loop angle control


void torque_closedloop_control(uint8_t id, int32_t iqControl){
	canTxHeader.StdId = 0x140 + id;
	can1Tx0Data[0] = 0xA1;
	can1Tx0Data[1] = 0x00;
	can1Tx0Data[2] = 0x00;
	can1Tx0Data[3] = 0x00;
	can1Tx0Data[4] = *(uint8_t *)(&iqControl);
	can1Tx0Data[5] = *((uint8_t *)(&iqControl) + 1);
	can1Tx0Data[6] = 0x00;
	can1Tx0Data[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, can1Tx0Data, &TxMailBox);
}

void speed_closedloop_control(uint8_t id, int32_t speedControl){
	canTxHeader.StdId = 0x140 + id;
	can1Tx0Data[0] = 0xA2;
	can1Tx0Data[1] = 0x00;
	can1Tx0Data[2] = 0x00;
	can1Tx0Data[3] = 0x00;
	can1Tx0Data[4] = *(uint8_t *)(&speedControl);
	can1Tx0Data[5] = *((uint8_t *)(&speedControl) + 1);
	can1Tx0Data[6] = *((uint8_t *)(&speedControl) + 2);
	can1Tx0Data[7] = *((uint8_t *)(&speedControl) + 3);
	HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, can1Tx0Data, &TxMailBox);
}

void singleloop_angle_control(uint8_t id, int32_t speedControl){
	canTxHeader.StdId = 0x140 + id;
	can1Tx0Data[0] = 0xA5;
	can1Tx0Data[1] = spinDirection;
	can1Tx0Data[2] = 0x00;
	can1Tx0Data[3] = 0x00;
	can1Tx0Data[4] = *(uint8_t *)(&angleControl);
	can1Tx0Data[5] = *((uint8_t *)(&angleControl) + 1);
	can1Tx0Data[6] = *((uint8_t *)(&angleControl) + 2);
	can1Tx0Data[7] = *((uint8_t *)(&angleControl) + 3);
	HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, can1Tx0Data, &TxMailBox);
}

void singleloop_angle_read(uint8_t id, int32_t speedControl){
	canTxHeader.StdId = 0x140 + id;
	can1Tx0Data[0] = 0x94;
	can1Tx0Data[1] = 0x00;
	can1Tx0Data[2] = 0x00;
	can1Tx0Data[3] = 0x00;
	can1Tx0Data[4] = 0x00;
	can1Tx0Data[5] = 0x00;
	can1Tx0Data[6] = 0x00;
	can1Tx0Data[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, can1Tx0Data, &TxMailBox);
}

void multiloop_angle_control(uint8_t id, int32_t speedControl){
	canTxHeader.StdId = 0x140 + id;
	can1Tx0Data[0] = 0xA3;
	can1Tx0Data[1] = 0x00;
	can1Tx0Data[2] = 0x00;
	can1Tx0Data[3] = 0x00;
	can1Tx0Data[4] = *(uint8_t *)(&multiAngleControl);
	can1Tx0Data[5] = *((uint8_t *)(&multiAngleControl) + 1);
	can1Tx0Data[6] = *((uint8_t *)(&multiAngleControl) + 2);
	can1Tx0Data[7] = *((uint8_t *)(&multiAngleControl) + 3);
	HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, can1Tx0Data, &TxMailBox);
}

void multiloop_angle_read(uint8_t id, int32_t speedControl){
	canTxHeader.StdId = 0x140 + id;
	can1Tx0Data[0] = 0x92;
	can1Tx0Data[1] = 0x00;
	can1Tx0Data[2] = 0x00;
	can1Tx0Data[3] = 0x00;
	can1Tx0Data[4] = 0x00;
	can1Tx0Data[5] = 0x00;
	can1Tx0Data[6] = 0x00;
	can1Tx0Data[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, can1Tx0Data, &TxMailBox);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == B1_Pin){

		//change mode whether button is pressed 0->1->2 cyclic
		mode++;
		if (mode == 3){
			mode = 0;
		}

		if(mode%3 == 0){
			speed_closedloop_control(1, 0);
			//speed_closedloop_control(2, 0);
		}
		if(mode%3 == 1){
			speed_closedloop_control(1, 72000);
			//speed_closedloop_control(2, 0);
		}
		if(mode%3 == 2){
			speed_closedloop_control(1, 0);
			//speed_closedloop_control(2, 72000);
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, can1Rx0Data);
	datacheck = 1;
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);  //start CAN

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //activate notification for when FIFO 0 is pending

  canTxHeader.DLC = 8;  //Length of data
  canTxHeader.IDE = CAN_ID_STD; //standard id, as opposed to extended
  canTxHeader.RTR = CAN_RTR_DATA;
  canTxHeader.StdId = 0x140; //0x140+ ID(1~32)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // if message is received
	  if (datacheck){
		  //blink led
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

		  //reconstruct encoder value
		  uint8_t highByte = can1Rx0Data[7];
		  uint8_t lowByte = can1Rx0Data[6];
		  uint16_t encoderVal = (highByte << 8) | lowByte;

		  //display data through uart
		  //HAL_UART_Transmit(&huart2, (uint8_t*)&encoderVal, sizeof(encoderVal), 10);
		  datacheck = 0;
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterMaskIdHigh = 0x7f0 << 5;
  canfilterconfig.FilterIdHigh = 0x140 << 5;
  canfilterconfig.FilterMaskIdLow = 0x7f0 << 5;
  canfilterconfig.FilterIdLow = 0x140 << 5;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);


  /* USER CODE END CAN1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
