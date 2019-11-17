/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <string.h> // strlen
#include <stdlib.h>  // atoi
#include <stdio.h>  // sprintf
#include <math.h>   // sin\cos
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  DATA_FRAME_12 = 12,
  DATA_FRAME_9 = 9,
  DATA_FRAME_BOTH = 0,
  DATA_FRAME_INVALID
} eDATA_FRAME_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TERMINAL_USART  USART2
#define DSPACE_USART  USART1

#define terminal_huart huart2
#define dspace_huart huart1

#define htim_1s    htim21
#define htim_2s    htim2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static char buffer[1000];
uint8_t buffer_tx[10]={69, 77, 67, 85, 46, 69, 85, 13, 10};
volatile uint8_t buffer_rx[3000];
uint8_t received_data, received_data_from_dspace;
volatile uint8_t received_buffer_size = 0;
volatile uint8_t command_handler_request = 0; // 0=NO, 1=Process

volatile int8_t led3_blink = -1;      // 0=OFF, 1=blink, 2=ON
volatile uint8_t dSpace_Alive = 1;    // =0, dSpace is alive, !=0, dSpace is disconnected
volatile uint8_t dSpace_req_data = 0; // =0, dSpace not request data, =1, dSpace requested for data

// Configuration - Variables
uint8_t b_encoder_1_enable = 1;
uint8_t b_encoder_2_enable = 1;
uint8_t b_encoder_3_enable = 1;
uint8_t b_mcu_dspace_enable = 0;
uint8_t b_mcu_pc_terminal_enable = 0;
uint8_t b_setting_mode = 0;
eDATA_FRAME_t data_frame_mcu2termial = DATA_FRAME_12;
int32_t b_frequency_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */

uint32_t Read_SSI_SPIx(int index);
uint32_t raw_data[3];              // Raw-Data of Encoders

int Cal_EncoderData(eDATA_FRAME_t data_frame, uint32_t raw_data[3],
		uint8_t *array_data, int32_t angle_data[3]);
uint8_t array_data[13];            // maximum 13-byte array of data
int32_t angle_data[3];             // angle in degree - scaled 1000x

void Generate_CRCTable();
uint8_t crc_table[256];            // CRC look-up table

uint8_t CRCChecksum(uint8_t *data, uint8_t data_len);
uint8_t CRC_Cal = 0;               // Computed CRC


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// BUTTON Press Interrupt Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  }

}

// UART RECEIVE Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // DSpace Connected UaRT
  if(huart->Instance == DSPACE_USART)
  {
	if (received_data_from_dspace == 'd') {
      dSpace_Alive = 0; // dSpace is alive!
      dSpace_req_data = 1;
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}
	if (received_data_from_dspace == 'n'){
	  dSpace_Alive = 3; // set to allow LED-2 blink
	  dSpace_req_data = 1;
	}
	HAL_UART_Receive_IT(&dspace_huart, &received_data_from_dspace, 1);
  }

  // PC-Terminal Connected UaRT
  if(huart->Instance == TERMINAL_USART)
  {
    if (received_buffer_size == 0){
      if (received_data == (uint8_t)'@'){
        buffer_rx[received_buffer_size] = received_data;
        received_buffer_size += 1;
        HAL_TIM_Base_Start_IT(&htim21); // start 1s-period timer

        command_handler_request = 0;
      }
      else {
        // do nothing
      }
    }
    else {
      if (received_data == (uint8_t)'#'){
        buffer_rx[received_buffer_size] = received_data;
        received_buffer_size += 1;

        HAL_TIM_Base_Stop_IT(&htim21); // stop timer
        command_handler_request = 1;
      }
      else {
        buffer_rx[received_buffer_size] = received_data;
        received_buffer_size += 1;
      }
    }
    HAL_UART_Receive_IT(&terminal_huart, &received_data, 1);
  }
}


void CommandDecoder();

// Command Handler
void Cmd_Handler(){
  uint8_t i = 0;
  if (command_handler_request > 0) {
    if (command_handler_request == 1 && received_buffer_size > 0){
      MY_PRINT_0(buffer, "Command RECEIVED: ");
    }

    // After 1-sec since the starting command '@' has been received, there is no '#' comming
    // in the 1-sec timeout handler, command_handler_request is set to 2.
    if (command_handler_request == 2 && received_buffer_size > 0){
      MY_PRINT_0(buffer, "BAD Command RECEIVED: ");
    }
    // a valid command received
    if (command_handler_request >= 1 && received_buffer_size > 0) {
      for (i = 0; i < received_buffer_size; i++){
        buffer[i] = buffer_rx[i];
      }
      buffer[received_buffer_size] = '\n';
      HAL_UART_Transmit(&terminal_huart, (uint8_t *) buffer, received_buffer_size + 1, 5000);

      // Command Decoder - SOlving here
      CommandDecoder();
    }

    // clear the buffer
    command_handler_request = 0;
    for (i = 0; i < received_buffer_size; i++){
        buffer_rx[i] = 0;
    }
    received_buffer_size = 0;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int num_byte = -1;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */

  // Initialize UART-2 REceive Complete Interrupt with 1-byte buffer
  HAL_UART_Receive_IT(&terminal_huart, &received_data, 1);

  HAL_UART_Receive_IT(&dspace_huart, &received_data_from_dspace, 1);

  // Initialize UART-2 Transmit Complete Interrupt with 10-byte buffer
  HAL_UART_Transmit_IT(&terminal_huart, buffer_tx, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim_2s); // 2s-period timer start

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (b_frequency_ms > 0)
	             HAL_Delay(b_frequency_ms);
    Cmd_Handler();

    // Read raw-data from encoders
    if (b_mcu_dspace_enable == 1 || b_mcu_pc_terminal_enable == 1){
      if (b_encoder_1_enable == 1)
        raw_data[0] = Read_SSI_SPIx(0);
      if (b_encoder_2_enable == 1)
        raw_data[1] = Read_SSI_SPIx(1);
      if (b_encoder_3_enable == 1)
        raw_data[2] = Read_SSI_SPIx(2);
    }

    // Prepare data and send to Terminal
    if (b_setting_mode == 0 && b_mcu_pc_terminal_enable == 1){
      // convert encoder-data into an array bytes of data
      num_byte = Cal_EncoderData(data_frame_mcu2termial, raw_data,
			  array_data, angle_data);
      if (num_byte > 0)
      {
    	// Compute CRC code
        CRC_Cal = CRCChecksum(array_data, num_byte);

        // Save the computed CRC into the last element of the array
        array_data[num_byte] = CRC_Cal;

        // Increase the size of the array
        num_byte += 1;

        // Now send the array to Terminal
        HAL_UART_Transmit(&terminal_huart, array_data, num_byte, 5000);
       }


    }

    // Prepare data and transmit to DSpace
    if (dSpace_req_data == 1 && b_mcu_dspace_enable == 1){
      // convert encoder-data into an array bytes of data
      num_byte = Cal_EncoderData(DATA_FRAME_9, raw_data, array_data, angle_data);

      if (num_byte > 0)
      {
        // Compute CRC code
        CRC_Cal = CRCChecksum(array_data, num_byte);

        // Save the computed CRC into the last element of the array
        array_data[num_byte ] = CRC_Cal;

        // Increase the size of the array
        num_byte += 1;

        // Now send the array to dSpace
        HAL_UART_Transmit(&dspace_huart, array_data, num_byte, 5000);
      }

      dSpace_req_data = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 3999;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 1000;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
uint32_t Read_SSI_SPIx(int index) {
  uint32_t data1 = 0;
  return data1;
}

/**
  * @brief GPIO Initialization Function
  * @param [IN] eDATA_FRAME_t: data_frame (DATA_FRAME)
  * @param [IN] uint32_t raw_data[3]: raw-data of encoders
  * @param [OUT] uint8_t *array_data (not including CRC)
  * @param [OUT] int32_t angle_data[3]
  * @retval int: # of bytes returned
  */
int Cal_EncoderData(eDATA_FRAME_t data_frame, uint32_t raw_data[3], uint8_t *array_data, int32_t angle_data[3])
{
  uint32_t enc_data[3];              // Encoder Data either RAW-DATA or ANGLE-DATA
  int idx = 0, i;
  uint32_t scale = 1000;
  uint32_t value;
  float convert_factor = 360.0f * scale / MAX_VALUE_19_BIT;

  switch (data_frame){
    case DATA_FRAME_12:
      for (idx = 0; idx < 3; idx++)
      {
        value = raw_data[idx] & (uint32_t)0x007FFFF0;
        value = value / 16;         // last bit is ERROR bit, value >>= 4
        //enc_data[idx] = value;
        enc_data[idx] = (uint32_t)(value * convert_factor);
      }
      break;
    case DATA_FRAME_9:
      for (idx = 0; idx < 3; idx++)
      {
        enc_data[idx] = raw_data[idx];
      }
      break;
  default:
    return -1;
  }
  idx = 0;

  for (i = 0; i < 3; i++){
    array_data[idx++] = (uint8_t)(enc_data[i] & 0xFF);                   // 1st byte
    array_data[idx++] = (uint8_t)((uint32_t)(enc_data[i]>>8) & 0xFF);    // 2nd byte
    array_data[idx++] = (uint8_t)((uint32_t)(enc_data[i]>>16) & 0xFF);   // 3rd byte
    if (data_frame == DATA_FRAME_12)
      array_data[idx++] = (uint8_t)((uint32_t)(enc_data[i]>>24) & 0xFF); // 4th byte
  }

  return idx;
}

// Generate CRC Checksum Table
// Source: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
void Generate_CRCTable() {
  const uint8_t generator = 0x1D;
  uint8_t currentByte;
  for (int divident = 0; divident < 256; divident++) {
    currentByte = (uint8_t) divident;
    // calculate the CRC-8 value for current byte
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((currentByte & 0x80) != 0) { // 0x80 = 0b10000000
        currentByte <<= 1;           // Shift-to-left 1 bit
        currentByte ^= generator;    // XOR
      }
      else {
        currentByte <<= 1;           // Shift-to-left 1 bit
      }
    }
    // store CRC value in lookup table
    crc_table[divident] = currentByte;
  }
  crc_table[0] = 1;
}

uint8_t CRCChecksum(uint8_t *data, uint8_t data_len){
  uint8_t crc = 0xAB; // CRC_0
  uint8_t tmp;
  for (int i = 0; i < (int)data_len; i++) {
    tmp = data[i] ^ crc;
    crc = crc_table[tmp];
  }

  return crc;
}


void CommandDecoder()
{
  // Decode the command
  uint8_t cmd_grp = buffer_rx[1];
  uint8_t data_len = buffer_rx[2];
  char data_buff[256];
  uint8_t cmd_sub = 0xFF;
  if (data_len >= 1){
    cmd_sub = buffer_rx[3];
  }
  switch (cmd_grp){
    ////////////////////////////////////////////////////////////////////////////////////
    case '0':
      if (cmd_sub == '0'){ // setting-menu
        MY_PRINT_0(buffer, "@010# : Setting - Mode\n");
        MY_PRINT_0(buffer, "@011# : Running - Mode\n");
        MY_PRINT_0(buffer, "@111# : Turn ON Encoder 1\n");
        MY_PRINT_0(buffer, "@110# : Turn OFF Encoder 1\n");
        MY_PRINT_0(buffer, "@211# : Turn ON Encoder 2\n");
        MY_PRINT_0(buffer, "@210# : Turn OFF Encoder 2\n");
        MY_PRINT_0(buffer, "@311# : Turn ON Encoder 3\n");
        MY_PRINT_0(buffer, "@310# : Turn OFF Encoder 3\n");
        MY_PRINT_0(buffer, "@410# : MCU Sends RAW_DATA to TERMINAL\n");
        MY_PRINT_0(buffer, "@411# : MCU Sends ANGLE_DATA to TERMINAL\n");
        MY_PRINT_0(buffer, "@73100# : Set frequency of 100ms\n");
        MY_PRINT_0(buffer, "@810# : Update Encoders to TERMINAL\n");
        MY_PRINT_0(buffer, "@811# : Stop Updating to TERMINAL\n");
        MY_PRINT_0(buffer, "@910# : Update Encoders to DSpace\n");
        MY_PRINT_0(buffer, "@911# : Stop Updating Encoders to DSpace\n");
        b_setting_mode = 1;
        led3_blink = 1;
      }
      else if (cmd_sub == '1') { // Enter running mode
    	MY_PRINT_0(buffer, "Enter Running Mode\n");
        b_setting_mode = 0;
        if (b_mcu_pc_terminal_enable == 1){ // LED3 is ON!
          HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
          led3_blink = 2;
        }
        else { // LED3 is OFF!
          HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
          led3_blink = 0;
        }
      }
      else {
    	MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '1':              // Encoder -1
      if (cmd_sub == '1'){ // turn on
    	MY_PRINT_0(buffer, "TURN-ON Encoder-1\n");
        b_encoder_1_enable = 1;
      }
      else if (cmd_sub == '0'){ // turn off
        MY_PRINT_0(buffer, "TURN-OFF Encoder-1\n");
        b_encoder_1_enable = 0;
      }
      else {
    	MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '2':              // Encoder -2
      if (cmd_sub == '1'){ // turn on
        MY_PRINT_0(buffer, "TURN-ON Encoder-2\n");
        b_encoder_2_enable = 1;
      }
      else if (cmd_sub == '0'){ // turn off
        MY_PRINT_0(buffer, "TURN-OFF Encoder-2\n");
        b_encoder_2_enable = 0;
      }
      else {
    	MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '3':              // Encoder -3
      if (cmd_sub == '1'){ // turn on
    	MY_PRINT_0(buffer, "TURN-ON Encoder-3\n");
        b_encoder_3_enable = 1;
      }
      else if (cmd_sub == '0'){ // turn off
    	MY_PRINT_0(buffer, "TURN-OFF Encoder-3\n");
        b_encoder_3_enable = 0;
      }
      else {
    	MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '4':              // DATA Format
      if (cmd_sub == '0')  { // MCU sends RAW-Data to Terminal
    	MY_PRINT_0(buffer, "MCU sends RAW-Data to Terminal\n");
        data_frame_mcu2termial = DATA_FRAME_9;
      }
      else if (cmd_sub == '1')  { // MCU sends ANGLE-Data to Terminal
    	MY_PRINT_0(buffer, "MCU sends ANGLE-Data to Terminal\n");
        data_frame_mcu2termial = DATA_FRAME_12;
      }
      else {
    	MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
        data_frame_mcu2termial = DATA_FRAME_INVALID;
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '7': // Frequency
      if (data_len <= '9' && data_len >= '0'){ //  maximum of 4-byte frequency
        for (int i = 0; i < data_len - '0'; i++){
        	data_buff[i] = buffer_rx[3+i];
        }
        data_buff[data_len - '0'] = '\0';
        b_frequency_ms = atoi(data_buff);
        MY_PRINT_F(buffer, "Frequency is set to %ld (ms)\n", b_frequency_ms);
      }
      else {
    	MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), data_len(%c)\n", cmd_grp, data_len);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '8': // Update Encoders to Terminal
      if (cmd_sub == '1'){ // Encoder Data is sent to Terminal
        b_mcu_pc_terminal_enable = 1;
        MY_PRINT_0(buffer, "Update Encoders to Terminal\n");
      }
      else if (cmd_sub == '0') {
        b_mcu_pc_terminal_enable = 0;
        MY_PRINT_0(buffer, "Stop Updating Encoders to Terminal\n");
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '9': // Update Encoders to DSpace
      if (cmd_sub == '1'){ // Encoder Data is sent to Dspace
        b_mcu_dspace_enable = 1;
        MY_PRINT_0(buffer, "Update Encoders to DSpace\n");
      }
      else if (cmd_sub == '0') {
        b_mcu_dspace_enable = 0;
        MY_PRINT_0(buffer, "Stop Updating Encoders to DSpace\n");
      }
      else {
    	MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    default:
    	MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c)\n", cmd_grp);
      break;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
