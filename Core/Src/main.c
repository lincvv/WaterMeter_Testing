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
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define  CIRCLE_RADIUS        37
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

SRAM_HandleTypeDef hsram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCDTask */
osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for QspiTask */
osThreadId_t QspiTaskHandle;
const osThreadAttr_t QspiTask_attributes = {
  .name = "QspiTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for sensTask */
osThreadId_t sensTaskHandle;
const osThreadAttr_t sensTask_attributes = {
  .name = "sensTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GenWaterTask */
osThreadId_t GenWaterTaskHandle;
const osThreadAttr_t GenWaterTask_attributes = {
  .name = "GenWaterTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for xQueuePower */
osMessageQueueId_t xQueuePowerHandle;
const osMessageQueueAttr_t xQueuePower_attributes = {
  .name = "xQueuePower"
};
/* Definitions for xQueueMaxCurrent */
osMessageQueueId_t xQueueMaxCurrentHandle;
const osMessageQueueAttr_t xQueueMaxCurrent_attributes = {
  .name = "xQueueMaxCurrent"
};
/* Definitions for xQueueLCDMaxCurrent */
osMessageQueueId_t xQueueLCDMaxCurrentHandle;
const osMessageQueueAttr_t xQueueLCDMaxCurrent_attributes = {
  .name = "xQueueLCDMaxCurrent"
};
/* Definitions for xQueueSensMaxCurrent */
osMessageQueueId_t xQueueSensMaxCurrentHandle;
const osMessageQueueAttr_t xQueueSensMaxCurrent_attributes = {
  .name = "xQueueSensMaxCurrent"
};
/* Definitions for xQueueRstMaxCurrent */
osMessageQueueId_t xQueueRstMaxCurrentHandle;
const osMessageQueueAttr_t xQueueRstMaxCurrent_attributes = {
  .name = "xQueueRstMaxCurrent"
};
/* USER CODE BEGIN PV */
//uint32_t ts_count = 0;
TS_StateTypeDef  TS_State = {0};
//__attribute__((section(".noinit")))
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2S3_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartLCDTask(void *argument);
void StartQspiTask(void *argument);
void StartSensTask(void *argument);
void StartLightSensTask(void *argument);
void StartGenWaterTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void _putchar(char character)
{
//    ITM_SendChar(character);
    HAL_UART_Transmit(&huart2, &character, 1, 100);
}

void lcd_initialization()
{
    BSP_LCD_Init();
    BSP_LCD_Clear(LCD_COLOR_GRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_SetFont(&Font24);
}

void lcd_display_message(uint16_t LINE, uint32_t val, float val2, char* msg_str)
{
    static uint8_t dataQueue[16];
    sprintf((char *) dataQueue, msg_str, val2 ? val2 : val);
    BSP_LCD_DisplayStringAt(0, LINE, dataQueue, LEFT_MODE);
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
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2S3_Init();
  MX_QUADSPI_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xQueuePower */
  xQueuePowerHandle = osMessageQueueNew (3, sizeof(POWER_t), &xQueuePower_attributes);

  /* creation of xQueueMaxCurrent */
  xQueueMaxCurrentHandle = osMessageQueueNew (10, sizeof(float), &xQueueMaxCurrent_attributes);

  /* creation of xQueueLCDMaxCurrent */
  xQueueLCDMaxCurrentHandle = osMessageQueueNew (1, sizeof(uint32_t), &xQueueLCDMaxCurrent_attributes);

  /* creation of xQueueSensMaxCurrent */
  xQueueSensMaxCurrentHandle = osMessageQueueNew (1, sizeof(uint8_t), &xQueueSensMaxCurrent_attributes);

  /* creation of xQueueRstMaxCurrent */
  xQueueRstMaxCurrentHandle = osMessageQueueNew (1, sizeof(uint8_t), &xQueueRstMaxCurrent_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LCDTask */
  LCDTaskHandle = osThreadNew(StartLCDTask, NULL, &LCDTask_attributes);

  /* creation of QspiTask */
  QspiTaskHandle = osThreadNew(StartQspiTask, NULL, &QspiTask_attributes);

  /* creation of sensTask */
  sensTaskHandle = osThreadNew(StartSensTask, NULL, &sensTask_attributes);

  /* creation of LightSensTask */
//  LightSensTaskHandle = osThreadNew(StartLightSensTask, NULL, &LightSensTask_attributes);

  /* creation of GenWaterTask */
//  GenWaterTaskHandle = osThreadNew(StartGenWaterTask, NULL, &GenWaterTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    printf("START");

//    HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
//    HAL_I2C_Master_Seq_Transmit_DMA()

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//            if(TS_State.touchDetected)
//      {
//          x1 = TS_State.touchX[0];
//          y1 = TS_State.touchY[0];
//
//          if((y1>83) && (y1<157)){
//              BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
//              BSP_LCD_FillCircle(60, 120, CIRCLE_RADIUS);
//              BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//              BSP_LCD_FillCircle(180, 120, CIRCLE_RADIUS-1);
//              BSP_LCD_FillCircle(60, 200, CIRCLE_RADIUS-1);
//              BSP_LCD_FillCircle(180, 200, CIRCLE_RADIUS-1);
//          }
//      }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 0;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_5_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 142;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 34;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED3_Pin|LED4_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_BLCTRL_Pin|EXT_RESET_Pin|CTP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTGFS_PPWR_EN_GPIO_Port, USB_OTGFS_PPWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM_DATIN3_Pin */
  GPIO_InitStruct.Pin = DFSDM_DATIN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
  HAL_GPIO_Init(DFSDM_DATIN3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BLCTRL_Pin EXT_RESET_Pin CTP_RST_Pin */
  GPIO_InitStruct.Pin = LCD_BLCTRL_Pin|EXT_RESET_Pin|CTP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
  HAL_GPIO_Init(DFSDM_CKOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_SEL_Pin */
  GPIO_InitStruct.Pin = JOY_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM_DATIN0_Pin */
  GPIO_InitStruct.Pin = DFSDM_DATIN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
  HAL_GPIO_Init(DFSDM_DATIN0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_RIGHT_Pin JOY_LEFT_Pin */
  GPIO_InitStruct.Pin = JOY_RIGHT_Pin|JOY_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_UP_Pin JOY_DOWN_Pin LCD_TE_Pin USB_OTGFS_OVRCR_Pin */
  GPIO_InitStruct.Pin = JOY_UP_Pin|JOY_DOWN_Pin|LCD_TE_Pin|USB_OTGFS_OVRCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_CKIN_Pin */
  GPIO_InitStruct.Pin = M2_CKIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(M2_CKIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CODEC_INT_Pin CTP_INT_Pin */
  GPIO_InitStruct.Pin = CODEC_INT_Pin|CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTGFS_PPWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTGFS_PPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTGFS_PPWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_CKINA8_Pin */
  GPIO_InitStruct.Pin = M2_CKINA8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(M2_CKINA8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_DETECT_Pin */
  GPIO_InitStruct.Pin = uSD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
    for (;;) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_PIN);
        printf("Task DEF\n");
        osDelay(pdMS_TO_TICKS(500));
//        if(eTaskGetState(sensTaskHandle) != eSuspended){
//            osDelay(pdMS_TO_TICKS(10000));
//            vTaskSuspend(QspiTaskHandle);
//        }
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLCDTask */
/**
* @brief Function implementing the LCDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN StartLCDTask */
    POWER_t msg;
    float max_c_dyn;
    uint32_t f_data = 0;

    lcd_initialization();

    BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 - 10, BSP_LCD_GetXSize(), 1);

    /* Infinite loop */
    for (;;)
    {
        if (osMessageQueueGet(xQueueLCDMaxCurrentHandle, &f_data, NULL, 10) == osOK)
        {
            BSP_LCD_ClearStringLine(5);
            lcd_display_message(120, f_data, 0, "MAX C = %d");
        }

        if (osMessageQueueGet(xQueueMaxCurrentHandle, &max_c_dyn, NULL, 10) == osOK)
        {
            lcd_display_message(120, 0, max_c_dyn, "MAX C = %.3f");
        }

        if (osMessageQueueGet(xQueuePowerHandle, &msg, NULL, portMAX_DELAY) == osOK) {
            lcd_display_message(47, 0, msg.voltage, "V = %.3f V");
            lcd_display_message(72, 0, msg.current, "C = %.3f mA");
        }

        osDelay(pdMS_TO_TICKS(1));
    }
  /* USER CODE END StartLCDTask */
}

/* USER CODE BEGIN Header_StartQspiTask */
/**
* @brief Function implementing the QspiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartQspiTask */
void StartQspiTask(void *argument)
{
  /* USER CODE BEGIN StartQspiTask */
    printf("Task QSPI\n");
    uint8_t qspi_status = QSPI_ERROR;
    uint32_t max_current = 0;
    uint8_t r_data;
    uint8_t* data;
    qspi_status = BSP_QSPI_Init();
    qspi_status = BSP_QSPI_GetStatus();

    printf("QSPI STATUS - %d\n", qspi_status);

    for (uint8_t i = 0; i < 4; i++) {
        if (BSP_QSPI_Read(&r_data, i, sizeof(r_data)) == QSPI_OK) {
            max_current |= r_data << (8 * i);
            printf("r_data%d - %d\n", i, r_data);


        }
    }

    printf("Q_MAX CURRRENT - %b\n", max_current);

    xTaskNotify(sensTaskHandle, max_current, eSetValueWithOverwrite);

    configASSERT(osMessageQueuePut(xQueueLCDMaxCurrentHandle, &max_current, 0, 0) == osOK);


    /* Infinite loop */
  for(;;)
  {
      if (xTaskNotifyWait(0, 0, &max_current, 10) == pdTRUE) {
          printf("Notify max current -> %d\n", max_current);

          if (BSP_QSPI_Erase_Block(0) == QSPI_OK) {
//               data[0] = (uint8_t*)max_current;
              for (uint8_t i = 0; i < 4; i++) {
                  data = (uint8_t *) &max_current + i;

                  if (BSP_QSPI_Write((uint8_t *) &max_current + i, i, sizeof(uint8_t )) == QSPI_OK) {
                      printf("QSPI W - %d\n", *data);
//                  BSP_QSPI_Read(data, 0, sizeof(data));
//                  configASSERT(osMessageQueuePut(xQueueLCDMaxCurrentHandle, &data, 0, 0) == osOK);

//                  xTaskNotify(LCDTaskHandle, max_current, eSetValueWithOverwrite);
                  }
              }
          }

      }

      if (osMessageQueueGet(xQueueRstMaxCurrentHandle, &r_data, NULL, 10) == osOK) {
          if (BSP_QSPI_Erase_Block(0) == QSPI_OK) {
              for (uint8_t i = 0; i < 4; i++) {
                  if (BSP_QSPI_Write(&r_data, i, sizeof(r_data)) == QSPI_OK) {

                  }
              }
              uint32_t r_current = r_data;
              configASSERT(osMessageQueuePut(xQueueLCDMaxCurrentHandle, &r_current, 0, 0) == osOK);
              xTaskNotify(sensTaskHandle, r_current, eSetValueWithOverwrite);
              printf("CURRENT RST - %d\n", r_data);
          }
      }

    osDelay(pdMS_TO_TICKS(10));


  }
  /* USER CODE END StartQspiTask */
}

/* USER CODE BEGIN Header_StartSensTask */
/**
* @brief Function implementing the sensTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartSensTask */
void StartSensTask(void *argument)
{
  /* USER CODE BEGIN StartSensTask */

    printf("TASK SENS\n");
    POWER_t msg;
    uint8_t data[16];
    float max_current = 0;
    uint32_t f_max_current;

    ina260_t * ina260 = ina260_new(&hi2c2, 0x40);
    ina260_set_config(ina260, iotPower, iomContinuous, ictConvert140us, ictConvert140us, issSample4);
    HAL_StatusTypeDef status_sens = ina260_ready(ina260);

    /* Infinite loop */
    for (;;) {
        if (xTaskNotifyWait(0, 0, &f_max_current, 10) == pdTRUE) {

            sprintf((char *) data, "MAX C = %d", f_max_current);
            max_current = (float)f_max_current;
            printf("SEN TASK NOTIFY - %.3f\n", max_current);
        }

        if(status_sens == HAL_OK){
            ina260_get_voltage(ina260, &msg.voltage);
            ina260_get_current(ina260, &msg.current);
            msg.voltage = msg.voltage / 1000;

        } else{
            printf("SENS ERROR\n");
            msg.voltage = (float)3.34;
            msg.current = (float)2.55;
        }

        configASSERT(osMessageQueuePut(xQueuePowerHandle, &msg, 0, 10) == osOK);

        if (msg.current > max_current)
        {
            max_current = msg.current;
            printf("max current - %.3f\n", max_current);
            configASSERT(osMessageQueuePut(xQueueMaxCurrentHandle, &max_current, 0, 10) == osOK);
            xTaskNotify(QspiTaskHandle, (uint32_t)max_current, eSetValueWithOverwrite);
        }

        if(status_sens == HAL_ERROR){
            vTaskSuspend(sensTaskHandle);
        }

        osDelay(pdMS_TO_TICKS(15));
    }
  /* USER CODE END StartSensTask */
}


/* USER CODE BEGIN Header_StartGenWaterTask */
/**
* @brief Function implementing the GenWaterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGenWaterTask */
void StartGenWaterTask(void *argument)
{
  /* USER CODE BEGIN StartGenWaterTask */
    printf("Task GEN\n");
//    HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
//    HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
  /* Infinite loop */
  for(;;)
  {
//      for(int i = 1; i < 70; i++){
//
//          TIM3->CCR1 = i;
//          osDelay(pdMS_TO_TICKS(1000));
//          printf("TIM-%d\n",__HAL_TIM_GET_COUNTER(&htim3));
//      }
//      osDelay(pdMS_TO_TICKS(10000));


      HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
//      HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
      printf("TIM-%d\n",__HAL_TIM_GET_COUNTER(&htim3));
      osDelay(pdMS_TO_TICKS(5000));
      HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_1);
//      HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_2);
      osDelay(pdMS_TO_TICKS(900000));

  }
  /* USER CODE END StartGenWaterTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
