/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include <inttypes.h>
#include "stdarg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#define TIMER_HZ 1000000UL     // TIM3 tick rate (1 MHz)
#define PPR      330.0f         // <-- CHANGE THIS to your encoder PPR

#define ENC_GPIO_Port GPIOA    // <-- CHANGE if your encoder is on another port
#define ENC_Pin       GPIO_PIN_10  // <-- CHANGE if not PA10
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART2_UART_Init(void);
void myPrintf(const char *fmt,...){
    char buffer[128];   
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)buffer,
                      strlen(buffer),
                      HAL_MAX_DELAY);
  }
/* USER CODE BEGIN PFP */
void Motor_A_SetSpeed(uint8_t speed_percent);
void Motor_B_SetSpeed(uint8_t speed_percent);
void Both_Motors_SetSpeed(uint8_t speed_percent);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== MOTOR A CONTROL (TIM3_CH1 + PC0/PC1) ====================
#define MOTOR_A_FORWARD()   do { \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); \
                            } while(0)

#define MOTOR_A_REVERSE()   do { \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); \
                            } while(0)

#define MOTOR_A_STOP()      do { \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); \
                            } while(0)

// ==================== MOTOR B CONTROL (TIM3_CH2 + PC2/PC3) ====================
#define MOTOR_B_FORWARD()   do { \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); \
                            } while(0)

#define MOTOR_B_REVERSE()   do { \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); \
                            } while(0)

#define MOTOR_B_STOP()      do { \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); \
                              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); \
                            } while(0)

// ==================== BOTH MOTORS COMBINED ====================
#define BOTH_MOTORS_FORWARD()  do { MOTOR_A_FORWARD(); MOTOR_B_FORWARD(); } while(0)
#define BOTH_MOTORS_REVERSE()  do { MOTOR_A_REVERSE(); MOTOR_B_REVERSE(); } while(0)
#define BOTH_MOTORS_STOP()     do { MOTOR_A_STOP(); MOTOR_B_STOP(); } while(0)


// ==================== TIMER DIFF (handles rollover) ====================
static uint32_t timer_diff(uint32_t t1, uint32_t t2)
{
  if (t2 >= t1) return (t2 - t1);
  return ((htim3.Instance->ARR + 1U) - t1 + t2);
}

// ==================== UART HELPERS ====================


// ==================== ENCODER: WAIT FOR FALLING EDGE WITH TIMEOUT ====================
// Returns 1 if edge detected, 0 if timed out
static uint8_t wait_falling_edge_timeout(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    // Wait for HIGH
    while (HAL_GPIO_ReadPin(ENC_GPIO_Port, ENC_Pin) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - start > timeout_ms) return 0;
    }
    // Wait for LOW (falling edge)
    while (HAL_GPIO_ReadPin(ENC_GPIO_Port, ENC_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start > timeout_ms) return 0;
    }
    return 1;
}

// ==================== MOTOR SPEED FUNCTIONS ====================

// Motor A speed (TIM3_CH1)
void Motor_A_SetSpeed(uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;
    uint16_t pwm = (speed_percent * 999) / 100;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}

// Motor B speed (TIM3_CH2)
void Motor_B_SetSpeed(uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;
    uint16_t pwm = (speed_percent * 999) / 100;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}

// Both motors together
void Both_Motors_SetSpeed(uint8_t speed_percent)
{
    Motor_A_SetSpeed(speed_percent);
    Motor_B_SetSpeed(speed_percent);
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  // NOTE: Do NOT put any uart_print or variable references here.
  //       UART is not initialized yet at this point.
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USB_PCD_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  // FIX 1: Start base timer FIRST
  HAL_TIM_Base_Start(&htim3);

  // FIX 2: Start PWM on BOTH channels (was duplicate CH1 before)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // Motor A PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // Motor B PWM  <-- FIXED (was CH1 duplicate)

  // Clean init: stop both motors first
  BOTH_MOTORS_STOP();
  Motor_A_SetSpeed(0);
  Motor_B_SetSpeed(0);

  // Now UART is ready - safe to print
  myPrintf("System initialized.\r\n");
  myPrintf("Starting motors...\r\n");

  // Start both motors forward at full speed
  BOTH_MOTORS_FORWARD();
  Both_Motors_SetSpeed(100);

  myPrintf("Motors running. Encoder RPM polling started...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 1) First falling edge (500ms timeout)
    if (!wait_falling_edge_timeout(500))
    {
        myPrintf("No encoder pulse! Check wiring on PA10.\r\n");
        HAL_Delay(500);
        continue;
    }
    uint32_t t1 = __HAL_TIM_GET_COUNTER(&htim3);

    // 2) Second falling edge (500ms timeout)
    if (!wait_falling_edge_timeout(500))
    {
        myPrintf("No encoder pulse! Check wiring on PA10.\r\n");
        HAL_Delay(500);
        continue;
    }
    uint32_t t2 = __HAL_TIM_GET_COUNTER(&htim3);

    // 3) Compute period in ticks (us)
    uint32_t dt = timer_diff(t1, t2);

    if (dt > 0)
    {
      float freq = (float)TIMER_HZ / (float)dt;   // Hz
      float rpm  = (60.0f * freq) / PPR;

      myPrintf("dt = %lu us | RPM = %.2f\r\n", dt, rpm);
    }
    else
    {
      myPrintf("dt=0, skipping.\r\n");
    }

    HAL_Delay(200);
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief SPI1 Initialization Function
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM3 Initialization Function
  * FIX 3: Added TIM_CHANNEL_2 configuration for Motor B PWM
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;              // 48MHz / (47+1) = 1MHz timer clock
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;                // PWM period -> 1kHz
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // Channel 1 - Motor A
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  // FIX 3: Channel 2 - Motor B (was missing before!)
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
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
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief USB Initialization Function
  */
static void MX_USB_PCD_Init(void)
{
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /* MEMS interrupt pins */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* LED output pins */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Motor direction pins PC0-PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Button + Encoder input PA10 */
  GPIO_InitStruct.Pin = B1_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add logging here */
}
#endif /* USE_FULL_ASSERT */