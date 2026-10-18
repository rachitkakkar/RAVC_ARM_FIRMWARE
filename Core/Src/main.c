/* USER CODE BEGIN Header */
/**
 * @file main.c
 * @brief Five servo-PWM outputs with an in-firmware mock UART test.
 * Copyright (c) 2026 STMicroelectronics. All rights reserved.
 * Licensed under the LICENSE file in the project root; otherwise AS-IS.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_MIN_US       1000U
#define SERVO_NEUTRAL_US   1500U
#define SERVO_MAX_US       2000U
#define SERVO_COUNT        5U
#define SERVO_ID_ARM_BASE      1U  /* PA0,  TIM2_CH1 */
#define SERVO_ID_JOINT_ARM     2U  /* PA2,  TIM2_CH3 */
#define SERVO_ID_ARM_ROTATION  3U  /* PA3,  TIM2_CH4 */
#define SERVO_ID_CLAW          4U  /* PE9,  TIM1_CH1 */
#define SERVO_ID_WRIST         5U  /* PE11, TIM1_CH2 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define SERVO_ARM_BASE_TIMER       (&htim2)
#define SERVO_ARM_BASE_CHANNEL     TIM_CHANNEL_1

#define SERVO_JOINT_ARM_TIMER      (&htim2)
#define SERVO_JOINT_ARM_CHANNEL    TIM_CHANNEL_3

#define SERVO_ARM_ROTATION_TIMER   (&htim2)
#define SERVO_ARM_ROTATION_CHANNEL TIM_CHANNEL_4

#define SERVO_CLAW_TIMER           (&htim1)
#define SERVO_CLAW_CHANNEL         TIM_CHANNEL_1

#define SERVO_WRIST_TIMER          (&htim1)
#define SERVO_WRIST_CHANNEL        TIM_CHANNEL_2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef Servo_SetPulseUs(uint32_t servo, uint32_t pulse_us);
HAL_StatusTypeDef Servo_SetCommand(uint32_t servo, int32_t permille);
void Servo_NeutralAll(void);
static void Servo_Start(void);


#define ROS_CMD_TIMEOUT_MS 250U
#define ROS_RX_BUFFER_SIZE 64U

#define MOCK_UART_FIRMWARE
#ifndef MOCK_UART_FIRMWARE /* Physical UART reception is disabled during the mock test. */
static uint8_t ros_rx_byte;
#endif
static char ros_rx_buffer[ROS_RX_BUFFER_SIZE];
static volatile uint16_t ros_rx_len = 0;
static volatile uint8_t ros_msg_ready = 0;
static uint32_t ros_last_cmd_ms = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifndef MOCK_UART_FIRMWARE /* Original UART reception and ROS handler retained for reference. */
static void ROS_UART_Start(void)
{
    HAL_UART_Receive_IT(&huart3, &ros_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
        return;

    char c = (char)ros_rx_byte;

    if (c == '\n') {
        if (ros_rx_len > 0 && !ros_msg_ready) {
            ros_rx_buffer[ros_rx_len] = '\0';
            ros_msg_ready = 1;
        }
    } else if (c != '\r' && !ros_msg_ready) {
        if (ros_rx_len < ROS_RX_BUFFER_SIZE - 1) {
            ros_rx_buffer[ros_rx_len++] = c;
        } else {
            ros_rx_len = 0;  // Discard oversized packet
        }
    }

    HAL_UART_Receive_IT(&huart3, &ros_rx_byte, 1);
}

static void ROS_ProcessCommand(char *line)
{
    char prefix;
    int m1, m2, m3, m4, m5, m6;
    char extra;

    // Expected packet: M,100,0,-100
    if (sscanf(line, " %c , %d , %d , %d , %d , %d , %d %c",
               &prefix, &m1, &m2, &m3, &m4, &m5, &m6, &extra) != 4)
        return;

    if (prefix != 'M')
        return;

    // Clamp motor commands to [-1000, 1000]
    if (m1 < -1000) m1 = -1000;
    if (m1 >  1000) m1 =  1000;
    if (m2 < -1000) m2 = -1000;
    if (m2 >  1000) m2 =  1000;
    if (m3 < -1000) m3 = -1000;
    if (m3 >  1000) m3 =  1000;
    if (m4 < -1000) m4 = -1000;
    if (m4 >  1000) m4 =  1000;
    if (m5 < -1000) m5 = -1000;
    if (m5 >  1000) m5 =  1000;
    if (m6 < -1000) m6 = -1000;
    if (m6 >  1000) m6 =  1000;


    if (Servo_SetCommand(1, m1) == HAL_OK &&
        Servo_SetCommand(2, m2) == HAL_OK &&
        Servo_SetCommand(3, m3) == HAL_OK &&
		    Servo_SetCommand(1, m4) == HAL_OK &&
        Servo_SetCommand(2, m5) == HAL_OK &&
        Servo_SetCommand(3, m6) == HAL_OK) {
        ros_last_cmd_ms = HAL_GetTick();
    }
}

#endif

/* Five-command mock parser; exercises the existing buffer/update path. */
static void Mock_ProcessCommand(char *line)
{
    char prefix, extra;
    int cmd[SERVO_COUNT];

    if (sscanf(line, " %c , %d , %d , %d , %d , %d %c",
               &prefix, &cmd[0], &cmd[1], &cmd[2], &cmd[3], &cmd[4],
               &extra) != 6 || prefix != 'M')
        return;

    for (uint32_t i = 0; i < SERVO_COUNT; ++i) {
        /* Servo_SetCommand clamps each value to [-1000, 1000]. */
        if (Servo_SetCommand(i + 1U, cmd[i]) != HAL_OK)
            return;
    }
    ros_last_cmd_ms = HAL_GetTick();
}

static void ROS_UART_Update(void)
{
    char line[ROS_RX_BUFFER_SIZE];
    uint16_t len = 0;

    __disable_irq();

    if (ros_msg_ready) {
        len = ros_rx_len;
        memcpy(line, ros_rx_buffer, len);
        line[len] = '\0';

        ros_rx_len = 0;
        ros_msg_ready = 0;
    }

    __enable_irq();

    if (len > 0)
        Mock_ProcessCommand(line);

    // Stop commanded motion if communication times out
    if ((HAL_GetTick() - ros_last_cmd_ms) > ROS_CMD_TIMEOUT_MS)
        Servo_NeutralAll();
}

/* Called from main only; bypasses UART interrupts and newline framing. */
static void Mock_UART_Packet(const char *packet)
{
    size_t len = strlen(packet);
    if (len == 0 || len >= ROS_RX_BUFFER_SIZE || ros_msg_ready)
        return;

    memcpy(ros_rx_buffer, packet, len + 1U);
    ros_rx_len = (uint16_t)len;
    ros_msg_ready = 1U;
    ROS_UART_Update();
}

/* Servo IDs are 1..5. Calls are intended from one control task.
 * Compare preload makes changes take effect at the next 5 ms frame.
 * These are motor-output commands, NOT joint-angle or regulated speed targets.
 */
HAL_StatusTypeDef Servo_SetPulseUs(uint32_t servo, uint32_t pulse_us)
{
  if ((servo < 1U) || (servo > SERVO_COUNT)) return HAL_ERROR;
  if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
  if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

  switch (servo) {
    case SERVO_ID_ARM_BASE:
      __HAL_TIM_SET_COMPARE(SERVO_ARM_BASE_TIMER, SERVO_ARM_BASE_CHANNEL, pulse_us);
      break;
    case SERVO_ID_JOINT_ARM:
      __HAL_TIM_SET_COMPARE(SERVO_JOINT_ARM_TIMER, SERVO_JOINT_ARM_CHANNEL, pulse_us);
      break;
    case SERVO_ID_ARM_ROTATION:
      __HAL_TIM_SET_COMPARE(SERVO_ARM_ROTATION_TIMER, SERVO_ARM_ROTATION_CHANNEL, pulse_us);
      break;
    case SERVO_ID_CLAW:
      __HAL_TIM_SET_COMPARE(SERVO_CLAW_TIMER, SERVO_CLAW_CHANNEL, pulse_us);
      break;
    case SERVO_ID_WRIST:
      __HAL_TIM_SET_COMPARE(SERVO_WRIST_TIMER, SERVO_WRIST_CHANNEL, pulse_us);
      break;
    default:
      return HAL_ERROR;
  }
  return HAL_OK;
}

/* -1000 = full reverse, 0 = neutral, +1000 = full forward. */
HAL_StatusTypeDef Servo_SetCommand(uint32_t servo, int32_t permille)
{
  if (permille < -1000) permille = -1000;
  if (permille > 1000) permille = 1000;
  return Servo_SetPulseUs(servo, (uint32_t)(1500 + permille / 2));
}

void Servo_NeutralAll(void)
{
  for (uint32_t servo = 1U; servo <= SERVO_COUNT; ++servo)
    (void)Servo_SetPulseUs(servo, SERVO_NEUTRAL_US);
}

static void Servo_Start(void)
{
  /* Detect an unregenerated or incorrectly configured tim.c. */
  if ((htim2.Instance != TIM2) || (htim2.Init.Prescaler != 63U) ||
      (htim2.Init.Period != 4999U) ||
      (htim1.Instance != TIM1) || (htim1.Init.Prescaler != 63U) ||
      (htim1.Init.Period != 4999U)) Error_Handler();

  /* Enable OC preloads */
  __HAL_TIM_ENABLE_OCxPRELOAD(SERVO_ARM_BASE_TIMER, SERVO_ARM_BASE_CHANNEL);
  __HAL_TIM_ENABLE_OCxPRELOAD(SERVO_JOINT_ARM_TIMER, SERVO_JOINT_ARM_CHANNEL);
  __HAL_TIM_ENABLE_OCxPRELOAD(SERVO_ARM_ROTATION_TIMER, SERVO_ARM_ROTATION_CHANNEL);
  __HAL_TIM_ENABLE_OCxPRELOAD(SERVO_CLAW_TIMER, SERVO_CLAW_CHANNEL);
  __HAL_TIM_ENABLE_OCxPRELOAD(SERVO_WRIST_TIMER, SERVO_WRIST_CHANNEL);

  Servo_NeutralAll();

  /* Load the prescaler and neutral compare registers before enabling output. */
  if (HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

  /* Start PWM outputs */
  if (HAL_TIM_PWM_Start(SERVO_ARM_BASE_TIMER, SERVO_ARM_BASE_CHANNEL) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(SERVO_JOINT_ARM_TIMER, SERVO_JOINT_ARM_CHANNEL) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(SERVO_ARM_ROTATION_TIMER, SERVO_ARM_ROTATION_CHANNEL) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(SERVO_CLAW_TIMER, SERVO_CLAW_CHANNEL) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(SERVO_WRIST_TIMER, SERVO_WRIST_CHANNEL) != HAL_OK) Error_Handler();
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* MX_USART3_UART_Init(); -- physical UART bypassed for this mock test. */
  /* USER CODE BEGIN 2 */
  Servo_Start();
  HAL_Delay(1000); /* Observe neutral on all five outputs before the test. */
  /* Expected us: base=1550, joint=1500, rotation=1450, claw=1600, wrist=1400.
   * Send once; ROS_UART_Update returns all five to neutral after >250 ms.
   */
  Mock_UART_Packet("M,100,0,-100,200,-200");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ROS_UART_Update();
    HAL_Delay(5);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Stop pulse transmission on a detected HAL/init error. */
  if (htim2.Instance == TIM2)
  {
    TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    TIM2->CR1 &= ~TIM_CR1_CEN;
  }
  __disable_irq();
  while (1) {}
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
