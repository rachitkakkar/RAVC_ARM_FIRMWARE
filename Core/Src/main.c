<<<<<<< Updated upstream
/* USER CODE BEGIN Header */
/**
 * @file main.c
 * @brief Three SPARK Flex servo-PWM outputs for NEO Vortex motors.
 * Copyright (c) 2026 STMicroelectronics. All rights reserved.
 * Licensed under the LICENSE file in the project root; otherwise AS-IS.
 */
/* USER CODE END Header */
#include "main.h"
#include "tim.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* USER CODE BEGIN PD */
#define SPARK_MIN_US       1000U
#define SPARK_NEUTRAL_US   1500U
#define SPARK_MAX_US       2000U
#define SPARK_MOTOR_COUNT  3U
/* USER CODE END PD */
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* USER CODE BEGIN PV */
static const uint32_t spark_channels[SPARK_MOTOR_COUNT] = {
  TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_4
};
/* USER CODE END PV */
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef Spark_SetPulseUs(uint32_t motor, uint32_t pulse_us);
HAL_StatusTypeDef Spark_SetCommand(uint32_t motor, int32_t permille);
void Spark_NeutralAll(void);
static void Spark_Start(void);
/* USER CODE END PFP */
/* USER CODE BEGIN 0 */
/* Motor IDs are 1..3. Calls are intended from one control task.
 * Compare preload makes changes take effect at the next 5 ms frame.
 * These are motor-output commands, NOT joint-angle or regulated speed targets.
 */
HAL_StatusTypeDef Spark_SetPulseUs(uint32_t motor, uint32_t pulse_us)
{
  if ((motor < 1U) || (motor > SPARK_MOTOR_COUNT)) return HAL_ERROR;
  if (pulse_us < SPARK_MIN_US) pulse_us = SPARK_MIN_US;
  if (pulse_us > SPARK_MAX_US) pulse_us = SPARK_MAX_US;
  __HAL_TIM_SET_COMPARE(&htim2, spark_channels[motor - 1U], pulse_us);
  return HAL_OK;
}

/* -1000 = full reverse, 0 = neutral, +1000 = full forward. */
HAL_StatusTypeDef Spark_SetCommand(uint32_t motor, int32_t permille)
{
  if (permille < -1000) permille = -1000;
  if (permille > 1000) permille = 1000;
  return Spark_SetPulseUs(motor, (uint32_t)(1500 + permille / 2));
}

void Spark_NeutralAll(void)
{
  for (uint32_t motor = 1U; motor <= SPARK_MOTOR_COUNT; ++motor)
    (void)Spark_SetPulseUs(motor, SPARK_NEUTRAL_US);
}

static void Spark_Start(void)
{
  /* Detect an unregenerated or incorrectly configured tim.c. */
  if ((htim2.Instance != TIM2) || (htim2.Init.Prescaler != 63U) ||
      (htim2.Init.Period != 4999U)) Error_Handler();

  for (uint32_t i = 0; i < SPARK_MOTOR_COUNT; ++i)
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, spark_channels[i]);
  Spark_NeutralAll();
  /* Load the prescaler and neutral compare registers before enabling output. */
  if (HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  for (uint32_t i = 0; i < SPARK_MOTOR_COUNT; ++i)
    if (HAL_TIM_PWM_Start(&htim2, spark_channels[i]) != HAL_OK)
      Error_Handler();
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  MPU_Config();
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Spark_Start();
  /* USER CODE END 2 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    /* Deliberately stays neutral; no automatic motor movement.
     * Add command handling or joint control here. For example, once ready:
     * Spark_SetCommand(1U, 100);  // +10% command, 1550 us
     * Spark_SetCommand(2U, -100); // -10% command, 1450 us
     * Spark_SetCommand(3U, 0);    // neutral, 1500 us
     *
     * Hardware keeps transmitting the last command even if this loop stalls.
     * Add a watchdog/command timeout before implementing unattended motion.
     */
    HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
      RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    Error_Handler();
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
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
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  Error_Handler();
  /* USER CODE END 6 */
}
#endif
=======
/* USER CODE BEGIN Header */
/**
 * @file main.c
 * @brief Three SPARK Flex servo-PWM outputs for NEO Vortex motors.
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
#define SERVO_COUNT        3U

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

static uint8_t ros_rx_byte;
static char ros_rx_buffer[ROS_RX_BUFFER_SIZE];
static volatile uint16_t ros_rx_len = 0;
static volatile uint8_t ros_msg_ready = 0;
static uint32_t ros_last_cmd_ms = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
        ROS_ProcessCommand(line);

    // Stop commanded motion if communication times out
    if ((HAL_GetTick() - ros_last_cmd_ms) > ROS_CMD_TIMEOUT_MS)
        Servo_NeutralAll();
}

/* Motor IDs are 1..3. Calls are intended from one control task.
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
      (htim2.Init.Period != 4999U)) Error_Handler();

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Servo_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Deliberately stays neutral; no automatic motor movement.
     * Add command handling or joint control here. For example, once ready:
     * Spark_SetCommand(1U, 100);  // +10% command, 1550 us
     * Spark_SetCommand(2U, -100); // -10% command, 1450 us
     * Spark_SetCommand(3U, 0);    // neutral, 1500 us
     *
     * Hardware keeps transmitting the last command even if this loop stalls.
     * Add a watchdog/command timeout before implementing unattended motion.
     */
    HAL_Delay(5);
//    for (uint32_t i = 0; i++; i < 3) {
//    	uint32_t motorP = (uint32_t) PID_Control(encoder_targets[i], encoder_value[i], elapsed_time, i);
//    	Spark_SetCommand(i, motorP * 2); //2 times since motorP returns [-500,500]
//    }

//    start_time = HAL_GetTick();

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
>>>>>>> Stashed changes
