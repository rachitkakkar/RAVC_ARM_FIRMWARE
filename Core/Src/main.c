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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPARK_MIN_US       1000U
#define SPARK_NEUTRAL_US   1500U
#define SPARK_MAX_US       2000U
#define SPARK_MOTOR_COUNT  3U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const uint32_t spark_channels[SPARK_MOTOR_COUNT] = {
  TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_4
};
static const uint32_t encoder_channels[SPARK_MOTOR_COUNT] = {
  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
};
/* USER CODE END PV */

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float previous_error;

    float output_min;
    float output_max;

} PID_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef Spark_SetPulseUs(uint32_t motor, uint32_t pulse_us);
HAL_StatusTypeDef Spark_SetCommand(uint32_t motor, int32_t permille);
void Spark_NeutralAll(void);
static void Spark_Start(void);
/* USER CODE END PFP */

//TODO: tune the PID!!!!!
PID_t motor_pid[3] = {
		{
				.Kp = 1.0f,
				.Ki = 0.0f,
				.Kd = 0.0f,
				.integral = 0.0f,
				.previous_error = 0.0f,
				.output_min = -500.0f,
				.output_max = 500.0f
		},
		{
				.Kp = 1.0f,
				.Ki = 0.0f,
				.Kd = 0.0f,
				.integral = 0.0f,
				.previous_error = 0.0f,
				.output_min = -500.0f,
				.output_max = 500.0f
		},
		{
				.Kp = 1.0f,
				.Ki = 0.0f,
				.Kd = 0.0f,
				.integral = 0.0f,
				.previous_error = 0.0f,
				.output_min = -500.0f,
				.output_max = 500.0f
		}
};

static volatile uint32_t encoder_rising[3];
static volatile uint32_t encoder_value[3];
static volatile uint8_t encoder_waiting_falling[3];

static uint32_t encoder_targets[3] = {500, 500, 500};

/* Private user code ---------------------------------------------------------*/
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
  /* USER CODE BEGIN 2 */
  Spark_Start();

  //TODO: but this into function
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  uint32_t start_time = HAL_GetTick();
  uint32_t elapsed_time = 0;



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  elapsed_time = HAL_GetTick() - start_time;
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
    for (uint32_t i = 0; i++; i < 3) {
    	uint32_t motorP = (uint32_t) PID_Control(encoder_targets[i], encoder_value[i], elapsed_time, i);
    	Spark_SetCommand(i, motorP * 2); //2 times since motorP returns [-500,500]
    }

    start_time = HAL_GetTick();

  }
  /* USER CODE END 3 */
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error;
    float derivative;
    float output;


    error = Encoder_Error(setpoint, measurement);

    /* Integral */
    pid->integral += error * dt;

    /* Anti-windup */
    if (pid->integral > pid->output_max)
        pid->integral = pid->output_max;

    if (pid->integral < pid->output_min)
        pid->integral = pid->output_min;

    /* Derivative */
    derivative = (error - pid->previous_error) / dt;

    /* PID */
    output =
        (pid->Kp * error) +
        (pid->Ki * pid->integral) +
        (pid->Kd * derivative);

    /* Limit output */
    if (output > pid->output_max)
        output = pid->output_max;

    if (output < pid->output_min)
        output = pid->output_min;

    pid->previous_error = error;

    return output;
}

float PID_Control(uint16_t target_position,
                  uint16_t encoder_position,
                  float dt,
				  uint32_t motor)
{
    float correction;

    correction = PID_Update(&motor_pid[motor],
                            (float)target_position,
                            (float)encoder_position,
                            dt);

    /* Center around neutral */
    float pwm_us = 1500.0f + correction;

    /* Final safety limits */
    if (pwm_us > 2000.0f)
        pwm_us = 2000.0f;

    if (pwm_us < 1000.0f)
        pwm_us = 1000.0f;

    return pwm_us;
}

float Encoder_Error(uint16_t target, uint16_t actual)
{
    float error = (float)target - (float)actual;

    if (error > 512.0f)
        error -= 1024.0f;

    if (error < -512.0f)
        error += 1024.0f;

    return error;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1)
        return;

    int motor = -1;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        motor = 0;
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        motor = 1;
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        motor = 2;

    if (motor < 0)
        return;

    uint32_t capture =
        HAL_TIM_ReadCapturedValue(
            htim,
            encoder_channels[motor]
        );

    if (!encoder_waiting_falling[motor])
    {
        encoder_rising[motor] = capture;
        encoder_waiting_falling[motor] = 1;

        __HAL_TIM_SET_CAPTUREPOLARITY(
            htim,
            encoder_channels[motor],
            TIM_INPUTCHANNELPOLARITY_FALLING
        );
    }
    else
    {
        if (capture >= encoder_rising[motor])
        {
            encoder_value[motor] =
                capture - encoder_rising[motor];
        }
        else
        {
            encoder_value[motor] =
                (0xFFFF - encoder_rising[motor])
                + capture + 1;
        }

        encoder_waiting_falling[motor] = 0;

        __HAL_TIM_SET_CAPTUREPOLARITY(
            htim,
            encoder_channels[motor],
            TIM_INPUTCHANNELPOLARITY_RISING
        );
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
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
