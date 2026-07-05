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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "MPU6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR (0x68 << 1)
#define WHO_AM_I (0x75)
#define PWR_MGMT_1 (0x6B)
#define SMPLRT_DIV (0x19)
#define GYRO_CONFIG (0x1B)
#define ACCEL_CONFIG (0x1C)
#define ACCEL_XOUT_H (0x3B)
#define GYRO_XOUT_H (0x43)


#define DSHOT_LEN 18

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim4_up;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t RxData[20];
uint8_t temp[2];
int indx = 0;
uint8_t FinalData[20];
int countLoop = 0;

float kp;
float kd;
float ki;

float error = 0.0f;
float setpoint = 0.0f;
float integral = 0.0f;
float error_roc = 0.0f;
float tuned_value = 0.0f;
float prev_error = 0.0f;

int FilterInitialized = 0;


uint16_t telemetry = 0;
//uint16_t dshot_buffer[18];

uint16_t dshot_buffer[DSHOT_LEN] = {0};


uint32_t pulse;

volatile uint8_t sampleFlag = 0;
volatile uint8_t controlFlag = 0;


volatile uint8_t dshot_busy = 0;
volatile uint32_t dshot_start_count = 0;
volatile uint32_t dshot_done_count = 0;
volatile uint32_t dshot_busy_skip_count = 0;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    memcpy(RxData+indx, temp, 1);
    if (++indx >= 20) indx = 0;

    HAL_UART_Receive_IT(&huart2, temp, 1);
}


int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}



void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    }
    else if (htim->Instance == TIM3)
    {
        sampleFlag = 1;
    }
    else if (htim -> Instance == TIM5)
    {
        controlFlag = 1;
    }


}

void PID(void)
{
   error = setpoint - angleX;
   integral += error * dt;
   integral = fmaxf(-200, fminf(200, integral));
   error_roc = (error - prev_error) / dt;
   tuned_value = (error * kp) + (integral * ki) + (error_roc * kd);
   tuned_value = fmaxf(1000, fminf(2000, tuned_value));
   prev_error = error;
}

void Build_Dshot_Packet(uint16_t throttle_cmd)
{
    // Build 12-bit value: 11-bit throttle + telemetry bit
    uint16_t value = (throttle_cmd << 1) | telemetry;

    // DShot checksum
    uint16_t csum = 0;
    uint16_t csum_data = value;

    for (int i = 0; i < 3; i++)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    csum &= 0x0F;

    // Final 16-bit packet
    uint16_t packet = (value << 4) | csum;

//    printf("throttle=%u packet=0x%04X\r\n", throttle, packet);

    // Convert bits to PWM compare values
    for (int i = 0; i < 16; i++)
    {
        dshot_buffer[i] = (packet & (1 << (15 - i))) ? 420 : 210;
    }

    // Long low period after packet
    dshot_buffer[16] = 0;
    dshot_buffer[17] = 0;
//    printf("%u %u %u %u %u %u\r\n",
//           dshot_buffer[0], dshot_buffer[1], dshot_buffer[2],
//           dshot_buffer[3], dshot_buffer[4], dshot_buffer[5]);
}




void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_tim4_up)
    {
        TIM4 -> DIER &= ~TIM_DIER_UDE;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        dshot_done_count++;
        dshot_busy = 0;
    }
}

void DShot_DMA_Complete(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_tim4_up)
    {
        TIM4->DIER &= ~TIM_DIER_UDE;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

        dshot_done_count++;
        dshot_busy = 0;
    }
}

void Send_DShot(uint16_t throttle_cmd)
{

    if (dshot_busy)
    {
        dshot_busy_skip_count++;
        return;
    }

    dshot_busy = 1;
    dshot_start_count++;

    telemetry = 0;
    Build_Dshot_Packet(throttle_cmd);

    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

    HAL_StatusTypeDef status = HAL_DMA_Start_IT(&hdma_tim4_up,
                     (uint32_t)&dshot_buffer[0],
                     (uint32_t)&TIM4->CCR1,
                     DSHOT_LEN);

    if (status != HAL_OK)
    {
        dshot_busy = 0;
        return;
    }

    TIM4->DIER |= TIM_DIER_UDE;
}
//

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    printf("ACTIVE MAIN 12345\r\n");



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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  printf("BOOT\r\n");
  printf("RESET FLAGS: 0x%081X\r\n", RCC->CSR);
  __HAL_RCC_CLEAR_RESET_FLAGS();


  printf("ABOUT TO INIT MPU FROM REAL MAIN\r\n");

  HAL_Delay(300);

  for (int attempt = 1; attempt <=5; attempt ++)
  {

      if (MPU6050_Init())
      {
          printf("MPU Init Success on attempt %d\r\n", attempt);
          printf("Calibrating MPU\r\n");
          Calibrate_MPU6050();
          printf("Calibration done\r\n");
          break;
      }

      printf("MPU Init attempt failed attempt %d\r\n", attempt);
      HAL_Delay(200);
  }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);

    angleX = 0.0f;
    angleY = 0.0f;
    angleZ = 0.0f;

    int i = 0;

//    throttle = 1000;

//    hdma_tim4_up.XferCpltCallback = DShot_DMA_Complete;

//    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

//    DShot_Start_Frame();

//    uint32_t lastPrint = 0;

    hdma_tim4_up.XferCpltCallback = DShot_DMA_Complete;
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    uint32_t last_dshot_time = 0;
    uint32_t start = HAL_GetTick();

    uint32_t lastRampTime = HAL_GetTick();
    uint16_t throttle_cmd = 0;


    uint16_t min_throttle = 100;
    uint16_t max_throttle = 650;
    uint16_t base_throttle = 380;
    float max_angle = 90.0f;
    float target_angle = 5.0f;

    float error = 0;

//    float max_correction = 60.0f;

    float filteredThrottle = 0;

    float kp_up = 10.5f;
    float kd_up = 0.55f;

    float kp_down = 14.0f;
    float kd_down = 0.15f;

    float ki = 0.5f;

    float prev_error = 0.0f;
    const float dt = 0.01f;

    float integral = 0.0f;
    float integral_limit = 100.0f;

    float derivative = 0.0f;
    float correction = 0.0f;

    float P = 0.0f;
    float D = 0.0f;
    float I = 0.0f;


    HAL_TIM_Base_Start_IT(&htim5);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//      Send_DShot(300);
//      HAL_Delay(2);
////      printf("start=%lu done=%lu skip=%lu busy=%u\r\n",
////             dshot_start_count,
////             dshot_done_count,
////             dshot_busy_skip_count,
////             dshot_busy);


    if (controlFlag)
    {

        controlFlag = 0;

        MPU6050_Read();

           Gx = Gx - GxOffset;
           Gy = Gy - GyOffset;
           Gz = Gz - GzOffset;



           if (!FilterInitialized)
           {
               GxFiltered = Gx;
               GyFiltered = Gy;
               GzFiltered = Gz;
               FilterInitialized = 1;
           } else
           {
               GxFiltered = (0.9 * GxFiltered) + (Gx * 0.1);
               GyFiltered = (0.9 * GyFiltered) + (Gy * 0.1);
               GzFiltered = (0.9 * GzFiltered) + (Gz * 0.1);
           }




           float accelAngleX = atan2(Ay, Az) * 180.0f / M_PI;
           float accelAngleY = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0f / M_PI;

           angleX = 0.98f * (angleX + GxFiltered * dt) + 0.02f * accelAngleX;
           angleY = 0.98f * (angleY + GyFiltered * dt) + 0.02f * accelAngleY;

           pulse = (angleX * 999) / 180;
           __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);









        if (HAL_GetTick() - start < 3000)
                 {
                     throttle_cmd = 0;
                 }
                 else
                 {

                     error = target_angle - angleX;

                     if (error < 0)
                     {
                         P = kp_down * error;
                         D = -kd_down * GxFiltered;
                     }
                     else
                     {
                         P = kp_up * error;
                         D = -kd_up * GxFiltered;
                     }

                     if (fabs(error) < 10.0f)
                     {

                         integral += error * dt;
                     }
                     else
                     {
                         integral = 0.0f;
                     }


                     if (integral > integral_limit) integral = integral_limit;
                     if (integral < -integral_limit) integral = -integral_limit;




                     I = ki * integral;

                     correction = P + D + I;

                     throttle_cmd = base_throttle + correction;

//                     prev_error = error;





                 }

//        filteredThrottle = 0.9f * filteredThrottle + 0.1f * throttle_cmd;
//        throttle_cmd = filteredThrottle;

        if (throttle_cmd > max_throttle) throttle_cmd = max_throttle;
        if (throttle_cmd < min_throttle) throttle_cmd = min_throttle;


        Send_DShot(throttle_cmd);

    }

    if (i > 25)
    {
        printf("A:%7.2f E:%7.2f G:%7.2f I:%7.2f C:%8.2f  T:%3u\r\n",
                angleX,
                error,
                GxFiltered,
                I,
                correction,
                throttle_cmd);
    i = 0;
    }

    i++;






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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 559;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
