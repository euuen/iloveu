/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "font.h"
#include "oled.h"
#include "adc.h"
#include "dac.h"
#include "dac_audio_data.h"
#include "tim.h"
#include <stdint.h>
#include <stdio.h>
#include "pid.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for adc */
osThreadId_t adcHandle;
const osThreadAttr_t adc_attributes = {
  .name = "adc",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dac */
osThreadId_t dacHandle;
const osThreadAttr_t dac_attributes = {
  .name = "dac",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mpu */
osThreadId_t mpuHandle;
const osThreadAttr_t mpu_attributes = {
  .name = "mpu",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for pid */
osThreadId_t pidHandle;
const osThreadAttr_t pid_attributes = {
  .name = "pid",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for can */
osThreadId_t canHandle;
const osThreadAttr_t can_attributes = {
  .name = "can",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void adcFunc(void *argument);
void dacFunc(void *argument);
void mpuTask(void *argument);
void pidFunc(void *argument);
void canFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of adc */
  adcHandle = osThreadNew(adcFunc, NULL, &adc_attributes);

  /* creation of dac */
  dacHandle = osThreadNew(dacFunc, NULL, &dac_attributes);

  /* creation of mpu */
  mpuHandle = osThreadNew(mpuTask, NULL, &mpu_attributes);

  /* creation of pid */
  pidHandle = osThreadNew(pidFunc, NULL, &pid_attributes);

  /* creation of can */
  canHandle = osThreadNew(canFunc, NULL, &can_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_adcFunc */
/**
  * @brief  Function implementing the adc thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_adcFunc */
void adcFunc(void *argument)
{
  /* USER CODE BEGIN adcFunc */
  /* Infinite loop */
  for(;;)
  {
    // HAL_ADC_Start(&hadc1); // 开始读取ADC
    // HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // 等待ADC读取完毕
    // int value = HAL_ADC_GetValue(&hadc1); // 获取ADC值
    // float voltage = (value / 4095.0f) * 3.3f; // 转换为实际电压
    // printf("vol: %f\n", voltage);
    osDelay(20000);
  }
  /* USER CODE END adcFunc */
}

/* USER CODE BEGIN Header_dacFunc */
/**
* @brief Function implementing the dac thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dacFunc */
void dacFunc(void *argument)
{
  /* USER CODE BEGIN dacFunc */
  /* Infinite loop */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_Base_Start(&htim6);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac_audio_data_data, DAC_AUDIO_DATA_NUM_SAMPLES, DAC_ALIGN_12B_R); // dma搬运数据

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dacFunc */
}

/* USER CODE BEGIN Header_mpuTask */
/**
* @brief Function implementing the mpu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mpuTask */
void mpuTask(void *argument)
{
  /* USER CODE BEGIN mpuTask */
  /* Infinite loop */
  osDelay(20);
  // OLED_Init();
  // MPU_Init();
  for(;;)
  {
    // MPU_Update_Angle();
    // char stringbuf[32];
    // OLED_NewFrame();

    // sprintf(stringbuf, "yaw   : %.2f", mpu6050_yaw);
    // OLED_PrintASCIIString(0, 0, stringbuf, &afont12x6, OLED_COLOR_NORMAL);

    // sprintf(stringbuf, "pitch : %.2f", mpu6050_pitch);
    // OLED_PrintASCIIString(0, 12, stringbuf, &afont12x6, OLED_COLOR_NORMAL);

    // sprintf(stringbuf, "roll  : %.2f", mpu6050_roll);
    // OLED_PrintASCIIString(0, 12 * 2, stringbuf, &afont12x6, OLED_COLOR_NORMAL);

    // OLED_ShowFrame();
    osDelay(10);
  }
  /* USER CODE END mpuTask */
}

/* USER CODE BEGIN Header_pidFunc */
/**
* @brief Function implementing the pid thread.
* @param argument: Not used
* @retval None
*/
typedef enum {
    ANGLE_MOTOR,
    SPEED_MOTOR
  } MotorMode;
MotorMode motorMode = SPEED_MOTOR;
PID speedpid;
PID angelpid;
uint32_t lastEncoder;
uint32_t targetSpeed;
uint32_t targetAngleEncoder;
/* USER CODE END Header_pidFunc */
void pidFunc(void *argument)
{
  /* USER CODE BEGIN pidFunc */
  /* Infinite loop */
  // HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
  
  // PID_InitSpeed(&speedpid);
  // PID_InitAngle(&angelpid);

  // lastEncoder = __HAL_TIM_GET_COUNTER(&htim4);
  // targetSpeed = 5;
  // OLED_Init();
  for(;;)
  {
    // switch (motorMode) {
    //   case ANGLE_MOTOR:{
    //     uint32_t currEncoder = __HAL_TIM_GET_COUNTER(&htim4);
    //     int pwmOut = PID_Output(&angelpid, currEncoder, targetAngleEncoder, 0.01);
    //     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmOut);
    //     break;
    //   }
    //   case SPEED_MOTOR: {
    //     uint32_t currEncoder = __HAL_TIM_GET_COUNTER(&htim4);
    //     __HAL_TIM_SET_COUNTER(&htim4, 0);
    //     int pwmOut = PID_Output(&speedpid, currEncoder - lastEncoder, targetSpeed, 0.01);
    //     lastEncoder = currEncoder;
    //     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmOut);
    //     break;
    //   }
    //   default:
    //     printf("err\n");
    //     break;
    // }


    // OLED_NewFrame();

    // char stringbuf[32];
    // sprintf(stringbuf, "s:%d", currEncoder);
    // OLED_PrintASCIIString(0, 0, stringbuf, &afont12x6, OLED_COLOR_NORMAL);

    // sprintf(stringbuf, "p:%d", pwmOut);
    // OLED_PrintASCIIString(0, 12, stringbuf, &afont12x6, OLED_COLOR_NORMAL);

    // OLED_ShowFrame();

    osDelay(10);
  }
  /* USER CODE END pidFunc */
}

/* USER CODE BEGIN Header_canFunc */
/**
* @brief Function implementing the can thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canFunc */
void canFunc(void *argument)
{
  /* USER CODE BEGIN canFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END canFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

