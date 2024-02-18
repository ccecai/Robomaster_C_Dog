/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId imuTaskHandle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t standup_flag = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
fp32 gyro[3];
fp32 accel[3];
fp32 temp;

/* USER CODE END Variables */
osThreadId StartDebugTaskHandle;
osThreadId RemoteControlTaHandle;
osThreadId GIM_OutputHandle;
osThreadId GIM6010InitHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebug(void const * argument);
void RemoteControl(void const * argument);
void GIM_OutputTask(void const * argument);
void GIM6010InitTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
    osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StartDebugTask */
  osThreadDef(StartDebugTask, StartDebug, osPriorityNormal, 0, 128);
  StartDebugTaskHandle = osThreadCreate(osThread(StartDebugTask), NULL);

  /* definition and creation of RemoteControlTa */
  osThreadDef(RemoteControlTa, RemoteControl, osPriorityBelowNormal, 0, 512);
  RemoteControlTaHandle = osThreadCreate(osThread(RemoteControlTa), NULL);

  /* definition and creation of GIM_Output */
  osThreadDef(GIM_Output, GIM_OutputTask, osPriorityNormal, 0, 512);
  GIM_OutputHandle = osThreadCreate(osThread(GIM_Output), NULL);

  /* definition and creation of GIM6010Init */
  osThreadDef(GIM6010Init, GIM6010InitTask, osPriorityNormal, 0, 256);
  GIM6010InitHandle = osThreadCreate(osThread(GIM6010Init), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    vTaskResume(StartDebugTaskHandle);
    vTaskSuspend(GIM_OutputHandle);
    vTaskSuspend(RemoteControlTaHandle);
    vTaskSuspend(GIM6010InitHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDebug */
/**
  * @brief  Function implementing the StartDebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebug */
void StartDebug(void const * argument)
{
  /* USER CODE BEGIN StartDebug */
    Mymain_Init();

    vTaskResume(GIM6010InitHandle);
  /* Infinite loop */
  for(;;)
  {
      LED_Flash();

      osDelay(500);
  }
  /* USER CODE END StartDebug */
}

/* USER CODE BEGIN Header_RemoteControl */
/**
* @brief Function implementing the RemoteControlTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RemoteControl */
void RemoteControl(void const * argument)
{
  /* USER CODE BEGIN RemoteControl */
    remote_control_init();

    RC_ctrl_t *local_rc_ctrl;
    local_rc_ctrl = get_remote_control_point();
  /* Infinite loop */
  for(;;)
  {
      /*************
       *    遥控器只有左方拨档开关有用，最上方一档为蹲伏即上电状态，中间档为直立
       *    下方档为运动开关，右方摇杆为控制运动
       *
       ****************/
      Posture_Controller(local_rc_ctrl);

      osDelay(5);
  }
  /* USER CODE END RemoteControl */
}

/* USER CODE BEGIN Header_GIM_OutputTask */
/**
* @brief Function implementing the GIM_Output thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GIM_OutputTask */
void GIM_OutputTask(void const * argument)
{
  /* USER CODE BEGIN GIM_OutputTask */
  /* Infinite loop */
  for(;;)
  {
      Leg_Output();

    osDelay(5);
  }
  /* USER CODE END GIM_OutputTask */
}

/* USER CODE BEGIN Header_GIM6010InitTask */
/**
* @brief Function implementing the GIM6010Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GIM6010InitTask */
void GIM6010InitTask(void const * argument)
{
  /* USER CODE BEGIN GIM6010InitTask */
    Motor_Init();

    Read_beginPos(); //读取电机上电时候的初始位置
    Eight_PID_Init();
    ChangeGainOfPID(5.0f,0.0f,0.0f,0.0f);//输出化PID

    vTaskResume(RemoteControlTaHandle);
    vTaskResume(GIM_OutputHandle);
    vTaskSuspend(NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GIM6010InitTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
