/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <bluetooth.h>
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

#include "bmx160.h"
#include "Madgwick_BMX160.h"
#include <bluetooth.h>
#include "Kalman_Filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define A_PACK_INCLUDE_FRAME_NUMBER  10  //一个数据包中，包含5个数据协议帧
#define AGREEMENT_ARRAY_LENGTH 20       //一个数据协议帧中，包含40个byte
#define RECEIVE_ARRAY_LENGTH 5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char tempChar[4];
uint32_t  computeCount = 0;

uint8_t g_bleSendArray[AGREEMENT_ARRAY_LENGTH*A_PACK_INCLUDE_FRAME_NUMBER];
uint8_t g_bleReceiveBuffer[RECEIVE_ARRAY_LENGTH];
uint8_t g_bleReceiveArray[RECEIVE_ARRAY_LENGTH];
uint8_t g_bleReceiveFlag = 0;
_Bool is6Axis = pdFALSE;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint32_t tempTime = 0;

	uint8_t isSetErrorRTC = 0;
	
    if (huart->Instance == USART1) {
			for(uint8_t i=0;i<RECEIVE_ARRAY_LENGTH;i++){
				memcpy(&g_bleReceiveArray[0], &g_bleReceiveArray[1],RECEIVE_ARRAY_LENGTH-1);
				memcpy(&g_bleReceiveArray[RECEIVE_ARRAY_LENGTH-1], &g_bleReceiveBuffer[i],sizeof(uint8_t)*1);
				if((g_bleReceiveArray[0]==0xBB) && (g_bleReceiveArray[RECEIVE_ARRAY_LENGTH-1]==0xEE))
				{
					switch(g_bleReceiveArray[1])
					{
						case 1:
						{
							g_bleReceiveFlag=1;
						break;
						}
						case 6:
						{
							is6Axis = pdTRUE;
						break;
						}
						case 9:
						{
							is6Axis = pdFALSE;
						break;
						}
					}
					memset(g_bleReceiveBuffer,0,RECEIVE_ARRAY_LENGTH); 
					memset(g_bleReceiveArray,0,RECEIVE_ARRAY_LENGTH); 
					break;
				}
			}
			
		}
	}	

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	static TickType_t  xLastWakeTime;
	const TickType_t  xFrequency = pdMS_TO_TICKS(2);
	// 使用当前时间初始化变量xLastWakeTime ,注意这和vTaskDelay()函数不同
	xLastWakeTime = xTaskGetTickCount();
	osDelay(100);
	BMX160Init(2,2);
	
	osDelay(20);
	WLL_BLE240_Init();	
	HAL_UART_Receive_DMA(&huart1,g_bleReceiveBuffer,RECEIVE_ARRAY_LENGTH);
	osDelay(20);
  while(1)
  {
		if(g_bleReceiveFlag==1)
		{
//				HAL_Delay(1000);
//				HAL_Delay(1000);
//				HAL_Delay(1000);
//				HAL_Delay(1000);
//				HAL_Delay(1000);
//			Calibration_Mag();
			Calibration_Mag();
			g_bleReceiveFlag = 0;
			memset(g_bleSendArray,0,sizeof(g_bleSendArray)); 
			g_bleSendArray[0] = 0xBB;
			g_bleSendArray[1] = 0xBB;
			g_bleSendArray[2] = 1;  //0 是软磁数据，1是硬磁数据，2是欧拉角数据

			//时间戳
			memcpy(&g_bleSendArray[3],&computeCount,sizeof(computeCount));
			//硬磁X
			memcpy(&g_bleSendArray[7],&mag_offset.x,sizeof(mag_offset.x));
			//硬磁Y
			memcpy(&g_bleSendArray[11],&mag_offset.y,sizeof(mag_offset.y));
			//硬磁Z
			memcpy(&g_bleSendArray[15],&mag_offset.z,sizeof(mag_offset.z));
			g_bleSendArray[19] = 0xEE;
			
			g_bleSendArray[20] = 0xBB;
			g_bleSendArray[21] = 0xBB;
			g_bleSendArray[22] = 0;  //0 是软磁数据，1是硬磁数据，2是欧拉角数据

			//时间戳
			memcpy(&g_bleSendArray[23],&computeCount,sizeof(computeCount));
			//软磁X
			memcpy(&g_bleSendArray[27],&mag_scale.x,sizeof(mag_scale.x));
			//软磁Y
			memcpy(&g_bleSendArray[31],&mag_scale.y,sizeof(mag_scale.y));
			//软磁Z
			memcpy(&g_bleSendArray[35],&mag_scale.z,sizeof(mag_scale.z));
			g_bleSendArray[39] = 0xEE;
			
			BLE240_WAKEUP();	//WAKE拉高省电，拉低通信。不影响蓝牙连接
			;;;;;;;;;;
			HAL_UART_Transmit_DMA(&huart1, g_bleSendArray, sizeof(g_bleSendArray));
			HAL_Delay(20);
			memset(g_bleSendArray,0,sizeof(g_bleSendArray)); 
			computeCount=0;
			g_bleReceiveFlag = 0;
		}
		

		computeEuler(2,2);
		
		if(is6Axis){
			_mag_filter_value.x = 0.0f;
			_mag_filter_value.y = 0.0f;
			_mag_filter_value.z = 0.0f;
		}
		imu_solution_9axis(_gyro_filter_value.x, _gyro_filter_value.y, _gyro_filter_value.z, _accel_filter_value.x, _accel_filter_value.y, _accel_filter_value.z, _mag_filter_value.x, _mag_filter_value.y, _mag_filter_value.z, 0.002);
	
		euler_angle.pitch = kalmanFilter(&KFP_pitch, euler_angle.pitch);
		euler_angle.yaw = kalmanFilter(&KFP_yaw, euler_angle.yaw);
		euler_angle.roll = kalmanFilter(&KFP_roll, euler_angle.roll);
		
//		memcpy(&tempChar,&valueInt,sizeof(valueInt));
		
		g_bleSendArray[(computeCount%A_PACK_INCLUDE_FRAME_NUMBER)*AGREEMENT_ARRAY_LENGTH + 0] = 0xBB;
		g_bleSendArray[(computeCount%A_PACK_INCLUDE_FRAME_NUMBER)*AGREEMENT_ARRAY_LENGTH + 1] = 0xBB;
		g_bleSendArray[(computeCount%A_PACK_INCLUDE_FRAME_NUMBER)*AGREEMENT_ARRAY_LENGTH + 2] = 2;  //0 是软磁数据，1是硬磁数据，2是欧拉角数据

		//时间戳
		memcpy(&g_bleSendArray[computeCount%A_PACK_INCLUDE_FRAME_NUMBER*AGREEMENT_ARRAY_LENGTH + 3],&computeCount,4);
		//pitch
		memcpy(&g_bleSendArray[computeCount%A_PACK_INCLUDE_FRAME_NUMBER*AGREEMENT_ARRAY_LENGTH + 7],&euler_angle.pitch,4);
		//yaw
		memcpy(&g_bleSendArray[computeCount%A_PACK_INCLUDE_FRAME_NUMBER*AGREEMENT_ARRAY_LENGTH + 11],&euler_angle.yaw,4);
		//roll
		memcpy(&g_bleSendArray[computeCount%A_PACK_INCLUDE_FRAME_NUMBER*AGREEMENT_ARRAY_LENGTH + 15],&euler_angle.roll,4);
		g_bleSendArray[(computeCount%A_PACK_INCLUDE_FRAME_NUMBER)*AGREEMENT_ARRAY_LENGTH + 19] = 0xEE;
		
		computeCount++;
		
		if(computeCount%80==0){
			LED_GREEN_Toggle();
		}
		
		if(computeCount%A_PACK_INCLUDE_FRAME_NUMBER == 0)
		{
			BLE240_WAKEUP();	//WAKE拉高省电，拉低通信。不影响蓝牙连接
			for(int i=0;i<10;i++){;}
			HAL_UART_Transmit_DMA(&huart1, g_bleSendArray, sizeof(g_bleSendArray));
		}

    vTaskDelayUntil( &xLastWakeTime,xFrequency );   //采用绝对延时，确保线程执行周期稳定 Luzh
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
