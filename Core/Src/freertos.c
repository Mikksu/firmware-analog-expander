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
/* USER CODE BEGIN Includes */
#include "string.h"
#include "tftlcd.h"
#include "usbd_cdc_if.h"
#include "mb.h"
#include "fifo.h"
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

/**
 * @brief     The handle of the FIFO to save the data received from VCP.
 */
fifo_t        vcpRxFifo, *hVcpRxFifo;
fifo_t        vcpTxFifo, *hVcpTxFifo;

osThreadId    vcpMbDataProcTaskHandle;
osThreadId    mbTaskHandle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartVcpMbDataProcTask(void const * argument);
void StartMbTask(void const * argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  InitRTOSObjects();

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	
  // turn off the 3-color-led.
	HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	
  // initialize the TFT LCD
	LCD_Init();
	LCD_Clear(RED);

  // create the vcp received data process task.
	osThreadDef(vcpDataProcTask, StartVcpMbDataProcTask, osPriorityNormal, 0, 512);
  vcpMbDataProcTaskHandle = osThreadCreate(osThread(vcpDataProcTask), NULL);
	
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(LEDR_GPIO_Port, LEDR_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief The task to connect the VCP RX data and the modbus protocol stack.
 * 
 * @param argument
 */
void StartVcpMbDataProcTask(void const * argument)
{
  CdcRxBuff_TypeDef tmpBuff;
  
  // create the MODBUS protocol stack task.
  eMBInit(MB_RTU, 0X01, 1, 115200, MB_PAR_NONE);
	eMBEnable();
  osThreadDef (modbusPollingTask, StartMbTask, osPriorityNormal, 0, 256);
  mbTaskHandle = osThreadCreate(osThread(modbusPollingTask), NULL);

  // initialize the FIFO to save data received from VCP.
  hVcpRxFifo = &vcpRxFifo;
  fifo_init(hVcpRxFifo);
  hVcpTxFifo = &vcpTxFifo;
  fifo_init(hVcpTxFifo);


  // start to poll the VCP RX packages.
  for(;;)
  {
    osEvent evt = osMessageGet(cdcRxMsgHandle, osWaitForever);

    if(evt.status == osEventMessage && evt.value.p != NULL)
    {

      HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);

      // get the address of the pool
      uint32_t pPool = (uint32_t)evt.value.p;

      // copy the data to the temporary memory.
      memcpy((uint8_t*)&tmpBuff, (uint8_t*)pPool, sizeof(CdcRxBuff_TypeDef));
      
      // free the pool.
      osPoolFree(cdcRxPoolhandle, (void*)pPool);

      // push the data to the fifo.
      for(int i = 0; i < tmpBuff.Length; i++)
        fifo_put(hVcpRxFifo, tmpBuff.bufCdcRx[i]);


      // echo for TEST purpose
      extern USBD_HandleTypeDef hUsbDeviceFS;
      USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
      CDC_Transmit_FS(tmpBuff.bufCdcRx, tmpBuff.Length);
      while(hcdc->TxState != 0);
    }
  }
}

/**
 * @brief The task to run the modbus protocol stack.
 * 
 * @param argument
 */
void StartMbTask(void const * argument)
{
	for(;;)
	{
		eMBPoll();
	}

}

/* USER CODE END Application */
