/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "drv_lsm6dsl.h"
#include "adc.h"
#include "tim.h"
#include "lsm6dsl.h"
#include "drv_mouse.h"
#include "imucaculator.h"

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

#define ADC_CONVERTED_DATA_BUFFER_SIZE 5
uint32_t Timestamp = 0;
uint32_t RunTimes  = 0;

//static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId SensorTaskNameHandle;
osThreadId BluueCtolTaskNaHandle;
osThreadId CacuTaskNameHandle;
osThreadId GetTempTaskNameHandle;
osMessageQId SensorQueenHandle;
osMessageQId SpeedQueenHandle;
osMessageQId MouseQueenHandle;
osMutexId Usart1MutexHandle;
osMutexId SPI1MutexHandle;
osMutexId TIM3MutexHandle;
osSemaphoreId TIM3BinarySemHandle;
osSemaphoreId ADC1BinarySemHandle;
osSemaphoreId USART1BinarySemHandle;
osSemaphoreId SPI1BinarySemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void SensorTaskFunction(void const * argument);
void BluueCtolTaskFunction(void const * argument);
void CacuTaskFunction(void const * argument);
void GetTempTaskFunction(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

    RunTimes = timerMicros();
}

__weak unsigned long getRunTimeCounterValue(void)
{

    return (timerMicros() - RunTimes);

}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Create the mutex(es) */
    /* definition and creation of Usart1Mutex */
    osMutexDef(Usart1Mutex);
    Usart1MutexHandle = osMutexCreate(osMutex(Usart1Mutex));

    /* definition and creation of SPI1Mutex */
    osMutexDef(SPI1Mutex);
    SPI1MutexHandle = osMutexCreate(osMutex(SPI1Mutex));

    /* definition and creation of TIM3Mutex */
    osMutexDef(TIM3Mutex);
    TIM3MutexHandle = osMutexCreate(osMutex(TIM3Mutex));

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* definition and creation of TIM3BinarySem */
    osSemaphoreDef(TIM3BinarySem);
    TIM3BinarySemHandle = osSemaphoreCreate(osSemaphore(TIM3BinarySem), 1);

    /* definition and creation of ADC1BinarySem */
    osSemaphoreDef(ADC1BinarySem);
    ADC1BinarySemHandle = osSemaphoreCreate(osSemaphore(ADC1BinarySem), 1);

    /* definition and creation of USART1BinarySem */
    osSemaphoreDef(USART1BinarySem);
    USART1BinarySemHandle = osSemaphoreCreate(osSemaphore(USART1BinarySem), 1);

    /* definition and creation of SPI1BinarySem */
    osSemaphoreDef(SPI1BinarySem);
    SPI1BinarySemHandle = osSemaphoreCreate(osSemaphore(SPI1BinarySem), 1);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    osSemaphoreWait(TIM3BinarySemHandle, 0);
    osSemaphoreWait(ADC1BinarySemHandle, 0);


    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* definition and creation of SensorQueen */
    osMessageQDef(SensorQueen, 16, uint32_t);
    SensorQueenHandle = osMessageCreate(osMessageQ(SensorQueen), NULL);

    /* definition and creation of SpeedQueen */
    osMessageQDef(SpeedQueen, 16, uint32_t);
    SpeedQueenHandle = osMessageCreate(osMessageQ(SpeedQueen), NULL);

    /* definition and creation of MouseQueen */
    osMessageQDef(MouseQueen, 16, uint32_t);
    MouseQueenHandle = osMessageCreate(osMessageQ(MouseQueen), NULL);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of SensorTaskName */
    osThreadDef(SensorTaskName, SensorTaskFunction, osPriorityHigh, 0, 128);
    SensorTaskNameHandle = osThreadCreate(osThread(SensorTaskName), NULL);

    /* definition and creation of BluueCtolTaskNa */
    osThreadDef(BluueCtolTaskNa, BluueCtolTaskFunction, osPriorityAboveNormal, 0, 128);
    BluueCtolTaskNaHandle = osThreadCreate(osThread(BluueCtolTaskNa), NULL);

    /* definition and creation of CacuTaskName */
    osThreadDef(CacuTaskName, CacuTaskFunction, osPriorityNormal, 0, 128);
    CacuTaskNameHandle = osThreadCreate(osThread(CacuTaskName), NULL);

    /* definition and creation of GetTempTaskName */
    osThreadDef(GetTempTaskName, GetTempTaskFunction, osPriorityLow, 0, 128);
    GetTempTaskNameHandle = osThreadCreate(osThread(GetTempTaskName), NULL);

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
    int32_t ret;
    osPrintf("StartDefaultTask\r\n");
    osPrintf("xPortGetFreeHeapSize0 = %d\r\n", xPortGetFreeHeapSize());
		//chushi初始化陀螺仪零偏和刻度偏差
			CalcTempDiff();
    for(;;)
    {


        ret = lsm6dsl_IMU_init();
        if(HAL_OK == ret)
        {
            HAL_TIM_Base_Start_IT(&htim2); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE
            //开启定时器中断
            HAL_TIM_Base_Start_IT(&htim3); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE
            vTaskDelete(defaultTaskHandle);
        }
        osDelay(1000);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_SensorTaskFunction */
/**
* @brief Function implementing the SensorTaskName thread.
* @param argument: Not used;的得到带时间戳的数据
* @retval None
*/
/* USER CODE END Header_SensorTaskFunction */
void SensorTaskFunction(void const * argument)
{
    /* USER CODE BEGIN SensorTaskFunction */
    /* Infinite loop */
    osPrintf("SensorTaskFunction\r\n");
    osPrintf("xPortGetFreeHeapSize = %d\r\n", xPortGetFreeHeapSize());
    uint32_t sizeToMalloc;
    uint8_t * mempollHandle = NULL;
    uint8_t * TempMempollHandle = NULL;
    uint8_t * LSM6DSL_Axes_acc = NULL;
    uint8_t * LSM6DSL_Axes_gyo = NULL;
    uint8_t * aADCxConvertedData = NULL;
    sizeToMalloc = sizeof(uint32_t) + ADC_CONVERTED_DATA_BUFFER_SIZE * sizeof(uint16_t) + 2 * sizeof(LSM6DSL_Axes_t);
    while(NULL == (mempollHandle = pvPortMalloc(sizeToMalloc)))
    {
        osPrintf("pvPortMalloc error!\r\n");
        osDelay(10);
    }
    LSM6DSL_Axes_acc = mempollHandle;
    LSM6DSL_Axes_gyo = (LSM6DSL_Axes_acc + sizeof(LSM6DSL_Axes_t));
    aADCxConvertedData = (LSM6DSL_Axes_gyo +  sizeof(LSM6DSL_Axes_t));
    //初始化adc
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)aADCxConvertedData,
                          ADC_CONVERTED_DATA_BUFFER_SIZE) != HAL_OK)
    {
        osPrintf("adc convert error!\r\n");
    }
    for(;;)
    {
        //insure adc value is latest
        osSemaphoreWait(ADC1BinarySemHandle, 0);

//        osPrintf("TIM3BinarySemHandle=%d\r\n", osSemaphoreGetCount(TIM3BinarySemHandle));
        osSemaphoreWait(TIM3BinarySemHandle, osWaitForever);
        //get acc
        LSM6DSL_ACC_Driver.GetAxes(&obj_lsm6dsl, (LSM6DSL_Axes_t *)LSM6DSL_Axes_acc);
        //get gyo
        LSM6DSL_GYRO_Driver.GetAxes(&obj_lsm6dsl, (LSM6DSL_Axes_t *)LSM6DSL_Axes_gyo);
        //wait adc convert completed！        //get adc aADCxConvertedData[0 1 2 3 4]
        osSemaphoreWait(ADC1BinarySemHandle, osWaitForever);
        //disp adc value
//          osPrintf("adcvalue = %d\t%d\t%d\t%d\t%d\t\r\n",*(uint16_t *)(aADCxConvertedData+0),*(uint16_t *)(aADCxConvertedData+2),*(uint16_t *)(aADCxConvertedData+4),*(uint16_t *)(aADCxConvertedData+6),*(uint16_t *)(aADCxConvertedData+8));
        //增加时间戳
        *((uint32_t*)(mempollHandle + sizeToMalloc - sizeof(uint32_t))) = timerMicros();
        //malloc a mempool to contain the hole data
        if(NULL == (TempMempollHandle = pvPortMalloc(sizeToMalloc)))continue;
        memcpy(TempMempollHandle, mempollHandle, sizeToMalloc);//DMA mem to mem can save runtime
        //send a message or sem to say sensor data have got
        if(osOK != osMessagePut(SensorQueenHandle, (uint32_t)(TempMempollHandle), 0))
        {
            vPortFree(TempMempollHandle);
        }
    }
    /* USER CODE END SensorTaskFunction */
}

/* USER CODE BEGIN Header_BluueCtolTaskFunction */
/**
* @brief Function implementing the BluueCtolTaskNa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BluueCtolTaskFunction */
void BluueCtolTaskFunction(void const * argument)
{
    /* USER CODE BEGIN BluueCtolTaskFunction */
    /* Infinite loop */
    osPrintf("BluueCtolTaskFunction\r\n");
    osEvent MouseQueenBuffer;
    struct Control * MouseControl = NULL ;
    for(;;)
    {
        //get a message
        MouseQueenBuffer = osMessageGet( MouseQueenHandle, osWaitForever);

        if(osEventMessage == MouseQueenBuffer.status)
        {
            //wake up blueteeth
//                  mousePrintf("");
            //usart send data to blue
            MouseControl = MouseQueenBuffer.value.p;
            mousePrintf(MouseControl, MouseControl->length);
            //into blueteeth low power modle
//                                      mousePrintf("");

            //free mempool
            vPortFree(MouseQueenBuffer.value.p);
        }
    }
    /* USER CODE END BluueCtolTaskFunction */
}

/* USER CODE BEGIN Header_CacuTaskFunction */
/**
* @brief Function implementing the CacuTaskName thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CacuTaskFunction */
void CacuTaskFunction(void const * argument)
{
    /* USER CODE BEGIN CacuTaskFunction */
    /* Infinite loop */
    osPrintf("CacuTaskFunction\r\n");
    osEvent SensorBuffer;
    struct Control * MousePollHandle = NULL;
    uint32_t sizeToMalloc = sizeof(struct Control);

    for(;;)
    {
        //get a message
        SensorBuffer = osMessageGet( SensorQueenHandle, osWaitForever);
        //malloc a pool to contain MouseQueenHandle
        MousePollHandle = pvPortMalloc(sizeToMalloc);
        if(NULL != MousePollHandle)
        {
						//初始化鼠标数据包
            PutMouseControl_Init(MousePollHandle);
        }
        else
        {
            osPrintf("malloc error MousePollHandle");
            //free mempool
            if(osEventMessage == SensorBuffer.status)
            {
                vPortFree(SensorBuffer.value.p);
            }
            continue;
        }
        //caculator data 得到运动数据x,y,z，
				
				//手掌姿态数据
        
				//copy data to memary pool

        //send a message
        if(osOK != osMessagePut(MouseQueenHandle, (uint32_t)MousePollHandle, 0))
        {
            vPortFree(MousePollHandle);
        }
        //free mempool
        if(osEventMessage == SensorBuffer.status)
        {
            vPortFree(SensorBuffer.value.p);
        }

    }
    /* USER CODE END CacuTaskFunction */
}

/* USER CODE BEGIN Header_GetTempTaskFunction */
/**
* @brief Function implementing the GetTempTaskName thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GetTempTaskFunction */
void GetTempTaskFunction(void const * argument)
{
    /* USER CODE BEGIN GetTempTaskFunction */
    /* Infinite loop */
    osPrintf("GetTempTaskFunction\r\n");
    int16_t lsm6dsl_temperature;
	float rawtemp;
//    char *tempbuffer;
//    tempbuffer = pvPortMalloc(500);
    for(;;)
    {
//    vTaskList(tempbuffer);
//    osPrintf("%s\r\n", tempbuffer);
//        vTaskGetRunTimeStats(tempbuffer);
//        osPrintf("%s\r\n", tempbuffer);
//        osPrintf("xPortGetFreeHeapSize0 = %d Byte\r\n", xPortGetFreeHeapSize());
        if(HAL_OK != lsm6dsl_temperature_raw_get(&obj_lsm6dsl.Ctx, (uint8_t *)&lsm6dsl_temperature))
        {
            osPrintf("temp get error\r\n");
        }
        else
        {
rawtemp=calc_dtemp(lsm6dsl_temperature);
					osPrintf("lsm6dsl_temperature = %2.2f℃\r\n", rawtemp);
        }


        osDelay(1000);
    }
    /* USER CODE END GetTempTaskFunction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    //adc convert complete
    if(hadc == &hadc1)
    {
//    osSemaphoreRelease(ADC1BinarySemHandle);
        portBASE_TYPE taskWoken = pdFALSE;
        xSemaphoreGiveFromISR(ADC1BinarySemHandle, &taskWoken);
        portEND_SWITCHING_ISR( taskWoken );
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
    portBASE_TYPE taskWoken = pdFALSE;

    if(htim == (&htim3))
    {
        xSemaphoreGiveFromISR(TIM3BinarySemHandle, &taskWoken);
    }
    if(htim == (&htim2))
    {
        Timestamp += 0x10000;

    }
    portEND_SWITCHING_ISR( taskWoken );

}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
