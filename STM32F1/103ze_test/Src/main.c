/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 2


#define TIMER_TIM			htim6	// can only use 32bit timers TIM2 or TIM5
#define TIMER_CLOCK			(rccClocks.PCLK1_Frequency * 2)
#define TIMER_EN			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE)
#define TIMER_IRQ_CH		TIM6_IRQn
#define TIMER_ISR			TIM6_IRQHandler
//#define TIMER_CORE_HALT		DBGMCU_TIM5_STOP

#define timerMicros()		(uint32_t)((uint32_t)TIMER_TIM.Instance->CNT+clock6high8)
#define timerStart()		timerStart = timerMicros()
#define timerStop()			(timerMicros() - timerStart)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buffer[BUFFER_SIZE]={0};
uint32_t clock6high8=0,timerStart;
   char str[100]={0};
uint16_t adc_con_val[100]={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void	UserTaskEntry(void);
void	UserTaskEntry1(void);
	
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DMA_Usart_Send(uint8_t *buf,uint8_t len)//串口发送封装
{
//    while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY){}
//      while (0!= usart1_send_complete_flag){}	   

//    while ((hdma_usart1_tx.Instance->CNDTR)>0);
			HAL_UART_Transmit_DMA(&huart1, buf,len);
//	    while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY){}

      	while (huart1.gState==HAL_UART_STATE_BUSY_TX){;	}	
//				usart1_send_complete_flag=1;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
    if(rx_buffer[0]!=0x0D)
     HAL_UART_Transmit_DMA(&huart1,rx_buffer,sizeof(rx_buffer)-1);//可以通过DMA把数据发出去
    else
			     HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"\r\n",2);//可以通过DMA把数据发出去

    HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer)-1);    //重新使能接收    
}
//回调函数，定时器中断服务函数调用的HAL库函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&htim6))
    { 
//				DMA_Usart_Send((uint8_t*)"begin6\r\n",8);
        clock6high8+=0x10000;

			
    }
    if(htim==(&htim7))
    { 
    }		
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
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
//	UserTaskEntry();
	UserTaskEntry1();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void	UserTaskEntry(void){
	volatile uint32_t i=0,temptime=0,temptime1=0;
	volatile uint32_t testint32=0,q2=0xffffffff,q3=0xf0000000;
	volatile float testfloat=1.1f,q1=1.1f,q4=2.2f;
	volatile double testdouble=1.1,q5=1.1f,q6=2.2f,q7=3.3f;
	
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer)-1);//qi启动usart receive
//	DMA_Usart_Send((uint8_t*)"begin\r\n",7);
//	for(;i>0;i--)
// 	sprintf(str,"timerMicros = %d\r\n",timerMicros());
//  DMA_Usart_Send((uint8_t*)str,strlen(str));
	/////////////////////////////////////
	i=0xffffffff;temptime=0xffffffff;
	timerStart();
	i=3000;
	for(;i>0;i--){
	}
temptime=	timerStop();
///////////////////////////////////////	
	i=0xffffffff;temptime1=0xffffffff;
	timerStart();
	i=3000;
	for(;i>0;i--){
//		testdouble=testdouble+testdouble;
//		 testfloat=q1-q4;
//		 testint32=q2*q3;
testfloat=sin(i);
	}
	temptime1=timerStop();
 	sprintf(str,"testdouble = %d\r\n",temptime1-temptime);
  DMA_Usart_Send((uint8_t*)str,strlen(str));	
/////////////////////////////////////////////
//	i=0;
//	timerStart();
//	i=300000;
//	for(;i>0;i--){
//testint32=q2*q3;
//	}
//	temptime1=timerStop();
// 	sprintf(str,"testint32 = %d\r\n",temptime1-temptime);
//  DMA_Usart_Send((uint8_t*)str,strlen(str));	
///////////////////////////////////////////////
//	i=0;
//	timerStart();
//	i=300000;
//	for(;i>0;i--){
//testfloat=q1*q4;
//	}
//	temptime1=timerStop();
// 	sprintf(str,"testfloat = %d\r\n",temptime1-temptime);
//  DMA_Usart_Send((uint8_t*)str,strlen(str));		
//	
//	 	sprintf(str,"double int32_t float = %d\t%d\t%d\r\n",sizeof(double),sizeof(int32_t),sizeof(float));
//  DMA_Usart_Send((uint8_t*)str,strlen(str));
// 	sprintf(str,"runover = %d\r\n",timerMicros());
//  DMA_Usart_Send((uint8_t*)str,strlen(str));	





}

void	UserTaskEntry1(void){
	DMA_Usart_Send((uint8_t*)"begin\r\n",7);

//HAL_ADC_Start(&hadc1);
 HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_con_val,3);
	while(1){
 	sprintf(str,"adc =\t%d\t%d\t%d\t%d\t%d\t\r\n",adc_con_val[0],adc_con_val[1],adc_con_val[2],adc_con_val[3],adc_con_val[4]);
  DMA_Usart_Send((uint8_t*)str,strlen(str));	
	
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
