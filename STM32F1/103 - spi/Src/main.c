/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
//#include "mems_conf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BSP_LSM6DSL_CS_PORT GPIOA
#define BSP_LSM6DSL_CS_PIN GPIO_PIN_8
#define BUFFER_SIZE 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
//extern SPI_HandleTypeDef hspi1;

static  char str[100];
volatile uint8_t rx_len=0;                            //接收数据长度
volatile uint8_t recv_end_flag=0;                     //接收完成标记位
uint8_t rx_buffer[BUFFER_SIZE]={0},spi_rx_buffer[100],receivecomplete=0;   //接收数据缓存

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DMA_Usart_Send(uint8_t *buf,uint8_t len)//串口发送封装
{
    while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY){}	   

   if(HAL_UART_Transmit_DMA(&huart1, buf,len)!= HAL_OK)
        {
            Error_Handler();
        }
    while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY){}	   
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
    
     HAL_UART_Transmit_DMA(&huart1,rx_buffer,sizeof(rx_buffer)-1);//可以通过DMA把数据发出去
    
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer)-1);    //重新使能接收    
}
//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
//{
////if(hspi==&hspi1){
////启动接收
//	HAL_SPI_Receive(&hspi1,spi_rx_buffer,1,1000);

////}
//}
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{

//	sprintf(str,"spi_rx_buffer = 0x%x \r\n",spi_rx_buffer[0]);
//	receivecomplete=1;

//}


uint8_t SPI1SendByte(uint8_t data)
{
    unsigned char writeCommand[1];
    unsigned char readValue[1];

    writeCommand[0] = data;
//	HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&writeCommand, (uint8_t*)&readValue, 1, 10);
//HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_SET);

	return readValue[0];
}
void SPI1_WriteReg(uint8_t address, uint8_t value)
{
HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_RESET);
	SPI1SendByte(address);
 

    SPI1SendByte(value);
HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPI1_ReadReg(uint8_t address)
{
    uint8_t	val;

HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_RESET);
    SPI1SendByte(address);
 

    val = SPI1SendByte(0xff);
HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_SET);
    return val;
}
  void lsm6dsl_write_reg1(uint8_t reg, uint8_t value)
{
    uint8_t send_buffer[2];
    send_buffer[0] = reg;// & 0x7f;
    send_buffer[1] = value;
SPI1_WriteReg(send_buffer[0],send_buffer[1]);
//	BSP_LSM6DSL_SPI_Send(send_buffer, 2);


 }


  uint8_t lsm6dsl_read_reg1(uint8_t reg)
{
    uint8_t rxBuf[2];
    uint8_t txBuf[2];
   txBuf[0] = (reg | 0x80) & 0xbf; // reg|0x80;//SPI2_ReadWriteByte((reg|0x80)&0xbf);
    //发送寄存器号 //Ored with "read request" bit
//	if(BSP_LSM6DSL_SPI_Send(txBuf, 1)>0)
//        { 
//                if(BSP_LSM6DSL_SPI_Recv(rxBuf, 2)==2)
//                {                            
//                }               
//        }
//	HAL_SPI_TransmitReceive(&hspi1,txBuf, rxBuf,1,100);
rxBuf[0]=SPI1_ReadReg(txBuf[0]);

    return rxBuf[0];
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer)-1) != HAL_OK)//main函数while(1)前，启动一次DMA接收
    {
        Error_Handler();
    }	
sprintf(str,"begin %d \r\n",1);
DMA_Usart_Send((uint8_t*)str,30);
 
//    HAL_UART_Transmit(&huart1, (uint8_t *)&"\r\ninto HAL_UART_RxCpltCallback\r\n",32,0xffff);//验证进入这个函数了

//HAL_UART_Transmit_DMA(&huart1,(uint8_t*)str,30);
//HAL_UART_Receive_DMA(&huart1,(uint8_t*)str,1);



//read lsm6dsl
//lsm6dsl_write_reg1(0x12,0x01);
		for(long i=0xffff;i>0;i--);
sprintf(str,"begin 0x%x \r\n",lsm6dsl_read_reg1(0x0f));
DMA_Usart_Send((uint8_t*)str,30);
if(lsm6dsl_read_reg1(0x0f)==0x6a)
{sprintf(str,"id =0x6a \r\n" );
DMA_Usart_Send((uint8_t*)str,30);}
//		str[0]=0x0f;str[1]=0x0f;
//		HAL_SPI_Transmit(&hspi1,(uint8_t*)str,1,1000);
//		while(receivecomplete==1);
//		receivecomplete=0;
//		DMA_Usart_Send((uint8_t*)str,30);
//		 		for(long i=0xfffff;i>0;i--);
//				str[0]=0x0f;str[1]=0x0f;
//    HAL_SPI_Transmit_DMA(&hspi1,(uint8_t*)str,1);
//		    while (HAL_DMA_GetState(&hdma_spi1_tx) != HAL_DMA_STATE_READY){}	   
//			HAL_SPI_Receive_DMA(&hspi1,(uint8_t*)str,1);
//						    while (HAL_DMA_GetState(&hdma_spi1_rx) != HAL_DMA_STATE_READY){}	   
//sprintf(str,"begin 0x%x \r\n",str[0]);
//DMA_Usart_Send((uint8_t*)str,30);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
				for(long i=0xfffff;i>0;i--);

sprintf(str,"begin 0x%x \r\n",lsm6dsl_read_reg1(0x0f));
DMA_Usart_Send((uint8_t*)str,30);
if(lsm6dsl_read_reg1(0x0f)==0x6a)
{sprintf(str,"id =0x6a \r\n" );
DMA_Usart_Send((uint8_t*)str,30);}
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
