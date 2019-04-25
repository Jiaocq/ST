/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
/* Demo application includes. */
#include "serial.h"
#include "usart.h"
//#include "string.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE				( ( QueueHandle_t ) 0 )
#define serNO_BLOCK						( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME				( 40 / portTICK_PERIOD_MS )

/*-----------------------------------------------------------*/

/* The queue used to hold received characters. */
static QueueHandle_t xRxedChars;
static QueueHandle_t xCharsForTx;
uint8_t  rx_buffer[4]={0}; 
/*-----------------------------------------------------------*/

/* UART interrupt handler. */
void vUARTInterruptHandler( void );

/*-----------------------------------------------------------*/

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{

//uint32_t ulChar;
xComPortHandle xReturn;


	/* Create the queues used to hold Rx/Tx characters. */
	xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );

	/* If the queues were created correctly then setup the serial port
	hardware. */
	if( ( xRxedChars != serINVALID_QUEUE ) && ( xCharsForTx != serINVALID_QUEUE ) )
	{
		xReturn = &huart3;
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)rx_buffer, 1);  //重新使能接收

		
		/* Enable the peripheral clock in the PMC. */
		/* Configure USART in serial mode. */
		/* Disable all the interrupts. */
		/* Enable the receiver and transmitter. */
		/* Clear any characters before enabling interrupt. */
		/* Enable Rx end interrupt. */
		/* Configure and enable interrupt of USART. */

	}
	else
	{
		xReturn = ( xComPortHandle ) 0;
	}

	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
	return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
	/* The port handle is not required as this driver only supports one port. */
//	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
//char* p=(char* )pcString;
	/* Send each character in the string, one at a time. */
//	HAL_UART_Transmit( (UART_HandleTypeDef*)pxPort, (uint8_t*)pcString,(uint16_t)usStringLength ,1000);	
//	while(usStringLength--)
//	xSerialPutChar( pxPort, *(p++), serTX_BLOCK_TIME);
	osPrintf(pcString);
	

}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{
	char p[2]={0};
	p[0]=cOutChar;
	p[1]=0;
	osPrintf(p);
//if(HAL_OK == HAL_UART_Transmit( (UART_HandleTypeDef*)pxPort, (uint8_t*)&cOutChar,1 ,100))
//	return pdPASS;
	return pdFAIL;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/

void vUARTInterruptHandler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
char cChar;
 
	if( __HAL_UART_GET_IT( &huart3, UART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* A character was retrieved from the queue so can be sent to the
			THR now. */
			HAL_UART_Transmit_IT( &huart3,(uint8_t*)&cChar ,1);
		}
		else
		{
			__HAL_UART_DISABLE_IT( &huart3, UART_IT_TXE );		
		}		
	}
	
	if( __HAL_UART_GET_IT(  &huart3, UART_IT_RXNE ) == SET )
	{
		cChar = HAL_UART_Receive_IT( &huart3,(uint8_t*)&cChar ,1);
		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

    void HAL_UART_RxCpltCallback(UART_HandleTypeDef * UartHandle)
    {
			portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

xQueueSendFromISR( xRxedChars, rx_buffer, &xHigherPriorityTaskWoken );	
			HAL_UART_Receive_DMA(&huart3, (uint8_t *)rx_buffer, 1);  //重新使能接收

			portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }



	
