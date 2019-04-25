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
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "sdio.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
//#include "usbd_cdc_if.h"
#include "led.h"
// #include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_FLASH_CS_PORT GPIOB
#define SPI_FLASH_CS_PIN GPIO_PIN_0
#define BUFFER_SIZE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static  char str[4096];
volatile uint8_t rx_len = 0;                          //接收数据长度
volatile uint8_t recv_end_flag = 0;                   //接收完成标记位
uint8_t rx_buffer[BUFFER_SIZE] = {0}; //接收数据缓存
int8_t usart1_send_complete_flag = 0;
__align(4) uint8_t rxbuf[100] = {0}, txbuf[100] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UserTestEntry(void);
void UserTestEntry1(void);
void UserTestEntry2(void);
void UserTestEntry3(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void  usart_transmit_comp_callback(UART_HandleTypeDef *huart){

//usart1_send_complete_flag=0;
//  return;
//}

void DMA_Usart_Send(uint8_t *buf, uint8_t len) //串口发送封装
{
//    HAL_UART_Transmit_DMA(&huart1, buf, len);
//    while (huart1.gState == HAL_UART_STATE_BUSY_TX);
HAL_UART_Transmit(&huart1, buf, len,1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

    HAL_UART_Transmit_DMA(&huart1, rx_buffer, sizeof(rx_buffer) - 1); //可以通过DMA把数据发出去

    HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer) - 1);  //重新使能接收
}
void print_rcc_freq_info(void)
{
    sprintf(str, "SYSCLK_Frequency is %dHZ\r\n", HAL_RCC_GetSysClockFreq());
    DMA_Usart_Send((uint8_t*)str, strlen(str));
    sprintf(str, "PCLK1_Frequency is %dHZ\r\n", HAL_RCC_GetPCLK1Freq());
    DMA_Usart_Send((uint8_t*)str, strlen(str));
    sprintf(str, "PCLK2_Frequency is %dHZ\r\n", HAL_RCC_GetPCLK2Freq());
    DMA_Usart_Send((uint8_t*)str, strlen(str));
    sprintf(str, "SystemCoreClock is %dHZ\r\n", SystemCoreClock);
    DMA_Usart_Send((uint8_t*)str, strlen(str));
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
    MX_SDIO_SD_Init();
    MX_USART1_UART_Init();
    MX_FATFS_Init();
    MX_RTC_Init();
    MX_USB_DEVICE_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    /* USER CODE BEGIN 2 */
//  hdma_usart1_tx.XferCpltCallback=usart1_dma_XferCpltCallback;
    if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer) - 1) != HAL_OK) //main函数while(1)前，启动一次DMA接收
    {
        Error_Handler();
    }
    print_rcc_freq_info();

    sprintf(str, "build date : %s\r\nbuild time :  %s \r\n", __DATE__, __TIME__);
    DMA_Usart_Send((uint8_t*)str, strlen(str));
    UserTestEntry();
    UserTestEntry1();
    UserTestEntry2();
    UserTestEntry3();
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
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /**Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
char *USER_Path = SDPath;
void mount_disk(void)
{
    uint8_t res = f_mount(&SDFatFS, "/", 0);
    if (res != FR_OK)
    {
        sprintf(str, "FAILED: %d\r\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));

        return;
    }
    sprintf(str, "MOUNT OK\r\n");
    DMA_Usart_Send((uint8_t*)str, strlen(str));

}

void format_disk(void)
{
    uint8_t res = 0;

    sprintf(str, "PROCESSING...\r\n");
    DMA_Usart_Send((uint8_t*)str, strlen(str));
    res = f_mkfs("/", FM_FAT | FM_ANY, 512, str, 512);
    if (res == FR_OK)
    {
        sprintf(str, "OK!\r\n");
        DMA_Usart_Send((uint8_t*)str, strlen(str));

    }
    else
    {
        sprintf(str, "failed with: %d\r\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));

    }
}

void create_file(void)
{
//  FIL file;
//  FIL *pf = &file;
    uint8_t res;

    res = f_open(&SDFile, "0:test_1.txt", FA_CREATE_ALWAYS  | FA_WRITE);
    if (res == FR_OK)
    {
        sprintf(str, "creat ok\r\n");
        DMA_Usart_Send((uint8_t*)str, strlen(str));
    }
    else
    {
        sprintf(str, "creat failed\r\n");
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        sprintf(str, "error code: %d\r\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));

    }

    f_printf(&SDFile, "hello fatfs!\r\n");

    res = f_close(&SDFile);
    if (res != FR_OK)
    {
        sprintf(str, "close file error\r\n");
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        sprintf(str, "error code: %d\r\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
    }
}

void get_disk_info(void)
{
//  FATFS fs;
    FATFS *fls = &SDFatFS;
    FRESULT res;
    DWORD fre_clust;

    res = f_getfree("/", &fre_clust, &fls);       /* Get Number of Free Clusters */
    if (res == FR_OK)
    {
        /* Print free space in unit of MB (assuming 4096 bytes/sector) */
        sprintf(str, "%ld KB Total Drive Space.\r\n%ld KB Available Space.\r\n",
                ((fls->n_fatent - 2)*fls->csize) * 4 / 8, (fre_clust * fls->csize) * 4 / 8);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
    }
    else
    {
        sprintf(str, "get disk info error\n");
        DMA_Usart_Send((uint8_t*)str, strlen(str));

        sprintf(str, "error code: %d\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));

    }
}

void read_file(void)
{
//  FIL file;
    FRESULT res;
    UINT bw;
    uint8_t rbuf[100] = {0};

    res = f_open(&SDFile, "0:/test_1.txt", FA_READ);
    if (res != FR_OK)
    {
        sprintf(str, "open error: %d\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        return;
    }
    f_read(&SDFile, rbuf, 15, &bw);
    sprintf(str, "%s,%d\n", rbuf,bw);
    DMA_Usart_Send((uint8_t*)str, bw);

    res = f_close(&SDFile);
    if (res != FR_OK)
    {
        sprintf(str, "close file error\n");
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        sprintf(str, "error code: %d\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
    }
}
void write_file(void)
{
    FRESULT res;
    UINT bw;
    uint32_t temp=0;
    res = f_open(&SDFile, "0:/write1.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        sprintf(str, "creat error: %d\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        return;
    }
//    for(; temp < 0xf; temp++)
//    {
//        sprintf((char*)txbuf, "%d\r\n", temp);
//    }
		sprintf((char*)txbuf, "%d\r\n", 1234567);
//    f_printf(&SDFile, txbuf);
		

    f_write(&SDFile, txbuf, strlen((char*)txbuf), &bw);
    DMA_Usart_Send((uint8_t*)"txbuf:", 6);
    DMA_Usart_Send((uint8_t*)txbuf, strlen((char*)txbuf));
    res = f_close(&SDFile);
    if (res != FR_OK)
    {
        sprintf(str, "close file error\n");
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        sprintf(str, "error code: %d\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
    }
    res = f_open(&SDFile, "0:/write1.txt", FA_READ | FA_WRITE|FA_OPEN_APPEND);
    if (res != FR_OK)
    {
        sprintf(str, "open error: %d\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        return;
    }
		    for(; temp <= 0xffff; temp++)
    {
        sprintf((char*)txbuf, "%d\r\n", temp);
					f_puts((const char *)txbuf,&SDFile);

    }
		
    f_read(&SDFile, rxbuf, strlen((char*)txbuf), &bw);
    DMA_Usart_Send((uint8_t*)"rxbuf:", 6);

    DMA_Usart_Send((uint8_t*)rxbuf, strlen((char*)txbuf));


    res = f_close(&SDFile);
    if (res != FR_OK)
    {
        sprintf(str, "close file error\n");
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        sprintf(str, "error code: %d\n", res);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
    }
}
uint8_t uer_button_step = 0, uer_button_step1 = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_PIN_4 == GPIO_Pin)
    {
        switch(uer_button_step ++)
        {
            case 0:
                break;
            case 1:
                break;
            default:
                break;
        }
        if(uer_button_step > 1)
        {
            uer_button_step = 1;
        }
        LED1TOG;

    }
    if(GPIO_PIN_3 == GPIO_Pin)
    {
        switch(uer_button_step1 ++)
        {
            case 0:
                break;
            case 1:
                break;
            default:
                break;
        }
        if(uer_button_step1 > 1)
        {
            uer_button_step1 = 1;
        }
        LED2TOG;

    }
}
void UserTestEntry(void)
{
    sprintf(str, "UserTestEntry bagin\r\n");
    DMA_Usart_Send((uint8_t*)str, strlen(str));


    /*
        //https://www.cnblogs.com/dongdongbh/p/6256931.html
        通过STM32CUBEMX生成DMA读写sdio的工程，再读写过程中总会卡死在DMA中断等待读写完成的while中，
        最终发现while等待的标志在SDIO的中断里置位的，而SDIO中断优先级如果小于或等于DMA中断优先级，
        则SDIO中断永远不能抢占DMA中断，DMA处于持续等待中，解决办法由两种，一种是直接提高SDIO中断
        优先级到比DMA中断优先级高，第二种是直接在HAL库中卡住的中断等待函数中注释掉while等待。
    　　另外还有一点就是在SDIO数据读写的时候需要注意的两点，一个是读写数据最好四字节对其，否则
        可能出BUG，二是读写BUFF的大小必须大于等于读写函数的SIZE参数，否则会出现内存莫名错误导致
        指针出乱，程序出乱。
        */
    mount_disk();
//  format_disk();
//  create_file();
    get_disk_info();
//    read_file();
//    write_file();

}
//  struct mouseHID_t {
//      uint8_t buttons;
//      int8_t x;
//      int8_t y;
//      int8_t wheel;
//  };
//extern    USBD_HandleTypeDef hUsbDeviceFS;
//    struct mouseHID_t mouseHID;

extern uint8_t* UserRxBufferFS;
void UserTestEntry1(void)
{
    sprintf(str, "UserTestEntry1 bagin\r\n");
    DMA_Usart_Send((uint8_t*)str, strlen(str));
//      for(char temp=0;1;temp++){
//  mouseHID.buttons = 0;
//  mouseHID.x = 10;
//  mouseHID.y = 0;
//  mouseHID.wheel = 0;
//  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mouseHID, 4);
//  HAL_Delay(1000);
//      }
//    uint32_t q32=2;
//              USBD_Interface_fops_FS.Receive((uint8_t*)str,&q32);

    for(uint32_t temp = 0; temp < 9; temp++)
    {
//      CDC_Transmit_FS((uint8_t*)str,q32);


//      HAL_Delay(5000);
        if(uer_button_step >= 1)
        {
            uer_button_step = 0;
//          sprintf(str,"button0\r\n");
            DMA_Usart_Send((uint8_t*)str, strlen(str));
//            USBD_Interface_fops_FS.Receive((uint8_t*)str,&q32);
        }

        if(uer_button_step1 >= 1)
        {
            uer_button_step1 = 0;
//          sprintf(str,"button1\r\n");
            DMA_Usart_Send((uint8_t*)str, strlen(str));
//            CDC_Transmit_FS((uint8_t*)UserRxBufferFS,q32+2);
        }
//      sprintf(str,"test cdc = %d\r\n",temp++);
//CDC_Transmit_FS((uint8_t*)str,strlen(str));
//          sprintf(str,"test cdc = %d\r\n",temp++);
//CDC_Transmit_FS((uint8_t*)str,strlen(str));
//      HAL_Delay(500);
    }
}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
//    LED1TOG;
}
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
    LED2TOG;
}
void StandbyMode_Measure(void)
{
    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Allow access to Backup */
    HAL_PWR_EnableBkUpAccess();

    /* Reset RTC Domain */
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();

    /* Disable all used wakeup sources: Pin1(PA.0) */
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

    /* Clear all related wakeup flags */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    /* Re-enable all used wakeup sources: Pin1(PA.0) */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

    /*## Enter Standby Mode ####################################################*/
    /* Request to enter STANDBY mode  */
    HAL_PWR_EnterSTANDBYMode();
}
void StopMode_Measure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Configure all GPIO as analog to reduce current consumption on non used IOs */
    /* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();

//            if(uer_button_step1 >= 1)
    {
//                          uer_button_step1=0;

        HAL_SuspendTick();
        HAL_PWREx_EnableFlashPowerDown();


        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        SystemClock_Config();
        print_rcc_freq_info();
    }
}
void UserTestEntry2(void)
{
    sprintf(str, "UserTestEntry2 bagin\r\n");
    DMA_Usart_Send((uint8_t*)str, strlen(str));
    sprintf(str, "HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)= 0x%x\r\n", HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1));
    DMA_Usart_Send((uint8_t*)str, strlen(str));



    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;

    for(uint32_t temp = 0; temp <= 2; temp++)
    {

        /* Get the RTC current Time */
        HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
        /* Get the RTC current Date */
        HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
        /* Display time Format : hh:mm:ss */
        sprintf((char *)str, "\r%2d:%2d:%2d, %2d-%2d-%2d",
                stimestructure.Hours, stimestructure.Minutes,
                stimestructure.Seconds, sdatestructure.Month,
                sdatestructure.Date, 2000 + sdatestructure.Year);
        DMA_Usart_Send((uint8_t*)str, strlen(str));
        HAL_Delay(1000);
    }





}

void UserTestEntry3(void)
{
    sprintf(str, "\r\nUserTestEntry3 bagin\r\n");
    DMA_Usart_Send((uint8_t*)str, strlen(str));
    CAN_Config();
    for(uint32_t temp = 0; temp >= 8; temp++)
    {
        if(uer_button_step >= 1)
        {
            uer_button_step = 0;
            if(TestCanTxMessage() == HAL_OK)
            {
                LED1OFF;
                sprintf(str, "(TestCanTxMessage() == HAL_OK)\r\n");
//           DMA_Usart_Send((uint8_t*)str,strlen(str));
            }

        }
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
