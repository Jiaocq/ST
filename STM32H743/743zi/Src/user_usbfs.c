#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

#include "user_usbfs.h"
#include "led.h"




////////////////////////////////////////////////
volatile uint32_t byteswritten = 0, bytesread = 0; /* File write/read counts */
uint8_t wtext[] = "This is STM32 MSC working with FatFs, GoodGood Study, DayDayUp"; /* File write buffer */
volatile uint8_t uer_button_step = 0;

void readfile_test(void)
{
    FIL fil;
    FRESULT fr;
    uint8_t rtext[100]; /* File read buffer */
    if(f_open(&fil, "0:/mySTtest.txt", FA_READ) != FR_OK) /* Opens an existing file. If not exist, creates a new file. */
    {
        return;
    }
    else
    {
        fr = f_read(&fil, rtext, sizeof(rtext), (void *)&bytesread);
        if((bytesread == 0) || (fr != FR_OK))
        {
            return;
        }
        else
        {
            f_close(&fil);
            if((bytesread != byteswritten))
            {
                return;
            }
        }
    }
}

void writefile_test(void)
{
    FIL fil;
    FRESULT fr;
    if(f_open(&fil, "0:/mySTtest.txt", FA_READ | FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
    {
        return;
    }
    else
    {
        fr = f_write(&fil, wtext, sizeof(wtext), (void *)&byteswritten);
        if((byteswritten == 0) || (fr != FR_OK))
        {
            return;
        }
        else
        {
            f_close(&fil);
        }
    }
}

//////////////////////////////////////////////////////////
extern USBH_HandleTypeDef hUsbHostFS;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(USBH_MSC_IsReady(&hUsbHostFS))
    {
        switch(uer_button_step ++)
        {
        case 0:
            writefile_test(); //写文件测试
            break;
        case 1:
            readfile_test(); //读文件测试
            break;
        default:
            break;
        }
        if(uer_button_step > 1)
        {
            uer_button_step = 0;
        }
    }
		LED1TOG;
}

///////////////////////////////////////////
extern ApplicationTypeDef Appli_state;
static ApplicationTypeDef pre_state = APPLICATION_IDLE;
FATFS fs;
extern void DMA_Usart_Send(uint8_t *buf,uint8_t len);
static char str[100];
void testUsbMassStorigeFS(void)
{
    {
        MX_USB_HOST_Process();
        if (pre_state != Appli_state)
        {
            switch(Appli_state)
            {
            case APPLICATION_DISCONNECT: //USB flash disk remove
                if(f_mount(NULL, "", 0) != FR_OK)
                {
                  sprintf(str,"ERROR : Cannot exit FatFs! \n");
									DMA_Usart_Send((uint8_t*)str,strlen(str));
                }
                break;
            case APPLICATION_READY: //USB flash disk plugin
                if(f_mount(&fs, "", 0) != FR_OK)
                {
                    break;
                }
                break;
            default:
                break;
            }
            pre_state = Appli_state;
        } //end if (pre_state != Appli_state)
    }
}






