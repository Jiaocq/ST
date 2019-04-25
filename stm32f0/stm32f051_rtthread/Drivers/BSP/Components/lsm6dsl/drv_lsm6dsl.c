#include "user_config.h"
#ifdef USER_USING_LSM6DSL
#include "drv_lsm6dsl.h"
#include "mems_conf.h"
#include "custom_bus.h"
#include "led.h"

extern SPI_HandleTypeDef hspi1;


static void lsm6dsl_write_reg1(uint8_t reg, uint8_t value)
{
    uint8_t send_buffer[2];
  HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_SET);

    send_buffer[0] = reg & 0x7f;
    send_buffer[1] = value;
	BSP_LSM6DSL_SPI_Send(send_buffer, 2);
	  HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_RESET);

 }

//
static uint8_t lsm6dsl_read_reg1(uint8_t reg)
{
    uint8_t rxBuf[2];
    uint8_t txBuf[2];
  HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_SET);

    txBuf[0] = (reg | 0x80) & 0xbf; // reg|0x80;//SPI2_ReadWriteByte((reg|0x80)&0xbf);
    //发送寄存器号 //Ored with "read request" bit
//	if(BSP_LSM6DSL_SPI_Send(txBuf, 1)>0)
//        { 
//                if(BSP_LSM6DSL_SPI_Recv(rxBuf, 2)==2)
//                {                            
//                }               
//        }
	HAL_SPI_TransmitReceive(&hspi1,txBuf, rxBuf,1,100);
//	  if(HAL_SPI_Transmit(&hspi1, txBuf, 1, 1000) == HAL_OK)
//  {

//	  if(HAL_SPI_Receive(&hspi1, rxBuf, 2, 1000) == HAL_OK)
//  {
//   }
//   }
   HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_RESET);

    return rxBuf[0];
}


static uint8_t lsm6dsl_read_buffer1(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[20];
  HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_SET);

    txBuf[0] = (reg | 0x80) & 0xbf; // reg|0x80;
		BSP_LSM6DSL_SPI_Send(txBuf, 1);
	
BSP_LSM6DSL_SPI_Recv(rxBuf, len);
     rt_memcpy(buffer, rxBuf, len);
   HAL_GPIO_WritePin(BSP_LSM6DSL_CS_PORT, BSP_LSM6DSL_CS_PIN, GPIO_PIN_RESET);

    return 0;
}

int lsm6dsl_read_alldata1( rt_int16_t *buffer)
{
//	rt_uint8_t buff1[2]={LSM6DSL_OUT_TEMP_L,LSM6DSL_OUT_TEMP_L};
//	 rt_spi_send_then_recv(spi_imu_lsm6dsl->rt_spi_device, buff1, 1, (uint8_t*)buffer, 14);

    lsm6dsl_read_buffer1(LSM6DSL_OUT_TEMP_L,(uint8_t*)buffer,14);
    return    RT_EOK;
}

uint8_t lsm6dsl_get_accel(int16_t *accel, int16_t *temp)
{
    uint8_t buf[8];
    lsm6dsl_read_buffer1(LSM6DSL_OUTX_L_XL, buf, 6);

    accel[0] = *((rt_int16_t*)(buf + 0))    ; //((int16_t)buf[0]<<8) + buf[1];
    accel[1] = *((rt_int16_t*)(buf + 2))    ; //((int16_t)buf[2]<<8) + buf[3];
    accel[2] = *((rt_int16_t*)(buf + 4))    ; //((int16_t)buf[4]<<8) + buf[5];

    return 0;
}


uint8_t lsm6dsl_get_gyro(int16_t *gyro)
{
    uint8_t buf[8];
    lsm6dsl_read_buffer1(LSM6DSL_OUTX_L_G, buf, 8);

    gyro[0] = *((rt_int16_t*)(buf + 0))    ; // (buf[0]<<8) + buf[1];
    gyro[1] = *((rt_int16_t*)(buf + 2))    ; //(buf[2]<<8) + buf[3];
    gyro[2] = *((rt_int16_t*)(buf + 4))    ; //(buf[4]<<8) + buf[5];

    return 0;
}
uint8_t lsm6dsl_get_temp(int16_t *temp)
{
    uint8_t buf[4];
    lsm6dsl_read_buffer1(LSM6DSL_OUT_TEMP_L, buf, 2);

    temp[0] = *((rt_int16_t*)(buf + 0))    ; // (buf[0]<<8) + buf[1];

    return 0;
}

uint8_t lsm6dsl_get_AllData(int16_t *AllData)
{
    uint8_t buf[16], i = 0;
    lsm6dsl_read_buffer1(LSM6DSL_OUT_TEMP_L, buf, 14);
    for(i = 0; i < 7; i++)
    {
        AllData[i] = *((rt_int16_t*)(buf + 2 * i)) ;
    }

    return 0;
}


  
rt_err_t lsm6dsl_IMU_init(void)
{
    uint8_t id;
    rt_uint8_t ImuSetBuffer = 0;

    /* init sensor*/

 
    lsm6dsl_write_reg1(LSM6DSL_CTRL3_C, 0x01);  //复位，

    rt_thread_delay(50);
//    lsm6dsl_write_reg(LSM6DSL_PWR_MGMT_1, 0x01);      //关闭睡眠，自动选择时钟
//    rt_thread_delay(50);

    id = lsm6dsl_read_reg1(LSM6DSL_WHO_AM_I);//读取ID
    rt_kprintf("lsm6dsl id=0x%x\r\n", id);

    if(id != 0x6a)
    {
        rt_kprintf("lsm6dsl id error !!!\r\n");
        return -RT_ENOSYS;

    }

//    rt_kprintf("lsm6dsl init pass\r\n");


    //设置量程// ACC scale
    ImuSetBuffer = 0;
#if LSM6DSL_ACC_SCALE == 2
    ImuSetBuffer |= LSM6DSL_FS_XL_2g;
#elif LSM6DSL_ACC_SCALE == 4
    ImuSetBuffer |= LSM6DSL_FS_XL_4g;
#elif LSM6DSL_ACC_SCALE == 8
    ImuSetBuffer |= LSM6DSL_FS_XL_8g;
#else //LSM6DSL_ACC_SCALE == 16
    ImuSetBuffer |= LSM6DSL_FS_XL_16g;
#endif
    //设置acc采样率
#ifdef LSM6DSL_ACC_SAMPLERATE
    ImuSetBuffer |= LSM6DSL_ACC_SAMPLERATE;
#else
    ImuSetBuffer |= LSM6DSL_ODR_XL_1660Hz;
#endif
    lsm6dsl_write_reg1(LSM6DSL_CTRL1_XL, ImuSetBuffer);
    rt_thread_delay(10);

//acc lpf
    ImuSetBuffer = 0;
    ImuSetBuffer |= 0xa9;//0xc9; //  odr / 9,     400/9 0b1100 1001.odr/50=0b1000 1001.odr/100=0b1010 1001
    lsm6dsl_write_reg1(LSM6DSL_CTRL8_XL, ImuSetBuffer);



    rt_thread_delay(10);
    // GYO scale

    ImuSetBuffer = 0;
#if LSM6DSL_GYO_SCALE == 250
    ImuSetBuffer |= LSM6DSL_FS_G_250dps;
#elif LSM6DSL_GYO_SCALE == 500
    ImuSetBuffer |= LSM6DSL_FS_G_500dps;
#elif LSM6DSL_GYO_SCALE == 1000
    ImuSetBuffer |= LSM6DSL_FS_G_1000dps;
#else//elif LSM6DSL_GYO_SCALE == 2000
    ImuSetBuffer |= LSM6DSL_FS_G_2000dps;
#endif
    //设置gyo采样率
#ifdef LSM6DSL_GYO_SAMPLERATE
    ImuSetBuffer |= LSM6DSL_GYO_SAMPLERATE;
#else
    ImuSetBuffer |= LSM6DSL_ODR_G_1660Hz;
#endif
    lsm6dsl_write_reg1(LSM6DSL_CTRL2_G, ImuSetBuffer);

    rt_thread_delay(10);
//gyo lpf
    ImuSetBuffer = 0;
    ImuSetBuffer |= 0x06; //  使能lpf /禁用iic 禁用睡眠
    lsm6dsl_write_reg1(LSM6DSL_CTRL4_C, ImuSetBuffer);
    rt_thread_delay(10);

    ImuSetBuffer = 0;
    ImuSetBuffer |= 0x02; //  odr=800.lpf=155
    lsm6dsl_write_reg1(LSM6DSL_CTRL6_G, ImuSetBuffer);


     return RT_EOK;
}
 
void  lsm6dsl_thread_entry(void*parameter)
{

	rt_int16_t id=0,rec_bufer[10],num;


	while(1)
		{rt_hw_led_on();
			    id = lsm6dsl_read_reg1(LSM6DSL_WHO_AM_I);//读取ID
    rt_kprintf("lsm6dsl id=0x%x\r\n", id);
 
rt_hw_led_off();
	rt_thread_delay(10);



	}
}
 

int lsm6dsl_thread_init(void)
{
rt_thread_t tid;
 BSP_LSM6DSL_SPI_Init();
	lsm6dsl_apply_spi_settings();
    lsm6dsl_IMU_init();

 //    /* 创建线程 */
    tid = rt_thread_create("lsm6dsl",
                           lsm6dsl_thread_entry,
                           RT_NULL,
                           1024,
                           25,
                           10);
    /* 创建成功则启动线程 */
    if (tid == RT_NULL)
    {
        return RT_ERROR;
    }
    rt_thread_startup(tid);
    return RT_EOK;
}


INIT_APP_EXPORT(lsm6dsl_thread_init);
#endif

