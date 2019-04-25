/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_LSM6DSL_H
#define __DRV_LSM6DSL_H
#include "stm32f1xx_hal.h"

#include "drv_lsm6dsl.h"
 
 #include "user_config.h"
 #define LSM6DSL_SPI_BAUD	    (SPI_BAUDRATEPRESCALER_32)
#define LSM6DSL_READ_BIT	    (0x01<<7)
#define LSM6DSL_WRITE_BIT	    (0x00<<7)

#define LSM6DSL_ACC_SCALE 8

#define LSM6DSL_ACC_SAMPLERATE LSM6DSL_ODR_XL_3330Hz
#define LSM6DSL_GYO_SCALE 1000
#define LSM6DSL_GYO_SAMPLERATE LSM6DSL_ODR_G_1660Hz
 

#define BSP_LSM6DSL_CS_PORT GPIOA
#define BSP_LSM6DSL_CS_PIN GPIO_PIN_8
#define BUFFER_SIZE 2

//void LSM6DSL_test();
//void LSM6DSL_Init(void);//初始化
//u8 LSM6DSL_Read_Reg(u8 reg);			//读寄存器
//u8 LSM6DSL_Write_Reg(u8 reg, u8 value);//写寄存器
//u8 LSM6DSL_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);//写数据区
//u8 LSM6DSL_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);//读数据区
//u8 LSM6DSL_Check(void);//检查24L01是否存在
//u8 LSM6DSL_Get_RawAcc(short *ax,short *ay,short *az);
uint8_t lsm6dsl_get_accel(int16_t *accel, int16_t *temp);
uint8_t lsm6dsl_get_gyro(int16_t *gyro);
uint8_t lsm6dsl_get_temp(int16_t *temp);
uint8_t lsm6dsl_get_AllData(int16_t *AllData);
int lsm6dsl_read_alldata( int16_t *buffer);
int lsm6dsl_IMU_init(void);

// Return values
typedef enum
{
    IMU_SUCCESS = 0,
    IMU_HW_ERROR = 1,
    IMU_NOT_SUPPORTED= 2,
    IMU_GENERIC_ERROR= 3,
    IMU_OUT_OF_BOUNDS= 4,
    IMU_ALL_ONES_WARNING= 5,
    //...
} status_t;

/*******************************************************************************
* Register      : FIFO_CTRL5
* Address       : 0X0A
* Bit Group Name: ODR_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
    LSM6DSL_ODR_FIFO_10Hz 		 = 0x08,
    LSM6DSL_ODR_FIFO_25Hz 		 = 0x10,
    LSM6DSL_ODR_FIFO_50Hz 		 = 0x18,
    LSM6DSL_ODR_FIFO_100Hz 		 = 0x20,
    LSM6DSL_ODR_FIFO_200Hz 		 = 0x28,
    LSM6DSL_ODR_FIFO_400Hz 		 = 0x30,
    LSM6DSL_ODR_FIFO_800Hz 		 = 0x38,
    LSM6DSL_ODR_FIFO_1600Hz 		 = 0x40,
    LSM6DSL_ODR_FIFO_3300Hz 		 = 0x48,
    LSM6DSL_ODR_FIFO_6600Hz 		 = 0x50,
    LSM6DSL_ODR_FIFO_13300Hz 		 = 0x58,
} LSM6DSL_ODR_FIFO_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
    LSM6DSL_ODR_XL_POWER_DOWN 		   = 0x00,
//	LSM6DSL_ODR_XL_1p6Hz			   = 0b10110000,
    LSM6DSL_ODR_XL_13Hz 		       = 0x10,
    LSM6DSL_ODR_XL_26Hz 		       = 0x20,
    LSM6DSL_ODR_XL_52Hz 		       = 0x30,
    LSM6DSL_ODR_XL_104Hz 		 	= 0x40,
    LSM6DSL_ODR_XL_208Hz 		 	= 0x50,
    LSM6DSL_ODR_XL_416Hz 		 	= 0x60,
    LSM6DSL_ODR_XL_833Hz 		 	= 0x70,
    LSM6DSL_ODR_XL_1660Hz 			= 0x80,
    LSM6DSL_ODR_XL_3330Hz 		 	= 0x90,
    LSM6DSL_ODR_XL_6660Hz 		 	= 0xA0,
//	LSM6DSL_ODR_XL_13330Hz 		 = 0xB0,

} LSM6DSL_ODR_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
    LSM6DSL_FS_XL_2g 		 = 0x00,
    LSM6DSL_FS_XL_16g 		 = 0x04,
    LSM6DSL_FS_XL_4g 		 = 0x08,
    LSM6DSL_FS_XL_8g 		 = 0x0C,
} LSM6DSL_FS_XL_t;


/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: BW_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
    LSM6DSL_BW_XL_HIGH 		 = 0x00,
    LSM6DSL_BW_XL_LOW 		 = 0x10,
} LSM6DSL_BW_XL_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: FS_125
* Permission    : RW
*******************************************************************************/
typedef enum {
    LSM6DSL_FS_125_DISABLED 		 = 0x00,
    LSM6DSL_FS_125_ENABLED 		 = 0x02,
} LSM6DSL_FS_125_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: FS_G
* Permission    : RW
*******************************************************************************/
typedef enum {
    LSM6DSL_FS_G_250dps 		 = 0x00,
    LSM6DSL_FS_G_500dps 		 = 0x04,
    LSM6DSL_FS_G_1000dps 		 = 0x08,
    LSM6DSL_FS_G_2000dps 		 = 0x0C,
} LSM6DSL_FS_G_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: ODR_G
* Permission    : RW
*******************************************************************************/
typedef enum {
    LSM6DSL_ODR_G_POWER_DOWN 		 = 0x00,
    LSM6DSL_ODR_G_13Hz 		 = 0x10,
    LSM6DSL_ODR_G_26Hz 		 = 0x20,
    LSM6DSL_ODR_G_52Hz 		 = 0x30,
    LSM6DSL_ODR_G_104Hz 		 = 0x40,
    LSM6DSL_ODR_G_208Hz 		 = 0x50,
    LSM6DSL_ODR_G_416Hz 		 = 0x60,
    LSM6DSL_ODR_G_833Hz 		 = 0x70,
    LSM6DSL_ODR_G_1660Hz 		 = 0x80,
    LSM6DSL_ODR_G_3330Hz 		 = 0x90,
    LSM6DSL_ODR_G_6660Hz 		 = 0xa0,
} LSM6DSL_ODR_G_t;




//////////////////////////////////////////////////////////////////////////////////////////////////////////
/************** Device Register  *******************/
#define LSM6DSL_FUNC_CFG_ACCESS  									0X01
#define LSM6DSL_SENSOR_SYNC_TIME_FRAME  						0X04
#define LSM6DSL_SENSOR_SYNC_RES_RATIO  						0X05
#define LSM6DSL_FIFO_CTRL1  			0X06
#define LSM6DSL_FIFO_CTRL2  			0X07
#define LSM6DSL_FIFO_CTRL3  			0X08
#define LSM6DSL_FIFO_CTRL4  			0X09
#define LSM6DSL_FIFO_CTRL5  			0X0A
#define LSM6DSL_DRDY_PULSE_CFG_G 0X0B
#define LSM6DSL_INT1_CTRL  			0X0D
#define LSM6DSL_INT2_CTRL  			0X0E
#define LSM6DSL_WHO_AM_I  			0X0F
#define LSM6DSL_CTRL1_XL  			0X10
#define LSM6DSL_CTRL2_G  			0X11
#define LSM6DSL_CTRL3_C  			0X12
#define LSM6DSL_CTRL4_C  			0X13
#define LSM6DSL_CTRL5_C  			0X14//自检
#define LSM6DSL_CTRL6_G  			0X15
#define LSM6DSL_CTRL7_G  			0X16//gyo hpf
#define LSM6DSL_CTRL8_XL  			0X17
#define LSM6DSL_CTRL9_XL  			0X18
#define LSM6DSL_CTRL10_C  			0X19
#define LSM6DSL_MASTER_CONFIG  0X1A
#define LSM6DSL_WAKE_UP_SRC  	0X1B
#define LSM6DSL_TAP_SRC  			0X1C
#define LSM6DSL_D6D_SRC  			0X1D
#define LSM6DSL_STATUS_REG  			0X1E
#define LSM6DSL_OUT_TEMP_L  			0X20
#define LSM6DSL_OUT_TEMP_H  			0X21
#define LSM6DSL_OUTX_L_G  			0X22
#define LSM6DSL_OUTX_H_G  			0X23
#define LSM6DSL_OUTY_L_G  			0X24
#define LSM6DSL_OUTY_H_G  			0X25
#define LSM6DSL_OUTZ_L_G  			0X26
#define LSM6DSL_OUTZ_H_G  			0X27
#define LSM6DSL_OUTX_L_XL  			0X28
#define LSM6DSL_OUTX_H_XL  			0X29
#define LSM6DSL_OUTY_L_XL  			0X2A
#define LSM6DSL_OUTY_H_XL  			0X2B
#define LSM6DSL_OUTZ_L_XL  			0X2C
#define LSM6DSL_OUTZ_H_XL  			0X2D
#define LSM6DSL_SENSORHUB1_REG  		0X2E
#define LSM6DSL_SENSORHUB2_REG  		0X2F
#define LSM6DSL_SENSORHUB3_REG  		0X30
#define LSM6DSL_SENSORHUB4_REG  		0X31
#define LSM6DSL_SENSORHUB5_REG  		0X32
#define LSM6DSL_SENSORHUB6_REG  		0X33
#define LSM6DSL_SENSORHUB7_REG  		0X34
#define LSM6DSL_SENSORHUB8_REG  		0X35
#define LSM6DSL_SENSORHUB9_REG  		0X36
#define LSM6DSL_SENSORHUB10_REG  		0X37
#define LSM6DSL_SENSORHUB11_REG  		0X38
#define LSM6DSL_SENSORHUB12_REG  		0X39
#define LSM6DSL_FIFO_STATUS1  				0X3A
#define LSM6DSL_FIFO_STATUS2  				0X3B
#define LSM6DSL_FIFO_STATUS3  				0X3C
#define LSM6DSL_FIFO_STATUS4  				0X3D
#define LSM6DSL_FIFO_DATA_OUT_L  		0X3E
#define LSM6DSL_FIFO_DATA_OUT_H  		0X3F
#define LSM6DSL_TIMESTAMP0_REG  			0X40
#define LSM6DSL_TIMESTAMP1_REG  			0X41
#define LSM6DSL_TIMESTAMP2_REG  			0X42
#define LSM6DSL_STEP_TIMESTAMP_L  		0X49
#define LSM6DSL_STEP_TIMESTAMP_H  		0X4A
#define LSM6DSL_STEP_COUNTER_L  			0X4B
#define LSM6DSL_STEP_COUNTER_H  			0X4C
#define LSM6DSL_SENSORHUB13_REG  		0X4D
#define LSM6DSL_SENSORHUB14_REG  		0X4E
#define LSM6DSL_SENSORHUB15_REG  		0X4F
#define LSM6DSL_SENSORHUB16_REG  		0X50
#define LSM6DSL_SENSORHUB17_REG  		0X51
#define LSM6DSL_SENSORHUB18_REG  		0X52
#define LSM6DSL_FUNC_SRC1  					0X53
#define LSM6DSL_FUNC_SRC2  					0X54
#define LSM6DSL_TAP_CFG1  						0X58
#define LSM6DSL_TAP_THS_6D  					0X59
#define LSM6DSL_INT_DUR2  						0X5A
#define LSM6DSL_WAKE_UP_THS  				0X5B
#define LSM6DSL_WAKE_UP_DUR  				0X5C
#define LSM6DSL_FREE_FALL  					0X5D
#define LSM6DSL_MD1_CFG  						0X5E
#define LSM6DSL_MD2_CFG  						0X5F

/************** Access Device RAM  *******************/
#define LSM6DSL_MASTER_CMD_CODE  						0X60
#define LSM6DSL_SENS_SYNC_SPI_ERROR_CODE  		0X61

/************** Embedded functions register mapping  *******************/
#define LSM6DSL_OUT_MAG_RAW_X_L              0x66
#define LSM6DSL_OUT_MAG_RAW_X_H              0x67
#define LSM6DSL_OUT_MAG_RAW_Y_L              0x68
#define LSM6DSL_OUT_MAG_RAW_Y_H              0x69
#define LSM6DSL_OUT_MAG_RAW_Z_L              0x6A
#define LSM6DSL_OUT_MAG_RAW_Z_H              0x6B
#define LSM6DSL_X_OFS_USR                  	0x73
#define LSM6DSL_Y_OFS_USR                   	0x74
#define LSM6DSL_Z_OFS_USR                   	0x75

//////////////////////////////////////////////////////////////////////////////////////////////////////////


#endif /* __LSM6DSL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
