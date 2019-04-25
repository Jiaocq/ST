#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "user_config.h"


#define RTT_SAMPLE_FREQUENCY (400)//imu采集数据率
#define BLUEOUTFREQUENCY (200)//蓝牙反馈率

//#define USER_USING_INTERGRADE_STATIC//静态坐标系积分
#define USER_USING_INTERGRADE_DYNAMIC//动态坐标系积分
//#define USER_USING_TIMER_SOFT//使用软件定时器

#ifndef RT_USING_SPI1
#define RT_USING_SPI1
#endif
#ifndef RT_USING_SPI2
#define RT_USING_SPI2
#endif

/*外设使用情况*/

//#define USER_USING_TIM6
#define USER_USING_TIM7
//#define USER_USING_MPU6050
#define USER_USING_LSM6DSL




#endif
