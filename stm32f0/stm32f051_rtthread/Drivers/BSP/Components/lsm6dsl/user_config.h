#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "user_config.h"


#define RTT_SAMPLE_FREQUENCY (400)//imu�ɼ�������
#define BLUEOUTFREQUENCY (200)//����������

//#define USER_USING_INTERGRADE_STATIC//��̬����ϵ����
#define USER_USING_INTERGRADE_DYNAMIC//��̬����ϵ����
//#define USER_USING_TIMER_SOFT//ʹ�������ʱ��

#ifndef RT_USING_SPI1
#define RT_USING_SPI1
#endif
#ifndef RT_USING_SPI2
#define RT_USING_SPI2
#endif

/*����ʹ�����*/

//#define USER_USING_TIM6
#define USER_USING_TIM7
//#define USER_USING_MPU6050
#define USER_USING_LSM6DSL




#endif
