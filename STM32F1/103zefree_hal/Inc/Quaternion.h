#ifndef __QUATERNION_H
#define __QUATERNION_H
#include "quaternion.h"
//#include "user_config.h"
//#include "imucaculator.h"
//#include <rthw.h>
//#include <rtthread.h>

#define EulerAngle_Type float   //��������   
#define Quaternion_Type float
#define Acc_Type float
#define Gyro_Type float
#define Euler_Martix_Type float
	
#define RAD_TO_DEG		(180.0f / M_PI)
#define DEG_TO_RAD		(M_PI / 180.0f)
#ifndef M_PI
#define M_PI			3.14159265f
#define M_PI_2			(M_PI / 2.0f)
#endif
typedef struct        //ŷ���ǽṹ��
{
    EulerAngle_Type Roll, Pitch, Yaw;
}  EulerAngle ;
typedef struct     //��Ԫ���ṹ��
{
    Quaternion_Type q0, q1, q2, q3;
}  Quaternion    ;
typedef struct       //���ٶ�ֵ�ṹ��
{
    Acc_Type x, y, z;
}   Acc;
typedef struct      //������ֵ�ṹ��
{
    Gyro_Type x, y, z;
}   Gyro;
typedef struct      //ŷ������̬������ṹ��
{
    Euler_Martix_Type T11,T12,T13,  T21,T22,T23,    T31,T32,T33;
}   Euler_Martix;

typedef struct      //���սṹ��
{
    Quaternion q0;//A0-B0
    Quaternion q1;//A0-B1
    Quaternion StandardGravity;//��׼ϵ��������Ӧ��Ԫ��
    Quaternion ThisGravity;//��ǰϷ������Ӧ��Ԫ��
    Quaternion ThisAcc;//��ǰϵ���ٶȶ�Ӧ��Ԫ��
    Quaternion StandardAcc;//��׼ϵ����acc��Ӧ4Ԫ��
    Quaternion DeltaAcc;//���ٶ���������ֱ�����ڻ���
    EulerAngle EaAngle;//��ǰϵ�Ե�ǰ��׼�����ŷ����

}   IMUcalculator;
Quaternion Ea_to_Qu(EulerAngle ea) ;
Quaternion q0toq1(Quaternion Qu0,float *wSpeed, float TDelta);
Quaternion Quaternion_trans_positive(Quaternion Qu1,Quaternion Qu2) ;
Quaternion Quaternion_trans_negative(Quaternion Qu1,Quaternion Qu2) ;
Quaternion Quaternion_sub (Quaternion Qu1,Quaternion Qu2) ;
EulerAngle QuaternionToEuler(Quaternion q) ;
Quaternion Quaternion_multi (Quaternion Qu1,Quaternion Qu2) ;



#endif

