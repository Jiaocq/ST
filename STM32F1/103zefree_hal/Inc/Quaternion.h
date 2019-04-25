#ifndef __QUATERNION_H
#define __QUATERNION_H
#include "quaternion.h"
//#include "user_config.h"
//#include "imucaculator.h"
//#include <rthw.h>
//#include <rtthread.h>

#define EulerAngle_Type float   //定义类型   
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
typedef struct        //欧拉角结构体
{
    EulerAngle_Type Roll, Pitch, Yaw;
}  EulerAngle ;
typedef struct     //四元数结构体
{
    Quaternion_Type q0, q1, q2, q3;
}  Quaternion    ;
typedef struct       //加速度值结构体
{
    Acc_Type x, y, z;
}   Acc;
typedef struct      //陀螺仪值结构体
{
    Gyro_Type x, y, z;
}   Gyro;
typedef struct      //欧拉（姿态）矩阵结构体
{
    Euler_Martix_Type T11,T12,T13,  T21,T22,T23,    T31,T32,T33;
}   Euler_Martix;

typedef struct      //最终结构体
{
    Quaternion q0;//A0-B0
    Quaternion q1;//A0-B1
    Quaternion StandardGravity;//标准系的重力对应四元数
    Quaternion ThisGravity;//当前戏重力对应四元数
    Quaternion ThisAcc;//当前系加速度对应四元数
    Quaternion StandardAcc;//标准系坐标acc对应4元数
    Quaternion DeltaAcc;//加速度增量，可直接用于积分
    EulerAngle EaAngle;//当前系对当前标准坐标的欧拉角

}   IMUcalculator;
Quaternion Ea_to_Qu(EulerAngle ea) ;
Quaternion q0toq1(Quaternion Qu0,float *wSpeed, float TDelta);
Quaternion Quaternion_trans_positive(Quaternion Qu1,Quaternion Qu2) ;
Quaternion Quaternion_trans_negative(Quaternion Qu1,Quaternion Qu2) ;
Quaternion Quaternion_sub (Quaternion Qu1,Quaternion Qu2) ;
EulerAngle QuaternionToEuler(Quaternion q) ;
Quaternion Quaternion_multi (Quaternion Qu1,Quaternion Qu2) ;



#endif

