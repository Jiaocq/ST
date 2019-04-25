#ifndef __IMUCACULATOR_H__
#define __IMUCACULATOR_H__

#include "imucaculator.h"


#include "drv_lsm6dsl.h"
#define USER_USING_LSM6DSL
#define TEMPHUSTURY 50//温度存储数量
#define GRAVITYHISTYRY 250//重力校准值
//#define GRAVITY (9.80665)
#define AXISSCALOUT (2000.0/BLUEOUTFREQUENCY)//keyi可以保证修改反馈率的时候指针移动速度不变
#define MOVESPEED 0.5//0.5m/s视为正常速度

#define GRAVITYDEVIATION 0.30f//手持静止状态，允许重力的偏差
#define GYOSTATIC0  (20.0 * LSM6DSL_GYO_SCALE / 32768.0)//桌面静止，可以进入休眠状态
#define GYOSTATIC1  (60.0 * LSM6DSL_GYO_SCALE / 32768.0)//手持时刻静止一级
#define GYOSTATIC2  (100.0* LSM6DSL_GYO_SCALE / 32768.0)//手持静止，微微手抖，此状态可以判断加速度数值是否为重力加速度，若是，则可以进行对角度状态的校准
#define GYOSTATIC3  (200.0* LSM6DSL_GYO_SCALE / 32768.0)//运动后静止，此状态可以停止积分，并清空积分寄存器
#define IMU_ROOM_TEMP		25.0f//作为温度计算的标准

typedef struct
{
//      utilFilter_t tempFilter;
    float rawTemp;//温度，单位度
    float rawGyo[3];//
    float rawAcc[3];//标定后

    float temp;
    float gyo[3];//计算值
    float acc[3];//maybe计算值

    int16_t TempHistory[TEMPHUSTURY];//存储温度数据
    int16_t TempCounter;
    int16_t Temp;
    int32_t TempSum;

//    int16_t GravityHistory[GRAVITYHISTYRY];
    int16_t GravityCounter;//zhongli重力校准的指标
    float Gravity ;
    float GravitySum;
    float DeltaAcc;


    int16_t * temp_t;
    int16_t * gyo_t ;
    int16_t * acc_t ;//原始数据
    int32_t * time_t ;
    int16_t * xaxisspeed_t;//蓝牙输出数据x
    int16_t * yaxisspeed_t;//蓝牙输出数据y
    float xIntegrade;//控制蓝牙输出数据x
    float yIntegrade;//控制蓝牙输出数据y
    float zIntegrade;
    float TimeDifference;//单位秒
    float dRateRawGyo[3];//未知
    float gyoOffset[3];//未知
    uint32_t lastUpdate;

} lsm6dslStruct_t;
typedef struct {
    float temp;
    float dTemp, dTemp2, dTemp3;
    volatile uint32_t lastUpdate;
    //int alarm1Parameter;
    uint16_t nextPeriod;
    uint8_t calibReadWriteFlag;		// 0=no request, 1=read request, 2=write request
    uint8_t sensorsEnabled;


} dImuStruct_t;

typedef struct {
    volatile uint32_t *lastUpdate;
    uint8_t *magEnabled;

    float sinRot, cosRot;

    uint32_t fullUpdates;
    uint32_t halfUpdates;


} imuStruct_t ;
//#ifndef M_PI
//#define M_PI			3.14159265f
//#define M_PI_2			(M_PI / 2.0f)
//#endif

//#define RAD_TO_DEG		(180.0f / M_PI)
//#define DEG_TO_RAD		(M_PI / 180.0f)

#define GRAVITY			9.80665f	// m/s^2

enum configParameters {
    IMU_ROT=0,
    IMU_FLIP,
    IMU_ACC_BIAS_X,
    IMU_ACC_BIAS_Y,
    IMU_ACC_BIAS_Z,
    IMU_ACC_BIAS1_X,
    IMU_ACC_BIAS1_Y,
    IMU_ACC_BIAS1_Z,
    IMU_ACC_BIAS2_X,
    IMU_ACC_BIAS2_Y,
    IMU_ACC_BIAS2_Z,
    IMU_ACC_BIAS3_X,
    IMU_ACC_BIAS3_Y,
    IMU_ACC_BIAS3_Z,
    IMU_ACC_SCAL_X,
    IMU_ACC_SCAL_Y,
    IMU_ACC_SCAL_Z,
    IMU_ACC_SCAL1_X,
    IMU_ACC_SCAL1_Y,
    IMU_ACC_SCAL1_Z,
    IMU_ACC_SCAL2_X,
    IMU_ACC_SCAL2_Y,
    IMU_ACC_SCAL2_Z,
    IMU_ACC_SCAL3_X,
    IMU_ACC_SCAL3_Y,
    IMU_ACC_SCAL3_Z,
    IMU_ACC_ALGN_XY,
    IMU_ACC_ALGN_XZ,
    IMU_ACC_ALGN_YX,
    IMU_ACC_ALGN_YZ,
    IMU_ACC_ALGN_ZX,
    IMU_ACC_ALGN_ZY,
    IMU_MAG_BIAS_X,
    IMU_MAG_BIAS_Y,
    IMU_MAG_BIAS_Z,
    IMU_MAG_BIAS1_X,
    IMU_MAG_BIAS1_Y,
    IMU_MAG_BIAS1_Z,
    IMU_MAG_BIAS2_X,
    IMU_MAG_BIAS2_Y,
    IMU_MAG_BIAS2_Z,
    IMU_MAG_BIAS3_X,
    IMU_MAG_BIAS3_Y,
    IMU_MAG_BIAS3_Z,
    IMU_MAG_SCAL_X,
    IMU_MAG_SCAL_Y,
    IMU_MAG_SCAL_Z,
    IMU_MAG_SCAL1_X,
    IMU_MAG_SCAL1_Y,
    IMU_MAG_SCAL1_Z,
    IMU_MAG_SCAL2_X,
    IMU_MAG_SCAL2_Y,
    IMU_MAG_SCAL2_Z,
    IMU_MAG_SCAL3_X,
    IMU_MAG_SCAL3_Y,
    IMU_MAG_SCAL3_Z,
    IMU_MAG_ALGN_XY,
    IMU_MAG_ALGN_XZ,
    IMU_MAG_ALGN_YX,
    IMU_MAG_ALGN_YZ,
    IMU_MAG_ALGN_ZX,
    IMU_MAG_ALGN_ZY,
    IMU_GYO_BIAS_X,
    IMU_GYO_BIAS_Y,
    IMU_GYO_BIAS_Z,
    IMU_GYO_BIAS1_X,
    IMU_GYO_BIAS1_Y,
    IMU_GYO_BIAS1_Z,
    IMU_GYO_BIAS2_X,
    IMU_GYO_BIAS2_Y,
    IMU_GYO_BIAS2_Z,
    IMU_GYO_BIAS3_X,
    IMU_GYO_BIAS3_Y,
    IMU_GYO_BIAS3_Z,
    IMU_GYO_SCAL_X,
    IMU_GYO_SCAL_Y,
    IMU_GYO_SCAL_Z,
    IMU_GYO_ALGN_XY,
    IMU_GYO_ALGN_XZ,
    IMU_GYO_ALGN_YX,
    IMU_GYO_ALGN_YZ,
    IMU_GYO_ALGN_ZX,
    IMU_GYO_ALGN_ZY,
    IMU_MAG_INCL,
    IMU_MAG_DECL,
    IMU_PRESS_SENSE,

    CONFIG_NUM_PARAMS
};

int CaculatorLSM6DSL(uint8_t* in, int8_t * out);
float calc_dtemp(int16_t raw_tempture);
int32_t  CalcTempDiff(void);

#endif


