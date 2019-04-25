#ifndef __IMUCACULATOR_H__
#define __IMUCACULATOR_H__

#include "imucaculator.h"


#include "drv_lsm6dsl.h"
#define USER_USING_LSM6DSL
#define TEMPHUSTURY 50//�¶ȴ洢����
#define GRAVITYHISTYRY 250//����У׼ֵ
//#define GRAVITY (9.80665)
#define AXISSCALOUT (2000.0/BLUEOUTFREQUENCY)//keyi���Ա�֤�޸ķ����ʵ�ʱ��ָ���ƶ��ٶȲ���
#define MOVESPEED 0.5//0.5m/s��Ϊ�����ٶ�

#define GRAVITYDEVIATION 0.30f//�ֳ־�ֹ״̬������������ƫ��
#define GYOSTATIC0  (20.0 * LSM6DSL_GYO_SCALE / 32768.0)//���澲ֹ�����Խ�������״̬
#define GYOSTATIC1  (60.0 * LSM6DSL_GYO_SCALE / 32768.0)//�ֳ�ʱ�̾�ֹһ��
#define GYOSTATIC2  (100.0* LSM6DSL_GYO_SCALE / 32768.0)//�ֳ־�ֹ��΢΢�ֶ�����״̬�����жϼ��ٶ���ֵ�Ƿ�Ϊ�������ٶȣ����ǣ�����Խ��жԽǶ�״̬��У׼
#define GYOSTATIC3  (200.0* LSM6DSL_GYO_SCALE / 32768.0)//�˶���ֹ����״̬����ֹͣ���֣�����ջ��ּĴ���
#define IMU_ROOM_TEMP		25.0f//��Ϊ�¶ȼ���ı�׼

typedef struct
{
//      utilFilter_t tempFilter;
    float rawTemp;//�¶ȣ���λ��
    float rawGyo[3];//
    float rawAcc[3];//�궨��

    float temp;
    float gyo[3];//����ֵ
    float acc[3];//maybe����ֵ

    int16_t TempHistory[TEMPHUSTURY];//�洢�¶�����
    int16_t TempCounter;
    int16_t Temp;
    int32_t TempSum;

//    int16_t GravityHistory[GRAVITYHISTYRY];
    int16_t GravityCounter;//zhongli����У׼��ָ��
    float Gravity ;
    float GravitySum;
    float DeltaAcc;


    int16_t * temp_t;
    int16_t * gyo_t ;
    int16_t * acc_t ;//ԭʼ����
    int32_t * time_t ;
    int16_t * xaxisspeed_t;//�����������x
    int16_t * yaxisspeed_t;//�����������y
    float xIntegrade;//���������������x
    float yIntegrade;//���������������y
    float zIntegrade;
    float TimeDifference;//��λ��
    float dRateRawGyo[3];//δ֪
    float gyoOffset[3];//δ֪
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


