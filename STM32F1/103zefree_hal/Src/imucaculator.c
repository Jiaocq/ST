
#include "imucaculator.h"
#include "drv_lsm6dsl.h"

#include <math.h>
#include <stdio.h>
#include "quaternion.h"

static char debugfloatIMU = HAL_OK;

//static uint32_t  numfp = 0;

static lsm6dslStruct_t lsm6dslData = {0};
static float StandardGravity = GRAVITY; //��׼����9.8��
//static char str[200];
static float p[CONFIG_NUM_PARAMS] = {0};
static  dImuStruct_t dImuData;
static imuStruct_t imuData;
static uint8_t runstate = 0;
static IMUcalculator IMU_calc = {0};


int init_imuscale_data(void)
{
    float a = 0.0, b = 0.0, c = 0.0, d = 0.0;
    float	accscale = 1.0f / ((1<<16) / (LSM6DSL_ACC_SCALE * 2.0f))   * GRAVITY;
    p[IMU_ACC_SCAL_X] = 1.0f*9.88071/GRAVITY*0.99524;
    p[IMU_ACC_SCAL_Y] = 1.0f*10.1100/GRAVITY*0.9827;
    p[IMU_ACC_SCAL_Z] = 1.0f*9.94298/GRAVITY*0.9932;

    p[IMU_GYO_SCAL_X] = 1.0f;
    p[IMU_GYO_SCAL_Y] = 1.0f;
    p[IMU_GYO_SCAL_Z] = 1.0f;
    a = 0.0, b = 0.0, c = 0.0, d = 0.0;
    p[IMU_GYO_BIAS_X] = -a;
    p[IMU_GYO_BIAS1_X] = -b;
    p[IMU_GYO_BIAS2_X] = -c;
    p[IMU_GYO_BIAS3_X] = -d;
    a = 0.0, b = 0.0, c = 0.0, d = 0.0;
    p[IMU_GYO_BIAS_Y] = -a;
    p[IMU_GYO_BIAS1_Y] = -b;
    p[IMU_GYO_BIAS2_Y] =  -c;
    p[IMU_GYO_BIAS3_Y] = -d;
    a = 0.0, b = 0.0, c = 0.0, d = 0.0;
    p[IMU_GYO_BIAS_Z] = -a;
    p[IMU_GYO_BIAS1_Z] = -b;
    p[IMU_GYO_BIAS2_Z] = -c;
    p[IMU_GYO_BIAS3_Z] = -d;
    a = -85.7*accscale, b = 0.0, c = 0.0, d = 0.0;
    p[IMU_ACC_BIAS_X] = -a;
    p[IMU_ACC_BIAS1_X] = -b;
    p[IMU_ACC_BIAS2_X] = -c;
    p[IMU_ACC_BIAS3_X] = -d;
    a = -34.82*accscale, b = 0.0, c = 0.0, d = 0.0;
    p[IMU_ACC_BIAS_Y] = -a;
    p[IMU_ACC_BIAS1_Y] = -b;
    p[IMU_ACC_BIAS2_Y] = -c;
    p[IMU_ACC_BIAS3_Y] = -d;
//a=0.5483+0.383-0.26+0.07,b=-0.002766,c=-0.01821,d=0.0;
    a = 144.6*accscale, b = 0.0, c = 0.0, d = 0.0;
    p[IMU_ACC_BIAS_Z] = -a;
    p[IMU_ACC_BIAS1_Z] = -b;
    p[IMU_ACC_BIAS2_Z] = -c;
    p[IMU_ACC_BIAS3_Z] = -d;
    return 0;

}
void imuCalcRot(void)
{
    float rotAngle;

    rotAngle = p[IMU_ROT] * DEG_TO_RAD;

    imuData.sinRot = sin(rotAngle);
    imuData.cosRot = cos(rotAngle);
}


static void lsm6dslCalibAcc(float *in, volatile float *out)
{
    float a, b, c;
    float x, y, z;

    // bias
    a = (in[0] + p[IMU_ACC_BIAS_X] + p[IMU_ACC_BIAS1_X] * dImuData.dTemp + p[
             IMU_ACC_BIAS2_X] * dImuData.dTemp2 + p[IMU_ACC_BIAS3_X] * dImuData.dTemp3);
    b = (in[1] + p[IMU_ACC_BIAS_Y] + p[IMU_ACC_BIAS1_Y] * dImuData.dTemp + p[
             IMU_ACC_BIAS2_Y] * dImuData.dTemp2 + p[IMU_ACC_BIAS3_X] * dImuData.dTemp3);
    c = (in[2] + p[IMU_ACC_BIAS_Z] + p[IMU_ACC_BIAS1_Z] * dImuData.dTemp + p[
             IMU_ACC_BIAS2_Z] * dImuData.dTemp2 + p[IMU_ACC_BIAS3_X] * dImuData.dTemp3);

    // misalignment
    x = a + b * p[IMU_ACC_ALGN_XY] + c * p[IMU_ACC_ALGN_XZ];
    y = a * p[IMU_ACC_ALGN_YX] + b + c * p[IMU_ACC_ALGN_YZ];
    z = a * p[IMU_ACC_ALGN_ZX] + b * p[IMU_ACC_ALGN_ZY] + c;

    // scale
    x /= (p[IMU_ACC_SCAL_X] + p[IMU_ACC_SCAL1_X] * dImuData.dTemp + p[
              IMU_ACC_SCAL2_X] * dImuData.dTemp2 + p[IMU_ACC_SCAL3_X] * dImuData.dTemp3);
    y /= (p[IMU_ACC_SCAL_Y] + p[IMU_ACC_SCAL1_Y] * dImuData.dTemp + p[
              IMU_ACC_SCAL2_Y] * dImuData.dTemp2 + p[IMU_ACC_SCAL3_Y] * dImuData.dTemp3);
    z /= (p[IMU_ACC_SCAL_Z] + p[IMU_ACC_SCAL1_Z] * dImuData.dTemp + p[
              IMU_ACC_SCAL2_Z] * dImuData.dTemp2 + p[IMU_ACC_SCAL3_Z] * dImuData.dTemp3);

    // IMU rotation
    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

static void lsm6dslCalibGyo(float *in, volatile float *out)
{
    float a, b, c;
    float x, y, z;

    // bias
    a = (in[0] + lsm6dslData.gyoOffset[0] + p[IMU_GYO_BIAS_X] + p[
             IMU_GYO_BIAS1_X] * dImuData.dTemp + p[IMU_GYO_BIAS2_X] * dImuData.dTemp2 + p[
             IMU_GYO_BIAS3_X] * dImuData.dTemp3);
    b = (in[1] + lsm6dslData.gyoOffset[1] + p[IMU_GYO_BIAS_Y] + p[
             IMU_GYO_BIAS1_Y] * dImuData.dTemp + p[IMU_GYO_BIAS2_Y] * dImuData.dTemp2 + p[
             IMU_GYO_BIAS3_Y] * dImuData.dTemp3);
    c = (in[2] + lsm6dslData.gyoOffset[2] + p[IMU_GYO_BIAS_Z] + p[
             IMU_GYO_BIAS1_Z] * dImuData.dTemp + p[IMU_GYO_BIAS2_Z] * dImuData.dTemp2 + p[
             IMU_GYO_BIAS3_Z] * dImuData.dTemp3);

    // misalignment
    x = a + b * p[IMU_GYO_ALGN_XY] + c * p[IMU_GYO_ALGN_XZ];
    y = a * p[IMU_GYO_ALGN_YX] + b + c * p[IMU_GYO_ALGN_YZ];
    z = a * p[IMU_GYO_ALGN_ZX] + b * p[IMU_GYO_ALGN_ZY] + c;

    // scale
    x /= p[IMU_GYO_SCAL_X];
    y /= p[IMU_GYO_SCAL_Y];
    z /= p[IMU_GYO_SCAL_Z];

    // IMU rotation
    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

//�ж���һ����Χ��
static int16_t IsInARange(float compare, float compared)
{
    if( compare >= compared)return HAL_ERROR;
    else if(compare < -compared)return HAL_ERROR;
    return HAL_OK;
}
//�ж���һ����Χ��
static int16_t IsOutARange(float compare, float compared)
{
    if( compare >= compared)return HAL_OK;
    else if(compare < -compared)return HAL_OK;
    return HAL_ERROR;
}
//У׼����
static int16_t CalibrateGravity()
{
    lsm6dslData.GravitySum = lsm6dslData.GravitySum + lsm6dslData.Gravity ;
    if(++lsm6dslData.GravityCounter >= GRAVITYHISTYRY)
    {
        StandardGravity = lsm6dslData.GravitySum / GRAVITYHISTYRY;
        lsm6dslData.GravityCounter = 0;
        lsm6dslData.GravitySum = 0;
        return HAL_OK ;
    }
    return HAL_ERROR ;

}
//���е͹��ģ��ر�����
static int16_t LowPowerMode()
{

    return HAL_OK;
}
//�ӵ͹��Ļ���
static int16_t WakeUpPowerMode()
{

    return HAL_OK;
}

//tiֹͣ�������
static int16_t StopOutput()
{



    * lsm6dslData.xaxisspeed_t = 0;
    * lsm6dslData.yaxisspeed_t = 0;

    return HAL_OK;
}
//����
static int16_t IntegrateAxis()
{
    lsm6dslData.xIntegrade = lsm6dslData.xIntegrade+IMU_calc.DeltaAcc.q1*lsm6dslData.TimeDifference*100.0;
    lsm6dslData.yIntegrade =lsm6dslData.yIntegrade+ IMU_calc.DeltaAcc.q2*lsm6dslData.TimeDifference*100.0;
    lsm6dslData.zIntegrade = lsm6dslData.zIntegrade+IMU_calc.DeltaAcc.q3*lsm6dslData.TimeDifference*100.0;

//    sprintf(str, "\t%.1f\t%.1f\t%.1f", lsm6dslData.xIntegrade,lsm6dslData.yIntegrade,lsm6dslData.zIntegrade);
//    rt_kprintf("lsm6dslData.xIntegrade = %s\r\n", str);

    return HAL_OK;
}//ֹͣ����
static int16_t StopIntegrateAxis()
{

    lsm6dslData.xIntegrade = 0;
    lsm6dslData.yIntegrade = 0;
    lsm6dslData.zIntegrade = 0;


    return HAL_OK;
}
//���ٶ���ֵ���������ٶȣ����жԽǶ�״̬��У׼
static int16_t CalibrateAngle()
{
    StopIntegrateAxis();
    IMU_calc.StandardGravity.q0 = 0;
    IMU_calc.StandardGravity.q1 = 0;
    IMU_calc.StandardGravity.q2 = 0;
    IMU_calc.StandardGravity.q3 = -lsm6dslData.Gravity;
//    StandardGravity=lsm6dslData.Gravity;
    //IMU_calc.ThisGravity={0,* (lsm6dslData.acc + 0),* (lsm6dslData.acc + 1),* (lsm6dslData.acc + 2)};
    IMU_calc.ThisGravity.q0 = 0;
    IMU_calc.ThisGravity.q1 = * (lsm6dslData.acc + 0);
    IMU_calc.ThisGravity.q2 = * (lsm6dslData.acc + 1);
    IMU_calc.ThisGravity.q3 = * (lsm6dslData.acc + 2);

    //IMU_calc.EaAngle={0,asin(IMU_calc.ThisGravity.q1/IMU_calc.StandardGravity),atan2(-IMU_calc.ThisGravity.q2,-IMU_calc.ThisGravity.q3)};
    IMU_calc.EaAngle.Yaw = 0;
    IMU_calc.EaAngle.Pitch = asin(IMU_calc.ThisGravity.q1 / lsm6dslData.Gravity);
    IMU_calc.EaAngle.Roll = atan2(-IMU_calc.ThisGravity.q2, -IMU_calc.ThisGravity.q3);
// sprintf(str, "IMU_calc.EaAngle=%2.3f\t%2.3f \r\n",IMU_calc.EaAngle.Pitch*RAD_TO_DEG,RAD_TO_DEG*IMU_calc.EaAngle.Roll);
//  rt_kprintf(str);


    IMU_calc.q0 = Ea_to_Qu(IMU_calc.EaAngle);

// sprintf(str, "IMU_calc.q0=\t%2.3f\t%2.3f\t%2.3f\t%2.3f\r\n",IMU_calc.q0.q0,IMU_calc.q0.q1,IMU_calc.q0.q2,IMU_calc.q0.q3);
//  rt_kprintf(str);
    //IMU_calc.ThisAcc={0,*(lsm6dslData.acc+0),*(lsm6dslData.acc+1),*(lsm6dslData.acc+2)};
    IMU_calc.ThisAcc.q0 = 0;
    IMU_calc.ThisAcc.q1 = *(lsm6dslData.acc + 0);
    IMU_calc.ThisAcc.q2 = *(lsm6dslData.acc + 1);
    IMU_calc.ThisAcc.q3 = *(lsm6dslData.acc + 2);

    {
//���任����ʽ���ٶȱȽ�
        //��׼ϵ���������ı任
//  IMU_calc.StandardAcc=Quaternion_trans_positive(IMU_calc.q0,IMU_calc.StandardGravity);
    }
    {
        //����ϵ����׼ϵ�ı任
        IMU_calc.StandardAcc = Quaternion_trans_negative(IMU_calc.q0, IMU_calc.ThisAcc);
    }
//   sprintf(str, "IMU_calc.StandardAcc=%2.3f\t%2.3f\t%2.3f\t%2.3f \r\n",IMU_calc.StandardAcc.q0,IMU_calc.StandardAcc.q1,IMU_calc.StandardAcc.q2,IMU_calc.StandardAcc.q3);
//  rt_kprintf(str);

    IMU_calc.DeltaAcc = Quaternion_sub(IMU_calc.StandardAcc, IMU_calc.StandardGravity);
//sprintf(str, "IMU_calc.DeltaAcc=\t%2.3f\t%2.3f\t%2.3f\t%2.3f \r\n",IMU_calc.DeltaAcc.q0,IMU_calc.DeltaAcc.q1,IMU_calc.DeltaAcc.q2,IMU_calc.DeltaAcc.q3);
//  rt_kprintf(str);
    return HAL_OK;
}
//���ֲ����
static int16_t IntegrateWithoutOutput()
{
    //����
//    IntegrateAxis();
    //tiֹͣ�������
    StopOutput();
    return HAL_OK;
}
//���������
static int16_t IntegrateAndOutput()
{
    IntegrateWithoutOutput();
    //���
//    * lsm6dslData.xaxisspeed_t = (int16_t)(lsm6dslData.xIntegrade / MOVESPEED * AXISSCALOUT);
//    * lsm6dslData.yaxisspeed_t = (int16_t)(lsm6dslData.zIntegrade / MOVESPEED * AXISSCALOUT);
    if(HAL_OK == IsOutARange(* lsm6dslData.xaxisspeed_t, 127.0))
    {
        if(* lsm6dslData.xaxisspeed_t > 0)* lsm6dslData.xaxisspeed_t = 127;
        else * lsm6dslData.xaxisspeed_t = -127;
    }
    if(HAL_OK == IsOutARange(* lsm6dslData.yaxisspeed_t, 127.0))
    {
        if(* lsm6dslData.yaxisspeed_t > 0)* lsm6dslData.yaxisspeed_t = 127;
        else * lsm6dslData.yaxisspeed_t = -127;
    }

// LevelSpeed>>lingmindu>127?127:(LevelSpeed>>lingmindu<-127?-127:LevelSpeed>>lingmindu);
//-(VerticalSpeed>>lingmindu>127?127:(VerticalSpeed>>lingmindu<-127?-127:VerticalSpeed>>lingmindu));

    return HAL_OK;
}



//static int16_t OutputState0()
//{
//  return HAL_OK;
//}
static int16_t OutputState1()
{
    return HAL_OK;
}

static int16_t OutputState2()
{
    return HAL_OK;
}

static int16_t OutputState3()
{
    return HAL_OK;
}

static int16_t OutputState4()
{
    //��������У׼
    lsm6dslData.GravityCounter = 0;
    lsm6dslData.GravitySum = 0;
    //�ӵ͹��Ļ���
    WakeUpPowerMode();

    return HAL_OK;
}
static int16_t InputState0()
{
    IntegrateAndOutput();

    OutputState1();
    OutputState2();
    OutputState3();
    OutputState4();
    return HAL_OK;
}

static int16_t InputState1()
{
    IntegrateWithoutOutput();

    OutputState2();
    OutputState3();
    OutputState4();
    return HAL_OK;
}
static int16_t InputState2()
{
    //�ж��������ٶ��Ƿ���g�����ѡ��standardgravity+-3%  �� 4096->1m/s/s,��ֹԼΪ9.8=4096���ֳ־�ֹԼΪ10.2+-0.2=4250
    if(HAL_OK == IsInARange(lsm6dslData.Gravity - StandardGravity, StandardGravity * GRAVITYDEVIATION))
    {
        //���ٶ���ֵ���������ٶȣ����ü��ٶȼƽ��жԽǶ�״̬��У׼
//        CalibrateAngle();


    }

    OutputState3();
    OutputState4();
    return HAL_OK;
}
static int16_t InputState3()
{




    //�ж��������ٶ��Ƿ���g�����ѡ��standardgravity+-3%  �� 4096->1m/s/s,��ֹԼΪ9.8=4096���ֳ־�ֹԼΪ10.2+-0.2=4250
    if(HAL_OK == IsInARange(lsm6dslData.Gravity - StandardGravity, StandardGravity * GRAVITYDEVIATION))
    {
        //���ٶ���ֵ���������ٶȣ����ü��ٶȼƽ��жԽǶ�״̬��У׼
        CalibrateAngle();
    }


    OutputState4();
    return HAL_OK;
}

static int16_t InputState4()
{



    //�ж��������ٶ��Ƿ���g�����ѡ��standardgravity+-3%  �� 4096->1m/s/s,��ֹԼΪ9.8=4096���ֳ־�ֹԼΪ10.2+-0.2=4250
    if(HAL_OK == IsInARange(lsm6dslData.Gravity - StandardGravity, StandardGravity * GRAVITYDEVIATION))
    {
        //���ٶ���ֵ���������ٶȣ����ü��ٶȼƽ��жԽǶ�״̬��У׼
        CalibrateAngle();
    }

    //У׼����
    if(HAL_OK == CalibrateGravity())
    {
        //�͹���ģʽ���ر�һЩ����
        LowPowerMode();
    }

    return HAL_OK;
}

int CaculatorLSM6DSL(uint8_t* in, int8_t * out)
{

	
    lsm6dslCalibAcc(lsm6dslData.rawAcc, lsm6dslData.acc); //ȥ����ƫ
    lsm6dslCalibGyo(lsm6dslData.rawGyo, lsm6dslData.gyo); //
    if(debugfloatIMU != HAL_OK)
    {
//                  sprintf(str, "%2.3f\t%2.3f\t%2.3f\t%2.3f\t%2.3f\t%2.3f\t%2.3f\t%ld\t\r\n",
//                (lsm6dslData.temp + 0), *(lsm6dslData.gyo + 0), *(lsm6dslData.gyo + 1), *(lsm6dslData.gyo + 2),
//                *(lsm6dslData.acc + 0), *(lsm6dslData.acc + 1), *(lsm6dslData.acc + 2),
//                numfp++);
//        rt_kprintf(str);
    }
    //��¼������ʱ��
    lsm6dslData.TimeDifference = (*lsm6dslData.time_t - lsm6dslData.lastUpdate) / 1000000.0; //��λm��
    lsm6dslData.lastUpdate = *lsm6dslData.time_t;
#ifdef USER_USING_INTERGRADE_STATIC
    //��Ԫ������
    IMU_calc.q0 = q0toq1(IMU_calc.q0, lsm6dslData.gyo, lsm6dslData.TimeDifference);
    IMU_calc.StandardGravity.q0 = 0;
    IMU_calc.StandardGravity.q1 = 0;
    IMU_calc.StandardGravity.q2 = 0;
    IMU_calc.StandardGravity.q3 = -StandardGravity;

    IMU_calc.ThisAcc.q0 = 0;
    IMU_calc.ThisAcc.q1 = *(lsm6dslData.acc + 0);
    IMU_calc.ThisAcc.q2 = *(lsm6dslData.acc + 1);
    IMU_calc.ThisAcc.q3 = *(lsm6dslData.acc + 2);
    IMU_calc.StandardAcc = Quaternion_trans_negative(IMU_calc.q0, IMU_calc.ThisAcc);
    IMU_calc.DeltaAcc = Quaternion_sub(IMU_calc.StandardAcc, IMU_calc.StandardGravity);
#endif
#ifdef USER_USING_INTERGRADE_DYNAMIC
    //��Ԫ������
    IMU_calc.q1 = q0toq1(IMU_calc.q0, lsm6dslData.gyo, lsm6dslData.TimeDifference);
    IMU_calc.StandardGravity.q0 = 0;
    IMU_calc.StandardGravity.q1 = 0;
    IMU_calc.StandardGravity.q2 = 0;
    IMU_calc.StandardGravity.q3 = -StandardGravity;
    IMU_calc.ThisGravity = Quaternion_trans_positive(IMU_calc.q1, IMU_calc.StandardGravity);
    IMU_calc.EaAngle.Yaw = 0;
    IMU_calc.EaAngle.Pitch = asin(IMU_calc.ThisGravity.q1 / StandardGravity);
    IMU_calc.EaAngle.Roll = atan2(-IMU_calc.ThisGravity.q2, -IMU_calc.ThisGravity.q3);
    IMU_calc.q0 = Ea_to_Qu(IMU_calc.EaAngle);
    IMU_calc.ThisAcc.q0 = 0;
    IMU_calc.ThisAcc.q1 = *(lsm6dslData.acc + 0);
    IMU_calc.ThisAcc.q2 = *(lsm6dslData.acc + 1);
    IMU_calc.ThisAcc.q3 = *(lsm6dslData.acc + 2);
    IMU_calc.StandardAcc = Quaternion_trans_negative(IMU_calc.q0, IMU_calc.ThisAcc);
    IMU_calc.DeltaAcc = Quaternion_sub(IMU_calc.StandardAcc, IMU_calc.StandardGravity);
#endif

    lsm6dslData.Gravity = (float)sqrt((*(lsm6dslData.acc + 0) *
                                       *(lsm6dslData.acc + 0)) + (* (lsm6dslData.acc + 1) *
                                               * (lsm6dslData.acc + 1)) + (* (lsm6dslData.acc + 2) *
                                                       * (lsm6dslData.acc + 2)));
    lsm6dslData.DeltaAcc =  lsm6dslData.Gravity-StandardGravity;
//		(float)sqrt((IMU_calc.DeltaAcc.q1 * IMU_calc.DeltaAcc.q1 ) +
//														(IMU_calc.DeltaAcc.q2* IMU_calc.DeltaAcc.q2) +
//														(IMU_calc.DeltaAcc.q3* IMU_calc.DeltaAcc.q3));
//   sprintf(str, "%3.3f\t",lsm6dslData.DeltaAcc);
//   rt_kprintf("lsm6dslData.DeltaAcc = %s\r\n", str);
    IntegrateAxis();

//   sprintf(str, "\t%2.1f\t%2.1f\t%2.1f\t", IMU_calc.DeltaAcc.q1, IMU_calc.DeltaAcc.q2, IMU_calc.DeltaAcc.q3);
//   rt_kprintf("IMU_calc.DeltaAcc = %s\r\n", str);
//    sprintf(str, "\t%2.1f\t%2.1f\t%2.1f\t", IMU_calc.StandardAcc.q1, IMU_calc.StandardAcc.q2, IMU_calc.StandardAcc.q3);
//    rt_kprintf("IMU_calc.DeltaAcc = %s\r\n", str);

    //�ж��˶� state0
    if(HAL_OK == IsOutARange(lsm6dslData.DeltaAcc,StandardGravity * 0.005)||HAL_OK == IsOutARange(*(lsm6dslData.gyo + 0),  GYOSTATIC3)
            || HAL_OK == IsOutARange(*(lsm6dslData.gyo + 1),  GYOSTATIC3)
            || HAL_OK == IsOutARange(*(lsm6dslData.gyo + 2),  GYOSTATIC3))
    {
        //za���˶�
//        //���ֲ�д����������
//        sprintf(str, "\t%.3f\t%.3f\t%.3f", lsm6dslData.xIntegrade, lsm6dslData.yIntegrade, lsm6dslData.zIntegrade);
//        rt_kprintf("Integrade = %s\r\n", str);
        InputState0();
        runstate = 0;
    }
    else
        //state1
        if(HAL_OK == IsOutARange(*(lsm6dslData.gyo + 0), GYOSTATIC2)
                || HAL_OK == IsOutARange(*(lsm6dslData.gyo + 1), GYOSTATIC2)
                || HAL_OK == IsOutARange(*(lsm6dslData.gyo + 2), GYOSTATIC2))
        {
            //�˶���ֹ�������������������ڹ�ȥ��һ��ʱ���ڶ��й�������ƣ�
            //��״̬����ֹͣ���֣�����ջ��ּĴ���
            //�����״̬
            InputState1();
            runstate = 1;
        }
        else
            //state2
            if(HAL_OK == IsOutARange(*(lsm6dslData.gyo + 0), GYOSTATIC1)
                    || HAL_OK == IsOutARange(*(lsm6dslData.gyo + 1), GYOSTATIC1)
                    || HAL_OK == IsOutARange(*(lsm6dslData.gyo + 2), GYOSTATIC1))
            {
                //�˶���ֹ�������������������ڹ�ȥ��һ��ʱ���ڶ��ڹ��㣬
                //��״̬����ֹͣ���֣�����ջ��ּĴ���
                //�ֳ־�ֹ��΢΢�ֶ�����״̬�����жϼ��ٶ���ֵ�Ƿ�Ϊ�������ٶȣ�
                //���ǣ�����Խ��жԽǶ�״̬��У׼
                InputState2();
                runstate = 2;
            }
            else
                //state3
                if(HAL_OK == IsOutARange(*(lsm6dslData.gyo + 0), GYOSTATIC0)
                        || HAL_OK == IsOutARange(*(lsm6dslData.gyo + 1), GYOSTATIC0)
                        || HAL_OK == IsOutARange(*(lsm6dslData.gyo + 2), GYOSTATIC0))
                {
                    //�˶���ֹ�������������������ڹ�ȥ��һ��ʱ���ڶ��ڹ��㣬
                    //��״̬����ֹͣ���֣�����ջ��ּĴ���
                    //�ֳ־�ֹ��΢΢�ֶ�����״̬�����жϼ��ٶ���ֵ�Ƿ�Ϊ�������ٶȣ�
                    //���ǣ�����Խ��жԽǶ�״̬��У׼
                    //�ֳ�ʱ�̾�ֹһ��
                    InputState3();
                    runstate = 3;
                }
                else
                {
                    //state 4

                    //�˶���ֹ�������������������ڹ�ȥ��һ��ʱ���ڶ��ڹ��㣬
                    //��״̬����ֹͣ���֣�����ջ��ּĴ���
                    //�ֳ־�ֹ��΢΢�ֶ�����״̬�����жϼ��ٶ���ֵ�Ƿ�Ϊ�������ٶȣ�
                    //���ǣ�����Խ��жԽǶ�״̬��У׼
                    //�ֳ�ʱ�̾�ֹһ��
                    //���澲ֹ�����Խ�������״̬,����У׼������ֵ��ѡȡ��ʷ��ʮ���������ƽ��ֵ��
                    //У׼����
                    InputState4();
                    runstate = 4;
                }
    return 0;
}




static void dIMUCalcTempDiff(void)
{
    float temp = 0.0f;
    int i = 0;
#ifdef USER_USING_LSM6DSL
    temp += lsm6dslData.temp;
    i++;
#endif
    dImuData.temp = temp / (float)i;
    dImuData.dTemp = dImuData.temp - IMU_ROOM_TEMP;
    dImuData.dTemp2 = dImuData.dTemp * dImuData.dTemp;
    dImuData.dTemp3 = dImuData.dTemp2 * dImuData.dTemp;
}
float calc_dtemp(int16_t raw_tempture)
{

        //����¶���ֵ  ���¶�ֵΪ���n�β�����ֵ
        //��Ϊ�µļ��ϣ�ȥ����õ�
        lsm6dslData.TempSum = lsm6dslData.TempSum + raw_tempture
                              - lsm6dslData.TempHistory[lsm6dslData.TempCounter];
        lsm6dslData.TempHistory[lsm6dslData.TempCounter++] = * lsm6dslData.temp_t;
        if(lsm6dslData.TempCounter >= TEMPHUSTURY)lsm6dslData.TempCounter = 0;
        lsm6dslData.Temp = lsm6dslData.TempSum / TEMPHUSTURY;
        lsm6dslData.temp = (float) lsm6dslData.Temp / 256.0 + 25.0;

        dIMUCalcTempDiff();
return lsm6dslData.temp;
}
//////////////////////////////////////////////////////
int32_t  CalcTempDiff(void)
{


    init_imuscale_data();//li��ƫ������ʼ
    imuCalcRot();//xy�Ƕȳ�ʼ��

    return 0;

}


