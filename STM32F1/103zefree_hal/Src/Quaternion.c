/*
1�������������
2���ߣ����У����ӣ�����
3��Ԫ�������õ���һ���㷨�����ж�����������ȫ�ף�����Խ�߾���Խ�ã�����û����Ҫ��
    һ�׶�����Щ���㷨�����ü򵥵�ֵȡ����һЩ���Ǻ�������
4��ϸ�鼮���Կ�������ʽ���Ե���ԭ����Ԭ����������ǰ�����Ȿ��PDF��þ��ˣ�����һ�뻹û�ܳ�͸��
5�������ü��ٶȼ���������̬�����ò�ƶ������ɻ��ģ��ɻ��ϵļ��ٶȼƵ�����������Ķ������̫��
Ӧ�ò���ֱ����������һ�ף���һ���������������Ȩ�ط������������ɣ�������������̬�͸�����˵��~
*/
#include "quaternion.h"
#include <math.h>



static Quaternion Normalize(Quaternion e)      //��Ԫ����һ��
{
    Quaternion_Type s = (Quaternion_Type)sqrt(e.q0 * e.q0 + e.q1 * e.q1 + e.q2 * e.q2 + e.q3 * e.q3);
    if(s>0.01) {
        e.q0 /= s;
        e.q1 /= s;
        e.q2 /= s;
        e.q3 /= s;
    }
    else {
        e.q0 =1;
        e.q1 =0;
        e.q2 =0;
        e.q3 =0;


    }
    return e;
}

static Quaternion Multiply_L1(EulerAngle smallAngle,Quaternion BQ) //һ���㷨,С�Ƕȣ�q0
{
    Quaternion Q_result;   //Roll, Pitch, Yaw
    Q_result.q0 = BQ.q0 - BQ.q1 * smallAngle.Roll / 2 - BQ.q2 * smallAngle.Pitch/ 2 - BQ.q3 * smallAngle.Yaw/ 2;
    Q_result.q1 = BQ.q1 + BQ.q0 * smallAngle.Roll / 2 + BQ.q2 * smallAngle.Yaw / 2 - BQ.q3 * smallAngle.Pitch/ 2;
    Q_result.q2 = BQ.q2 + BQ.q0 * smallAngle.Pitch/ 2 - BQ.q1 * smallAngle.Yaw/ 2 + BQ.q3 * smallAngle.Roll/ 2;
    Q_result.q3 = BQ.q3 + BQ.q0 * smallAngle.Yaw/  2  + BQ.q1 * smallAngle.Pitch/ 2 - BQ.q2 * smallAngle.Roll/ 2;
    Q_result =  Normalize(Q_result);
    return Q_result;
}

Euler_Martix Q_to_EM(Quaternion e)       //����Ԫ���任��ŷ���ǣ���̬������T
{
    Euler_Martix result;
    Euler_Martix_Type q00,q01,q02,q03,q11,q12,q13,q22,q23,q33;
    q00=e.q0*e.q0;
    q01=e.q0*e.q1;
    q02=e.q0*e.q2;
    q03=e.q0*e.q3;
    q11=e.q1*e.q1;
    q12=e.q1*e.q2;
    q13=e.q1*e.q3;
    q22=e.q2*e.q2;
    q23=e.q2*e.q3;
    q33=e.q3*e.q3;
    result.T11=q00+q11-q22-q33;
    result.T12=2*(q12+q03);
    result.T13=2*(q13-q02);
    result.T21=2*(q12-q03);
    result.T22=q22-q33+q00-q11;
    result.T23=2*(q23+q01);
    result.T31=2*(q13+q02);
    result.T32=2*(q23-q01);
    result.T33=q33-q22-q11+q00;
    return result;
}

Quaternion Ea_to_Qu(EulerAngle ea)      //��ŷ���Ǳ任����Ԫ��      ����������������ˣ��þ����Ǹ���
{
    Quaternion result;

    Quaternion_Type CosY = cos(ea.Yaw /2.0);
    Quaternion_Type SinY = sin(ea.Yaw /2.0);
    Quaternion_Type CosP = cos(ea.Pitch/2.0);
    Quaternion_Type SinP = sin(ea.Pitch /2.0);
    Quaternion_Type CosR = cos(ea.Roll /2.0);
    Quaternion_Type SinR = sin(ea.Roll /2.0);

    result.q0 = CosY * CosP * CosR + SinY * SinP * SinR;
    result.q1 = CosY * CosP * SinR - SinY * SinP * CosR;
    result.q2 = CosY * SinP * CosR + SinY * CosP * SinR;
    result.q3 = SinY * CosP * CosR - CosY * SinP * SinR;
    return result;
}

Acc coordinate_body_to_inertia(Euler_Martix EM,Acc lacc)        //����������ٶȱ任����������
{
//���ɻ�����Ҫ��ʡ��
    return lacc;
}

EulerAngle EM_to_EU(Euler_Martix lem)        //����̬��������ȡ��̬��
{
    EulerAngle result;
    result.Yaw = atan2(lem.T12, lem.T11);
    result.Pitch = -asin(lem.T13);
    result.Roll = atan2(lem.T23, lem.T33);
    return result;
}
Quaternion Quaternion_multi (Quaternion Qu1,Quaternion Qu2) //��Ԫ���˷�Qu 1*Qu 2
{
    Quaternion Qu3;
    Qu3.q0 = Qu1.q0*Qu2.q0 -Qu1.q1*Qu2.q1 -Qu1.q2*Qu2.q2 -Qu1.q3*Qu2.q3;
    Qu3.q1 = Qu1.q0*Qu2.q1 +Qu1.q1*Qu2.q0 +Qu1.q2*Qu2.q3 -Qu1.q3*Qu2.q2;
    Qu3.q2 = Qu1.q0*Qu2.q2 -Qu1.q1*Qu2.q3 +Qu1.q2*Qu2.q0 +Qu1.q3*Qu2.q1;
    Qu3.q3 = Qu1.q0*Qu2.q3 +Qu1.q1*Qu2.q2 -Qu1.q2*Qu2.q1 +Qu1.q3*Qu2.q0;
    return Qu3;
}
Quaternion Quaternion_sub (Quaternion Qu1,Quaternion Qu2) //��Ԫ������qu1-qu2
{
    Quaternion Qu3;
    Qu3.q0 = Qu1.q0-Qu2.q0  ;
    Qu3.q1 = Qu1.q1-Qu2.q1 ;
    Qu3.q2 = Qu1.q2-Qu2.q2  ;
    Qu3.q3 = Qu1.q3-Qu2.q3 ;
    return Qu3;
}

static Quaternion Quaternion_inverse(Quaternion Qu2)//��Ԫ������
{
    Quaternion Qu3,Qu1;
    Qu1=Normalize(Qu2);
    Qu3.q0 =  Qu1.q0 ;
    Qu3.q1 = -Qu1.q1 ;
    Qu3.q2 = -Qu1.q2 ;
    Qu3.q3 = -Qu1.q3 ;
    return Qu3;
}
Quaternion Quaternion_trans_positive(Quaternion Qu1,Quaternion Qu2) //��Ԫ������任-���任��qu1Ϊ�任��qu2����
{
    Quaternion Qu3;
    Qu3=Quaternion_multi(Quaternion_multi(Quaternion_inverse(Qu1),Qu2),(Qu1));
    return Qu3;
}

Quaternion Quaternion_trans_negative(Quaternion Qu1,Quaternion Qu2) //��Ԫ������任-��任��qu1Ϊ�任��qu2����
{
    Quaternion Qu3;
    Qu3=Quaternion_multi(Quaternion_multi(Qu1,Qu2),Quaternion_inverse(Qu1));
    return Qu3;
}

Quaternion q0toq1(Quaternion Qu0,float *wSpeed, float TDelta)//����q0�������ǽ��ٶȣ�����ʱ��
{
    Quaternion Qu3;
    EulerAngle smallAngle;
    smallAngle.Roll=*(wSpeed+0)*TDelta*DEG_TO_RAD;
    smallAngle.Pitch=*(wSpeed+1)*TDelta*DEG_TO_RAD;
    smallAngle.Yaw=*(wSpeed+2)*TDelta*DEG_TO_RAD;


    Qu3=Multiply_L1(smallAngle,Qu0);
    return Qu3;
}
EulerAngle QuaternionToEuler(Quaternion q) // Z-Y-X Euler angles
{
    EulerAngle euler;



    euler.Yaw =atan2( 2*(q.q0*q.q1+q.q2*q.q3), 1-2*(q.q1*q.q1+q.q2*q.q2) );
    euler.Pitch=asin(2*(q.q0*q.q2-q.q3*q.q1) );
    euler.Roll=atan2(2*(q.q0*q.q3+q.q1*q.q2),1-2*(q.q2*q.q2+q.q3*q.q3) );



    return euler;
}

