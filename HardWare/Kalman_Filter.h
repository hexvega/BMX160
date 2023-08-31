#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__
#include "main.h"

//1. �ṹ�����Ͷ���
typedef struct 
{
    float LastP;//�ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P;//��ǰ����Э���� ��ʼ��ֵΪ0
    float out;//�������˲������ ��ʼ��ֵΪ0
    float Kg;//���������� ��ʼ��ֵΪ0
    float Q;//��������Э���� ��ʼ��ֵΪ0.001
    float R;//�۲�����Э���� ��ʼ��ֵΪ0.543
}KFP;//Kalman Filter parameter

extern KFP KFP_accel_x;
extern KFP KFP_gyro_x;
extern KFP KFP_mag_x;

extern KFP KFP_accel_y;
extern KFP KFP_gyro_y;
extern KFP KFP_mag_y;

extern KFP KFP_accel_z;
extern KFP KFP_gyro_z;
extern KFP KFP_mag_z;

extern KFP KFP_pitch;
extern KFP KFP_yaw;
extern KFP KFP_roll;

extern float kalmanFilter(KFP *kfp,float input);


#endif

