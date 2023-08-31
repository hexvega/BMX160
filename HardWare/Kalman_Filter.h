#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__
#include "main.h"

//1. 结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
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

