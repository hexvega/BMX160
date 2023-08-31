#include "Kalman_Filter.h"

//    float LastP;//上次估算协方差 初始化值为0.02
//    float Now_P;//当前估算协方差 初始化值为0
//    float out;//卡尔曼滤波器输出 初始化值为0
//    float Kg;//卡尔曼增益 初始化值为0
//    float Q;//过程噪声协方差 初始化值为0.001
//    float R;//观测噪声协方差 初始化值为0.543




/*
  Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
  R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
	
***其中p的初值可以随便取，但是不能为0（为0的话卡尔曼滤波器就认为已经是最优滤波器了）
***Q,R 的值需要我们试出来，讲白了就是(买的破温度计有多破，以及你的超人力有多强)
  Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
  R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
***Q 参数调滤波后的曲线平滑程度，Q越小越平滑。
***R 参数调整滤波后的曲线与实测曲线的相近程度，R越小越接近。
*/


//2. 定义卡尔曼结构体并初始化参数
//KFP KFP_accel={0.002,0,0,0,0.07,0.00943};
//KFP KFP_gyro={0.002,0,0,0,0.004,50.92843};
//KFP KFP_mag={0.002,0,0,0,0.04,0.003};


KFP KFP_accel_x={0.002,0,0,0,0.01,0.09};
KFP KFP_accel_y={0.002,0,0,0,0.01,0.09};
KFP KFP_accel_z={0.002,0,0,0,0.01,0.09};

KFP KFP_gyro_x={0.002,0,0,0,0.1,3.9};
KFP KFP_gyro_y={0.002,0,0,0,0.1,3.9};
KFP KFP_gyro_z={0.002,0,0,0,0.1,3.9};

KFP KFP_mag_x={0.002,0,0,0,0.06,20.4};
KFP KFP_mag_y={0.002,0,0,0,0.06,20.4};
KFP KFP_mag_z={0.002,0,0,0,0.06,20.4};

KFP KFP_pitch={0.002,0,0,0,0.07,0.8};
KFP KFP_yaw={0.002,0,0,0,0.06,0.8};
KFP KFP_roll={0.002,0,0,0,0.06,0.8};

/** *卡尔曼滤波器 *@param KFP *kfp 卡尔曼结构体参数 * float input 须要滤波的参数的测量值（即传感器的采集值） *@return 滤波后的参数（最优值） */
 float kalmanFilter(KFP *kfp,float input)
 {
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//由于这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }

///** *调用卡尔曼滤波器 实践 */
//int height;
//int kalman_height=0;
//kalman_height = kalmanFilter(&KFP_height,(float)height);