#ifndef __Madgwick_BMX160_h__
#define __Madgwick_BMX160_h__
#include <math.h>
#include <stdint.h>

typedef struct
{
    float pitch;
    float yaw;
    float roll;
}euler_angle_3d;

extern euler_angle_3d euler_angle;

extern float invSqrt(float x);
extern void imu_solution_9axis(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float delta);
extern void imu_solution_6axis(float gx, float gy, float gz, float ax, float ay, float az, float delta);

#endif
