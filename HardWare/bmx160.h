#ifndef __BMX160_H__
#define __BMX160_H__
#include "main.h"

typedef struct
{
    float x;
    float y;
    float z;
}vector_3d_t;

//typedef struct
//{
//    float pitch;
//    float roll;
//    float yaw;
//}euler_angles_t;

extern vector_3d_t _accel_filter_value;
extern vector_3d_t _gyro_filter_value;
extern vector_3d_t _mag_filter_value;

//extern vector_3d_t _accel_real_time;
//extern vector_3d_t _gyro_real_time;
//extern vector_3d_t _mag_real_time;

extern vector_3d_t mag_offset;
extern vector_3d_t mag_scale;	

#define IIC_ADDR 0X68        //
#define BMX160_CHIP_ID                           0xD8
#define BMI160_PMU_STATUS      									 0X03
#define BMX160_SOFT_RESET_CMD                    0xB6
     
/** bmx160 Register map */
#define BMX160_CHIP_ID_ADDR                      0x00
#define BMX160_ERROR_REG_ADDR                    0x02
#define BMX160_MAG_DATA_ADDR                     0x04
#define BMX160_GYRO_DATA_ADDR                    0x0C
#define BMX160_ACCEL_DATA_ADDR                   0x12
#define BMX160_STATUS_ADDR                       0x1B
#define BMX160_INT_STATUS_ADDR                   0x1C
#define BMX160_FIFO_LENGTH_ADDR                  0x22
#define BMX160_FIFO_DATA_ADDR                    0x24
#define BMX160_ACCEL_CONFIG_ADDR                 0x40
#define BMX160_ACCEL_RANGE_ADDR                  0x41
#define BMX160_GYRO_CONFIG_ADDR                  0x42
#define BMX160_GYRO_RANGE_ADDR                   0x43
#define BMX160_MAGN_CONFIG_ADDR                  0x44
#define BMX160_FIFO_DOWN_ADDR                    0x45
#define BMX160_FIFO_CONFIG_0_ADDR                0x46
#define BMX160_FIFO_CONFIG_1_ADDR                0x47
#define BMX160_MAGN_RANGE_ADDR                   0x4B
#define BMX160_MAGN_IF_0_ADDR                    0x4C
#define BMX160_MAGN_IF_1_ADDR                    0x4D
#define BMX160_MAGN_IF_2_ADDR                    0x4E
#define BMX160_MAGN_IF_3_ADDR                    0x4F
#define BMX160_INT_ENABLE_0_ADDR                 0x50
#define BMX160_INT_ENABLE_1_ADDR                 0x51
#define BMX160_INT_ENABLE_2_ADDR                 0x52
#define BMX160_INT_OUT_CTRL_ADDR                 0x53
#define BMX160_INT_LATCH_ADDR                    0x54
#define BMX160_INT_MAP_0_ADDR                    0x55
#define BMX160_INT_MAP_1_ADDR                    0x56
#define BMX160_INT_MAP_2_ADDR                    0x57
#define BMX160_INT_DATA_0_ADDR                   0x58
#define BMX160_INT_DATA_1_ADDR                   0x59
#define BMX160_INT_LOWHIGH_0_ADDR                0x5A
#define BMX160_INT_LOWHIGH_1_ADDR                0x5B
#define BMX160_INT_LOWHIGH_2_ADDR                0x5C
#define BMX160_INT_LOWHIGH_3_ADDR                0x5D
#define BMX160_INT_LOWHIGH_4_ADDR                0x5E
#define BMX160_INT_MOTION_0_ADDR                 0x5F
#define BMX160_INT_MOTION_1_ADDR                 0x60
#define BMX160_INT_MOTION_2_ADDR                 0x61
#define BMX160_INT_MOTION_3_ADDR                 0x62
#define BMX160_INT_TAP_0_ADDR                    0x63
#define BMX160_INT_TAP_1_ADDR                    0x64
#define BMX160_INT_ORIENT_0_ADDR                 0x65
#define BMX160_INT_ORIENT_1_ADDR                 0x66
#define BMX160_INT_FLAT_0_ADDR                   0x67
#define BMX160_INT_FLAT_1_ADDR                   0x68
#define BMX160_FOC_CONF_ADDR                     0x69
#define BMX160_CONF_ADDR                         0x6A

#define BMX160_IF_CONF_ADDR                      0x6B
#define BMX160_SELF_TEST_ADDR                    0x6D
#define BMX160_OFFSET_ADDR                       0x71
#define BMX160_OFFSET_CONF_ADDR                  0x77
#define BMX160_INT_STEP_CNT_0_ADDR               0x78
#define BMX160_INT_STEP_CONFIG_0_ADDR            0x7A
#define BMX160_INT_STEP_CONFIG_1_ADDR            0x7B
#define BMX160_COMMAND_REG_ADDR                  0x7E
#define BMX160_SPI_COMM_TEST_ADDR                0x7F
#define BMX160_INTL_PULLUP_CONF_ADDR             0x85

/* Sensor & time select definition*/
#define BMI160_ACCEL_SEL                          UINT8_C(0x01)
#define BMI160_GYRO_SEL                           UINT8_C(0x02)
#define BMI160_TIME_SEL                           UINT8_C(0x04)

#define ACC0_Q      (float)(0.06103515625*0.0098)
#define ACC1_Q      (float)(0.1220703125*0.0098)
#define ACC2_Q      (float)(0.244140625*0.0098)
#define ACC3_Q      (float)(0.48828125*0.0098)

#define GYR0_Q      (float)(0.0076294*0.01745329252)
#define GYR1_Q      (float)(0.0152588*0.01745329252)
#define GYR2_Q      (float)(0.0305176*0.01745329252)
#define GYR3_Q      (float)(0.0610352*0.01745329252)

#define BMX160_MAGN_UT_LSB      (0.3F)  ///< Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */

#define toRad(Math_D)   ((float)(Math_D)*0.017453292519943295769236907685f)
#define toDeg(Math_R)   ((float)(Math_R)*57.295779513082320876798154814105f)

extern void BMX160Init(unsigned char AccGrade, unsigned char GyroGrade);
extern void computeEuler(unsigned char AccGrade, unsigned char GyroGrade);
extern void Calibration_Mag(void);
#endif /* __BMX160_H__ */

