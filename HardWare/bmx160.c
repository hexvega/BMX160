#include "main.h"
#include "i2c.h"
#include "bmx160.h"
#include "math.h"
#include "usart.h"
#include "Madgwick_BMX160.h"
#include "Kalman_Filter.h"

#define IMU_FRAME_BYTE_NUMBER 48
#define IMU_RAW_DATA_LENGTH 20

HAL_StatusTypeDef read_status = 0;

float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};    //quaternion
uint8_t g_imuFinalData[IMU_FRAME_BYTE_NUMBER] = {0};
const uint8_t g_imuRegDataAddress[IMU_RAW_DATA_LENGTH] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
uint8_t g_imuData[IMU_RAW_DATA_LENGTH];

vector_3d_t mag_offset;
vector_3d_t mag_scale;	

vector_3d_t _accel_filter_value;
vector_3d_t _gyro_filter_value;
vector_3d_t _mag_filter_value;	
	

void quaternionToEulerAngles(float *y, float *p, float *r, float qtn[4]);	
void MadgwickAHRSupdateIMU_6(float gx, float gy, float gz, float ax, float ay, float az, volatile float qtn[4], uint16_t dt_ms);
void MadgwickAHRSupdateIMU_9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, volatile float qtn[4], uint16_t dt_ms);
	
uint8_t BMI160_Write_Byte(uint8_t reg, uint8_t val)
{
	uint8_t writeRegisterCmd = val;
	
	return HAL_I2C_Mem_Write(&hi2c1, (IIC_ADDR<<1), reg, I2C_MEMADD_SIZE_8BIT, &writeRegisterCmd, 1, 100);
}

uint8_t BMI160_Read_Byte(uint8_t reg)
{
	uint8_t readRegisterValue = 0x00;
  HAL_I2C_Mem_Read(&hi2c1, (IIC_ADDR<<1), reg, I2C_MEMADD_SIZE_8BIT, &readRegisterValue, 1, 100);
	
	return readRegisterValue;
}

HAL_StatusTypeDef ReadRegister(uint8_t reg_addr, uint8_t *out_value)
{
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(&hi2c1, (IIC_ADDR<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, out_value, sizeof(uint8_t)*IMU_RAW_DATA_LENGTH, 80);
    return hal_status;
}

void BMX160Init(unsigned char AccGrade, unsigned char GyroGrade)
{
		uint16_t delayValueMs = 40;
		uint8_t regCmd = 0x00;
		uint8_t regData = 0x00;
	
		mag_offset.x = -212.0f;
		mag_offset.y = 56.7f;
    mag_offset.z = -116.1f;
	
		mag_scale.x = 0.732f;	
		mag_scale.y = 0.743f;
		mag_scale.z = 3.481f;

		BMI160_Write_Byte(BMX160_COMMAND_REG_ADDR, BMX160_SOFT_RESET_CMD);
		HAL_Delay(200);
		
		while(1){
				regData = BMI160_Read_Byte(BMX160_CHIP_ID_ADDR);
				if(BMX160_CHIP_ID == regData){
						break;
				}else{
					myI2C_REST(&hi2c1);
					HAL_Delay(10);
				}
		}
		
		HAL_Delay(10); 

		regData = BMI160_Read_Byte(BMI160_PMU_STATUS);
		HAL_Delay(50); 
		
    regCmd = 0x11;   //0x11，配置加速度计正常模式，0x12为低功耗模式。生效耗时：3.2~3.8ms
		BMI160_Write_Byte(BMX160_COMMAND_REG_ADDR, regCmd);
    HAL_Delay(50); 
//    BMI160_Read_Byte(BMI160_COMMAND_REG_ADDR, &getData);

    regCmd = 0x15;   //0x15，配置陀螺仪正常模式，0x17为低功耗模式。生效耗时：55~80ms
		BMI160_Write_Byte(BMX160_COMMAND_REG_ADDR, regCmd);
    HAL_Delay(100); 
		
		regCmd = 0x19;   //0x19，配置磁力计正常模式，0x1A为低功耗模式。生效耗时：0.35~0.5ms
		BMI160_Write_Byte(BMX160_COMMAND_REG_ADDR, regCmd);
    HAL_Delay(10); 
		
		regData = BMI160_Read_Byte(BMI160_PMU_STATUS);
		HAL_Delay(10); 
		
		regCmd = 0x2C;    //配置加速度计输出速率 0x2C@1600Hz，0x2B@800Hz，
		BMI160_Write_Byte(BMX160_ACCEL_CONFIG_ADDR, regCmd);
		HAL_Delay(10); 
		
    switch(AccGrade){
        case 0: regCmd = 0x03;   break;   //±2g
        case 1: regCmd = 0x05;   break;   //±4g
        case 2: regCmd = 0x08;   break;   //±8g       //优选
        case 3: regCmd = 0x0C;   break;   //±16g
			default: regCmd = 0x03;   break;   //±2g
    }
		BMI160_Write_Byte(BMX160_ACCEL_RANGE_ADDR, regCmd);
    HAL_Delay(10);
		ReadRegister(g_imuRegDataAddress[0],(uint8_t*)g_imuData);
		HAL_Delay(10);
		
		
		regCmd = 0x2C;    //配置加速度计输出速率 0x2C@1600Hz， 0x2D@3200Hz， 0x2B@800Hz，
		BMI160_Write_Byte(BMX160_GYRO_CONFIG_ADDR, regCmd);
		HAL_Delay(10); 
    switch(GyroGrade){
        case 0: regCmd = 0x03;   break;   //±250 °/s
        case 1: regCmd = 0x02;   break;   //±500 °/s
        case 2: regCmd = 0x01;   break;   //±1000 °/s    //优选
        case 3: regCmd = 0x00;   break;   //±2000 °/s
    }
		BMI160_Write_Byte(BMX160_GYRO_RANGE_ADDR, regCmd);
    HAL_Delay(10);
		ReadRegister(g_imuRegDataAddress[0],(uint8_t*)g_imuData);HAL_Delay(10);

		BMI160_Write_Byte(BMX160_MAGN_IF_0_ADDR, 0x80);
    HAL_Delay(60);
    // Sleep mode
    BMI160_Write_Byte(BMX160_MAGN_IF_3_ADDR, 0x01);HAL_Delay(10); 
    BMI160_Write_Byte(BMX160_MAGN_IF_2_ADDR, 0x4B);HAL_Delay(10); 
    // REPXY 
    BMI160_Write_Byte(BMX160_MAGN_IF_3_ADDR, 0x04);HAL_Delay(10); 
    BMI160_Write_Byte(BMX160_MAGN_IF_2_ADDR, 0x51);HAL_Delay(10); 
    // REPZ
    BMI160_Write_Byte(BMX160_MAGN_IF_3_ADDR, 0x0E);HAL_Delay(10); 
    BMI160_Write_Byte(BMX160_MAGN_IF_2_ADDR, 0x52);HAL_Delay(10); 
    
    BMI160_Write_Byte(BMX160_MAGN_IF_3_ADDR, 0x02);HAL_Delay(10); 
    BMI160_Write_Byte(BMX160_MAGN_IF_2_ADDR, 0x4C);HAL_Delay(10); 
    BMI160_Write_Byte(BMX160_MAGN_IF_1_ADDR, 0x42);HAL_Delay(10); 
    BMI160_Write_Byte(BMX160_MAGN_CONFIG_ADDR, 0x08);HAL_Delay(10); 
    BMI160_Write_Byte(BMX160_MAGN_IF_0_ADDR, 0x03);HAL_Delay(10); 
    HAL_Delay(60);
		
		regData = BMI160_Read_Byte(BMX160_MAGN_CONFIG_ADDR);
		HAL_Delay(60);
		for(int i=0;i<30;i++)
		{
			read_status = ReadRegister(g_imuRegDataAddress[0],(uint8_t*)g_imuData);
		}
}

void Calibration_Mag(void)	
{
	float magOrgX=0,magOrgY=0,magOrgZ=0;
	vector_3d_t mag_max;
	vector_3d_t mag_min;
	vector_3d_t mag_chord;
	uint32_t temp = 0;
	float avg_chord;
		/* make sure max < mag data first  */
	mag_max.x = -2000;
	mag_max.y = -2000;
	mag_max.z = -2000;

	/* make sure min > mag data first  */
	mag_min.x = 2000;
	mag_min.y = 2000;
	mag_min.z = 2000;
	
	for(temp=0; temp<10000; temp++)
	{
		if(temp%5==0){LED_GREEN_Toggle();}
		HAL_Delay(2);
		read_status = ReadRegister(g_imuRegDataAddress[0],(uint8_t*)g_imuData);
		if(read_status!=0){
			HAL_Delay(2);
			continue;
		}
		magOrgX = ((int16_t)((((uint16_t)(g_imuData[1]))<<8) | (g_imuData[0])))*0.3f;	
		magOrgY = ((int16_t)((((uint16_t)(g_imuData[3]))<<8) | (g_imuData[2])))*0.3f;	
		magOrgZ = ((int16_t)((((uint16_t)(g_imuData[5]))<<8) | (g_imuData[4])))*0.3f;
		if((magOrgX==0) && (magOrgY==0) && (magOrgZ==0))
		{
			continue;
		}
		mag_min.x = (magOrgX < mag_min.x) ? magOrgX : mag_min.x;
		mag_max.x = (magOrgX > mag_max.x) ? magOrgX : mag_max.x;
		mag_max.y = (magOrgY > mag_max.y) ? magOrgY : mag_max.y;
		mag_min.y = (magOrgY < mag_min.y) ? magOrgY : mag_min.y;	
		mag_min.z = (magOrgZ < mag_min.z) ? magOrgZ : mag_min.z;
		mag_max.z = (magOrgZ > mag_max.z) ? magOrgZ : mag_max.z;
		
		mag_offset.x = (mag_max.x + mag_min.x) / 2;
    mag_offset.y = (mag_max.y + mag_min.y) / 2;
    mag_offset.z = (mag_max.z + mag_min.z) / 2;
    // added soft iron calibration
    mag_chord.x = (mag_max.x - mag_min.x) / 2;
    mag_chord.y = (mag_max.y - mag_min.y) / 2;
    mag_chord.z = (mag_max.z - mag_min.z) / 2;
    avg_chord = (mag_chord.x + mag_chord.y + mag_chord.z)/3;
    mag_scale.x = avg_chord / mag_chord.x;
    mag_scale.y = avg_chord / mag_chord.y;
    mag_scale.z = avg_chord / mag_chord.z;
	}
}

void computeEuler(unsigned char AccGrade, unsigned char GyroGrade)
{
		uint8_t count = 0;
		uint8_t i = 0;
    float A_Q, G_Q;
		float gyrSqrt = 0.0f;
		static float last_Pitch=0.0f,last_Yaw=0.0f,last_Roll=0.0f;
	
		vector_3d_t accel_buffer_value;
		vector_3d_t gyro_buffer_value;
		vector_3d_t mag_buffer_value;
	
		int16_t gyrX=0,gyrY=0,gyrZ=0;
		int16_t accX=0,accY=0,accZ=0;
		int16_t magX=0,magY=0,magZ=0;
	
		read_status = ReadRegister(g_imuRegDataAddress[0],(uint8_t*)g_imuData);
		if(read_status!=0){
			HAL_Delay(2);
		}
		magX = ((int16_t)((((uint16_t)(g_imuData[1]))<<8) | (g_imuData[0])))*0.3f;	
		magY = ((int16_t)((((uint16_t)(g_imuData[3]))<<8) | (g_imuData[2])))*0.3f;	
		magZ = ((int16_t)((((uint16_t)(g_imuData[5]))<<8) | (g_imuData[4])))*0.3f;
	
		gyrX = (int16_t)((((uint16_t)(g_imuData[9]))<<8) | (g_imuData[8]));		
		gyrY = (int16_t)((((uint16_t)(g_imuData[11]))<<8) | (g_imuData[10]));	
		gyrZ = (int16_t)((((uint16_t)(g_imuData[13]))<<8) | (g_imuData[12]));	
	
		accX = (int16_t)((((uint16_t)(g_imuData[15]))<<8) | (g_imuData[14]));	
		accY = (int16_t)((((uint16_t)(g_imuData[17]))<<8) | (g_imuData[16]));	
		accZ = (int16_t)((((uint16_t)(g_imuData[19]))<<8) | (g_imuData[18]));	
	
	 switch(AccGrade){  //选择量程范围的转换刻度，±8g
        case 0: A_Q = ACC0_Q;   break;
        case 1: A_Q = ACC1_Q;   break;
        case 2: A_Q = ACC2_Q;   break;   //±8g
        case 3: A_Q = ACC3_Q;   break;
    }
			
	 switch(GyroGrade){
        case 0: G_Q = GYR0_Q;   break;
        case 1: G_Q = GYR1_Q;   break;
        case 2: G_Q = GYR2_Q;   break;
        case 3: G_Q = GYR3_Q;   break;
    }

		accel_buffer_value.x = (float)(accX) * A_Q;
		accel_buffer_value.y = (float)(accY) * A_Q;
		accel_buffer_value.z = (float)(accZ) * A_Q;

		gyro_buffer_value.x = (float)(gyrX) * G_Q;
		gyro_buffer_value.y = (float)(gyrY) * G_Q;
		gyro_buffer_value.z = (float)(gyrZ) * G_Q;

		
		mag_buffer_value.x = ((magX - mag_offset.x)*mag_scale.x);
		mag_buffer_value.y = ((magY - mag_offset.y)*mag_scale.x);
		mag_buffer_value.z = ((magZ - mag_offset.z)*mag_scale.x);

		_accel_filter_value.x = kalmanFilter(&KFP_accel_x, accel_buffer_value.x);
		_accel_filter_value.y = kalmanFilter(&KFP_accel_y, accel_buffer_value.y);
		_accel_filter_value.z = kalmanFilter(&KFP_accel_z, accel_buffer_value.z);
		_gyro_filter_value.x = kalmanFilter(&KFP_gyro_x, gyro_buffer_value.x);
		_gyro_filter_value.y = kalmanFilter(&KFP_gyro_y, gyro_buffer_value.y);
		_gyro_filter_value.z = kalmanFilter(&KFP_gyro_z, gyro_buffer_value.z);
		_mag_filter_value.x = kalmanFilter(&KFP_mag_x, mag_buffer_value.x);
		_mag_filter_value.y = kalmanFilter(&KFP_mag_y, mag_buffer_value.y);
		_mag_filter_value.z = kalmanFilter(&KFP_mag_z, mag_buffer_value.z);


//		gyrSqrt = invSqrt((_gyro_filter_value.x*_gyro_filter_value.x) + (_gyro_filter_value.y*_gyro_filter_value.y) + (_gyro_filter_value.z*_gyro_filter_value.z));
//		roll = gyrSqrt;

//		updateIMU(_gyro_filter_value.x, _gyro_filter_value.y, _gyro_filter_value.z, _accel_filter_value.x, _accel_filter_value.y, _accel_filter_value.z, gyrSqrt);
////		update(_gyro_filter_value.x, _gyro_filter_value.y, _gyro_filter_value.z, _accel_filter_value.x, _accel_filter_value.y, _accel_filter_value.z, _mag_filter_value.x, _mag_filter_value.y, _mag_filter_value.z);
//	
//		pitch = kalmanFilter(&KFP_pitch, pitch);
//		yaw = kalmanFilter(&KFP_yaw, yaw);
//		roll = kalmanFilter(&KFP_roll, roll);

//		pitch = 0.15*last_Pitch + 0.85*pitch;
//		yaw = 0.1*last_Yaw + 0.9*yaw;
//		roll = 0.15*last_Roll + 0.85*roll;
//		
//		last_Pitch = pitch;
//		last_Yaw = yaw;
//		last_Roll = roll;
}
