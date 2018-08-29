#include "Attitude.h"
#include "definitions.h"

void Attitude::start_init()
{
	  int i;  
	
    // MPU6500 init
    _mpu.mpu_init(0, BITS_DLPF_CFG_20HZ);
    _mpu.set_acc_scale(BITS_FS_8G);
   
    _mpu.set_gyro_scale(BITS_FS_2000DPS);
    
    // Gyro Calibration
    _mpu.gyroOffsetCalibration();
	  
	  for(i = 0; i < 10000; i++)
    {
			this->update();
		}
	  _madgwick.set_beta(0.002);

}

void Attitude::update()
{
	  float x,y,z;
	  Vector3f rad;
    // MPU6500原始数据采集
    _mpu.read_Data();
    
    Accel_af = _accel_filter.apply(_mpu.Acc_correct);
    Gyro_af = _gyro_filter.apply(_mpu.Gyro_rad);
    
    Temp_af = Sliding_Windows_Filter_Temp(_mpu.temp_deg);
    
    // 姿态解算
	  _madgwick.MadgwickUpdate(Gyro_af.x, Gyro_af.y, Gyro_af.z,
                              Accel_af.x,  Accel_af.y,  Accel_af.z);
    
	  _madgwick.getEuler(&euler, &euler_rad); 
	

}

void Attitude::get_yawerror(float roll_err, float yaw_err, float angel)
{
    rawYawError = -(yaw_err * cosf(angel*DEG_TO_RAD) + roll_err * sinf(angel*DEG_TO_RAD) - euler.z) * DEG_TO_RAD;
}

Vector3f Attitude::Sliding_Windows_Filter(Vector3f input)
{
    Vector3f sum;
    
    for(uint8_t i=1;i<FILTER_NUM;i++)
    {	//FIFO 操作
        FILTER_FIFO[i-1] = FILTER_FIFO[i];
    }
    
    FILTER_FIFO[FILTER_NUM-1] = input;
    
    for(uint8_t i=0;i<FILTER_NUM;i++)
    {
        sum.x += FILTER_FIFO[i].x;
        sum.y += FILTER_FIFO[i].y;
        sum.z += FILTER_FIFO[i].z;
    }
    return sum/FILTER_NUM;
}

float Attitude::Sliding_Windows_Filter_Temp(float in)
{
    float sum;
    
    for(uint8_t i=1;i<FILTER_NUM;i++)
    {	//FIFO 操作
        FILTER_FIFO_TEMP[i-1] = FILTER_FIFO_TEMP[i];
    }
    
    FILTER_FIFO_TEMP[FILTER_NUM-1] = in;
    
    for(uint8_t i=0;i<FILTER_NUM;i++)
    {
        sum += FILTER_FIFO_TEMP[i];
    }
    return sum/FILTER_NUM;
}

float Attitude::sin_pitch()
{
    return sinf(euler_rad.x);
}
float Attitude::cos_pitch()
{
    return cosf(euler_rad.x);
}
float Attitude::sin_roll()
{
    return sinf(euler_rad.y);
}
float Attitude::cos_roll()
{
    return cosf(euler_rad.x);
}
float Attitude::sin_yaw()
{
    return sinf(euler_rad.z);
}
float Attitude::cos_yaw()
{
    return cosf(euler_rad.x);
}
