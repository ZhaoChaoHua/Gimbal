#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#ifdef __cplusplus

#include "mpu6500.h"
#include "filter.h"
#include "attitude_filter.h"


#define FILTER_NUM 			    10       //滑动平均滤波数值个数

enum
{
    A_X = 0,
    A_Y,
    A_Z,
    G_Y,
    G_X,
    G_Z,
    TEM,
    ITEMS,
};

class Attitude
{
private:

    int16_t FILT_BUF[ITEMS][(FILTER_NUM + 1)];
    
    uint8_t filter_cnt,filter_cnt_old;
    float mpu_fil_tmp[ITEMS];

    Vector3f FILTER_FIFO[FILTER_NUM];
    float FILTER_FIFO_TEMP[FILTER_NUM];
    // 二阶滤波
    LowPassFilter2pVector3f _accel_filter;
    LowPassFilter2pVector3f _gyro_filter;
    
    // 滑动窗口滤波
    Vector3f Sliding_Windows_Filter(Vector3f in);
    float Sliding_Windows_Filter_Temp(float in);

    Madgwick&           _madgwick;
    MPU6500&              _mpu;
public:
    Vector3f              Gyro_af;
    Vector3f              Accel_af;
    float                 Temp_af;

    Vector3f              euler;
    Vector3f              euler_rad;
    
    float                 rawYawError;
    
    // 构造函数
    Attitude(Madgwick& imu, MPU6500& mpu_6500):
    _accel_filter(2000,20), // 采样频率2KHz, 截至频率20Hz
    _gyro_filter(2000,20),
    _madgwick(imu),
    _mpu(mpu_6500)
    {

    };
    
    void start_init();
    void update();
    void get_yawerror(float roll, float yaw, float angel);
    
    float sin_pitch();
    float cos_pitch();
    float sin_roll();
    float cos_roll();
    float sin_yaw();
    float cos_yaw();
};

#endif
#endif
