#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#ifdef __cplusplus

#include "mpu6500.h"
#include "filter.h"
#include "attitude_filter.h"
#include "bmm150.h"
#include "AccelCal.h"
#include "DCM_EKF_IMU.h"
#include "fir_vec3f.h"
#include "NotchFilter.h"

#define USING_MADGWICK
//#define USING_EKF


#define FILTER_NUM 			    10       //滑动平均滤波数值个数

// FIR Definetion
#define ACC_ORDER			10
#define ACC_COEF_LEN      ACC_ORDER+1
// ACC X Freq cut = 50
static float acc_coef[ACC_COEF_LEN] = {
	     0.01415480141223,    0.030102240352,  0.07209877436059,   0.1245408463908,
     0.1672746948244,   0.1836572853198,   0.1672746948244,   0.1245408463908,
    0.07209877436059,    0.030102240352,  0.01415480141223
};


#define GYRO_ORDER			10
#define GYRO_COEF_LEN      GYRO_ORDER+1
// Gyro x Freq cut = 100
static float gyro_coef[GYRO_COEF_LEN] = {
      -0,-0.005878733079749,  0.01225497357087,   0.1012036621354,
     0.2360642383216,   0.3127117181038,   0.2360642383216,   0.1012036621354,
    0.01225497357087,-0.005878733079749,                -0
};

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
    LowPassFilter2pVector3f _mag_filter;
    
    // 滑动窗口滤波
    Vector3f Sliding_Windows_Filter(Vector3f in);
    float Sliding_Windows_Filter_Temp(float in);

    Madgwick&           _madgwick;
    MahonyAHRS&         _mahony;  
    DCM_EKF&                _ekf;
    MPU6500&              _mpu;
    BMM150&               _bmm;
		
		MPU6500&              _mpu1;
		AccelCal&             _acc_cal;
		AccelCalibrator&      _acc_caltor;
		
		// FIR Filter
    FIR_VEC3F           _gyro_fir_filter;
    FIR_VEC3F           _acc_fir_filter;
		NotchFilterVector3f   _notch_filter_gyro;
			
//		FIR_VEC3F           _acc_fir_1;
//		FIR_VEC3F           _acc_fir_2;
//		FIR_VEC3F           _acc_fir_3;

public:
	  Vector3f              Gyro_raw;
    Vector3f              Gyro_af;
    Vector3f              Acc_raw;
    Vector3f              Accel_af;
    Vector3f              Accel_af1;
		Vector3f              Err_af;
    Vector3f              Vel_af;
    Vector3f              Acc_camera;

    Vector3f              follow_head;
    float                 Temp_af;

    // EKF
		ekf_state             e_state;
    float                 P[36];
    Vector3f              ng_acc;
    Vector3f              acc_inv;
    Matrix3f              e_dcm;
    Vector3f              correct_gyro;

    Vector3f              gyro_fir_data;
    Vector3f              acc_fir_data;
		Vector3f              acc_fir_data_1;
		Vector3f              acc_fir_data_2;
//		Vector3f              acc_fir_data_3;


    Vector3f              euler;
    Vector3f              euler_rad;
    Vector3f              angle_acc;
    Vector3f              euler_rad_raw;
    Vector3f              euler_rad_zeros;
    
    float                 rawYawError;
    bool                  calibration_done;

    float                 err_y;

    float                 beta_m;
    
    // 构造函数
    Attitude(Madgwick& mag_imu, MahonyAHRS& mah_imu, DCM_EKF& ekf, MPU6500& mpu_6500, BMM150& bmm_150, MPU6500& mpu1, AccelCal& acc, AccelCalibrator& acc_caltor):
    _accel_filter(2000,40), // 采样频率2KHz, 截至频率20Hz
    _gyro_filter(2000,100),
    _mag_filter(2000,100),
    _madgwick(mag_imu),
    _mahony(mah_imu),
		_ekf(ekf),
    _mpu(mpu_6500),
    _bmm(bmm_150),
    _mpu1(mpu1),
    _acc_cal(acc),
		_acc_caltor(acc_caltor),
		_gyro_fir_filter(GYRO_COEF_LEN, gyro_coef, gyro_coef, gyro_coef),
		_acc_fir_filter(ACC_COEF_LEN, acc_coef, acc_coef, acc_coef)
//		_acc_fir_1(ACC_1_COEF_LEN, acc_1_coef, acc_1_coef, acc_1_coef),
//		_acc_fir_2(ACC_2_COEF_LEN, acc_2_coef, acc_2_coef, acc_2_coef)
//		_acc_fir_3(ACC_3_COEF_LEN, acc_3_coef, acc_3_coef, acc_3_coef)
    {

    };

    
    void start_init();
    void update();
    void get_yawerror(float roll, float yaw, float angel);
		
		float q0(void) {return _madgwick.get_q0();};
		float q1(void) {return _madgwick.get_q1();};
		float q2(void) {return _madgwick.get_q2();};
		float q3(void) {return _madgwick.get_q3();};
		Vector3f acc_earth(void) { return _madgwick.get_acc_earth();}
		float acc_norm(void) { return _madgwick.get_acc_norm();}
		float ax_earth(void) {return _madgwick.axe;};
		float ay_earth(void) {return _madgwick.aye;};
		float az_earth(void) {return _madgwick.aze;};
		
		void set_madgwick_quaternion(float q0, float q1, float q2, float q3);
    
    float sin_pitch();
    float cos_pitch();
    float sin_roll();
    float cos_roll();
    float sin_yaw();
    float cos_yaw();
		bool calibration_Done(void) { return calibration_done;};
		
		void get_err_y(float err) { err_y = err;};
		Vector3i get_gyro_i16(void) { return _mpu.Gyro_I16;};
		Vector3f get_angle_acc(void);
		
		Vector3f reset_euler_z_zeros(void) {this->euler_rad_zeros.z = this->euler_rad_raw.z; return this->euler_rad_zeros;}
};

#endif
#endif
