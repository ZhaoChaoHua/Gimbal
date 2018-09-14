#include "Attitude.h"
#include "definitions.h"
#include "kiss_fft.h"
#include "att2mot.h"

KF kf_gyrox(0.008, 0.04);

void Attitude::start_init()
{
		
		AP_Param *beta;
		enum ap_var_type beta_type;
	  float beta_value;

		beta = AP_Param::find("ATT_M_P",&beta_type);
		beta_value = beta->cast_to_float(beta_type);
		beta_value = 0.0048;
		AP_Param::set_value(beta_type,beta,&beta_value);
		beta->save();

	  calibration_done = false;

    // MPU6500 init
    _mpu.mpu_init(0, BITS_DLPF_CFG_256HZ_NOLPF2, 0);
    _mpu.set_acc_scale(BITS_FS_8G);
    _mpu.set_gyro_scale(BITS_FS_500DPS);

    // Gyro Calibration
	  // New algorithm
    _mpu.gyroCalibration();
//	// Old
//	  _mpu.gyroOffsetCalibration();

		_notch_filter_gyro.init(2000, 182, 20, 100);



	#ifdef USING_EKF
	  _ekf.resetEKF();
	#endif
		_acc_cal.cal_done = true;

		rt_thread_delay(2);

//		kf_gyrox.init();
		
//		_mpu1.mpu_init(0, BITS_DLPF_CFG_20HZ, 1);
//    _mpu1.set_acc_scale(BITS_FS_8G);
//   
//    _mpu1.set_gyro_scale(BITS_FS_2000DPS);
//    
//    // Gyro Calibration
//    _mpu1.gyroOffsetCalibration();
}

int js_axr, js_ayr, js_azr, js_axf, js_ayf, js_azf;
int js_gxr, js_gyr, js_gzr, js_gxf, js_gyf, js_gzf;
int scop_acccomx, scop_acccomy, scop_acccomz;
int scop_acccamx, scop_acccamy, scop_acccamz;
int scop_accrawx, scop_accrawy, scop_accrawz;
int scop_err_x, scop_err_y, scop_err_z;
void Attitude::update()
{
	  static Vector3f last_gyro;
	  // time for calculate
	
	#ifdef USING_EKF
	  static float time = 0;
	  float time_unit = 0.001;
	  float time_count = 0.02;  //s
	  static uint32_t tnow, tPrev;
	  float dt;
    uint32_t count;
	#endif

    // MPU6500原始数据采集
    _mpu.read_Data();

		Acc_raw = _mpu.Acc_correct;
	  Gyro_raw = _mpu.Gyro_rad;
	
	  Accel_af = Acc_raw;
    Gyro_af = Gyro_raw;

	  // Lowpass filter
//    Accel_af = _accel_filter.apply(Acc_raw);
//    Gyro_af = _gyro_filter.apply(Gyro_raw);
		
//		Accel_af = _acc_fir_filter.update(_mpu.Acc_correct);
//		Gyro_af = _gyro_fir_filter.update(_mpu.Gyro_rad);
//		Gyro_af = _notch_filter_gyro.apply(Gyro_raw);
		
		js_gxr = Gyro_raw.x * 57295.8f;
		js_gyr = Gyro_raw.y * 57295.8f;
		js_gzr = Gyro_raw.z * 57295.8f;

		js_gxf = Gyro_af.x * 57295.8f;
		js_gyf = Gyro_af.y * 57295.8f;
		js_gzf = Gyro_af.z * 57295.8f;
		
	  // For acc calibration
	  _acc_caltor.new_sample(_mpu.Acc_ms2, 0);
		
//    Temp_af = Sliding_Windows_Filter_Temp(_mpu.temp_deg); 

		//Acc compensate
//		float k = 0.83f;
//		Vector3f acc_compensated;
//		acc_compensated.x = Accel_af.x - k*Acc_camera.x;
//		acc_compensated.y = Accel_af.y - k*Acc_camera.y;
//		acc_compensated.z = Accel_af.z - k*Acc_camera.z;
////		
////		scop_acccamx = Acc_camera.x * 1000;
////		scop_acccamy = Acc_camera.y * 1000;
////		scop_acccamz = Acc_camera.z * 1000;
////		
//		scop_acccomx = acc_compensated.x * 1000;
//		scop_acccomy = acc_compensated.y * 1000;
//		scop_acccomz = acc_compensated.z * 1000;
////
//		scop_accrawx = Accel_af.x * 1000;
//		scop_accrawy = Accel_af.y * 1000;
//		scop_accrawz = Accel_af.z * 1000;


    // 姿态解算
	#ifdef USING_MADGWICK
//    _madgwick.MadgwickUpdate(Gyro_af.x, Gyro_af.y, Gyro_af.z,
//                              Accel_af.x,  Accel_af.y,  Accel_af.z);
//    
//    _madgwick.getEuler(&euler, &euler_rad); 
		if(!calibration_done) calibration_done = true;//_madgwick.calibration_done;
	#endif
	
	#ifdef USING_MAHONY
	  _mahony.MahonyAHRSUpdate(Gyro_af.x, Gyro_af.y, Gyro_af.z,
                              Accel_af.x,  Accel_af.y,  Accel_af.z, err_y);
		_mahony.request_eular(&euler, &euler_rad);
	#endif
	
	#ifdef USING_EKF
	
		tnow = SysTick->VAL;
    count = (tnow > tPrev)?(SysTick->LOAD + tPrev - tnow) : (tPrev - tnow);
    dt = count / US_T;
    dt = dt / 1000000.0f;
		if(dt >= time_unit)
		{
			tPrev=tnow;
			this->angle_acc = (Gyro_af - last_gyro)/dt;
			last_gyro = Gyro_af;
		}
    if(dt < 0.0001f) dt = 0.00025f;
		
		_ekf.updateIMU(Gyro_af, Accel_af, dt);
		_ekf.getState(e_state);
		_ekf.getCovariance(P);
		_ekf.getNGAcc(ng_acc);
		_ekf.getEulerRad(euler_rad);
		_ekf.getEuler(euler);
		_ekf.getACCinv(acc_inv);
		_ekf.getDCM(e_dcm);
		
		correct_gyro = Gyro_af - e_state.bais;
		
		if(!calibration_done) calibration_done = true;
		
	#endif	
}

Vector3f Attitude::get_angle_acc(void)
{
	return this->angle_acc;
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
    return sinf(euler_rad.y);
}
float Attitude::cos_pitch()
{
    return cosf(euler_rad.y);
}
float Attitude::sin_roll()
{
    return sinf(euler_rad.x);
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
    return cosf(euler_rad.z);
}

void Attitude::set_madgwick_quaternion(float q0, float q1, float q2, float q3)
{
	_madgwick.q0 = q0;
	_madgwick.q1 = q2;
	_madgwick.q2 = q2;
	_madgwick.q3 = q3;
}
