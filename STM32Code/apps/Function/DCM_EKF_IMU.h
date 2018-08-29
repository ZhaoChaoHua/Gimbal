#ifndef DCM_EKF_IMU_H
#define DCM_EKF_IMU_H

#ifdef __cplusplus
/**
 * Includes
 */
#include <AP_Math.h>
#include "parameter.h"

/**
 * Default constant values
 */
#define DEFAULT_q_dcm2               (0.1f*0.1f)
#define DEFAULT_q_gyro_bias2         (0.0003f*0.0003f)
#define DEFAULT_r_acc2               (0.02f*0.02f)
#define DEFAULT_r_a2                 (10*10)
#define DEFAULT_q_dcm2_init          (1*1)
#define DEFAULT_q_gyro_bias2_init    (0.01f*0.01f)



typedef struct ekf_state_struct
{
    float C31;
    float C32;
    float C33;
    Vector3f bais;
}ekf_state;

// Z-X-Y
class DCM_EKF {

public:

	DCM_EKF();

	void updateIMU(const Vector3f Gyroscope, const Vector3f Accelerometer, const float SamplePeriod);
    
    void resetEKF(void);

	void getCovariance(float *Covariance);

    inline void getState(ekf_state &gState) { gState = stste; }
     
	inline void getNGAcc(Vector3f &ngacc) { ngacc = a; }
    
    inline void getEuler(Vector3f &e) { e = euler * RAD_TO_DEG; }
		
		inline void getEulerRad(Vector3f &e) { e = euler;}
    // b -> n
    inline void getDCM(Matrix3f &d) { d = dcm; }
    
    inline void getACCinv(Vector3f &inv) { inv = acc_inv; }
    
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
private:
	AP_Float g0;
	AP_Float g0_2;
	AP_Float q_dcm2;
	AP_Float q_gyro_bias2;
	AP_Float r_acc2;
	AP_Float r_a2;

	Vector3f a;
    Vector3f acc_inv;
    ekf_state stste;
    Matrix3f dcm;
    
    Vector3f euler;
	float P00, P01, P02, P03, P04, P05;
	float P10, P11, P12, P13, P14, P15;
	float P20, P21, P22, P23, P24, P25;
	float P30, P31, P32, P33, P34, P35;
	float P40, P41, P42, P43, P44, P45;
	float P50, P51, P52, P53, P54, P55;
};

#endif
#endif
