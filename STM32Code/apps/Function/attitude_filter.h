#ifndef __ATTITUDE_FILTER_H__
#define __ATTITUDE_FILTER_H__

#ifdef __cplusplus

#include <AP_Math.h>
#include "parameter.h"
#include <stm32f4xx.h>

// 梯度下降
class Madgwick
{
private:
    AP_Float beta;
    float invSqrt(float x);
public:
	  volatile float q0,q1,q2,q3; // 四元数
	  float acc_norm;   //纯加速度大小
    float axe;
    float aye;
    float aze;

    Madgwick(float bt);
    void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az);
    void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
    void getEuler(Vector3f *eular, Vector3f *rad);
    
    void set_beta(float b) { this->beta = b;};
		float get_beta(void) { return this->beta;};
		bool calibration_done;
		
    float   get_q0() { float q = q0; return q; }
    float   get_q1() { float q = q1; return q; }
    float   get_q2() { float q = q2; return q; }
    float   get_q3() { float q = q3; return q; }
		Vector3f get_acc_earth() { Vector3f v(axe, aye, aze); return v;}
		float get_acc_norm(){ return acc_norm;}
    
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
		
};

/* -------------------------------------------------------------------------- */
/*                                  Mahony                                    */
/* -------------------------------------------------------------------------- */
typedef struct accDeadband_s {
    uint8_t xy;                 // set the acc deadband for xy-Axis
    uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} accDeadband_t;

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;


#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))

// Mahony
class MahonyAHRS
{
private:
	float q0,q1,q2,q3;
	
	float rMat[3][3];

	accDeadband_t *accDeadband;

	uint32_t accTimeSum;        // keep track for integration of acc
	int accSumCount;

	int32_t accSum[3];
	float fc_acc;

	float invSqrt(float x);
	float atan2_approx(float y, float x);
	float sin_approx(float x);
	float acos_approx(float x);
    float To_180_degrees(float x);
protected:
    AP_Float Kp;
    AP_Float Ki;
    AP_Int8  t;

public:
	MahonyAHRS(float kp, float ki);
    void MahonyAHRSUpdate(float gx, float gy, float gz,  float ax, float ay, float az,  float yawError);
	void request_eular(Vector3f *eular, Vector3f *rad);
  float get_q0(void) {return q0;};
	float get_q1(void) {return q1;};
	float get_q2(void) {return q2;};
	float get_q3(void) {return q3;};

	void BodyToEarth(t_fp_vector * v);
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
};

#endif
#endif
