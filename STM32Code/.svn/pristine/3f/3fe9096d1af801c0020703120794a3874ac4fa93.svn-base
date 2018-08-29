#include "attitude_filter.h"
#include "stm32f40x_define.h"


// 四元数初始化
void Quarternion_math::quart_init(Quarternion q)
{
	q[0] = 0;
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
}

// 设置四元数
void Quarternion_math::quart_set(Quarternion q, float w, float x, float y, float z)
{
	q[0] = w;
	q[1] = x;
	q[2] = y;
	q[3] = z;
}

void Quarternion_math::quart_from_eular_d(Quarternion q, Eular e)
{
	q[0] = 1;
	q[1] = e[0] / 2;
	q[2] = e[1] / 2;
	q[3] = e[2] / 2;
}

// 欧拉角转四元数
void Quarternion_math::quart_from_eular(Quarternion q, Eular e)
{
	float cos_e0 = arm_cos_f32(e[0] / 2);
	float sin_e0 = arm_sin_f32(e[0] / 2);
	float cos_e1 = arm_cos_f32(e[1] / 2);
	float sin_e1 = arm_sin_f32(e[1] / 2);
	float cos_e2 = arm_cos_f32(e[2] / 2);
	float sin_e2 = arm_sin_f32(e[2] / 2);
	
	q[0] = cos_e0 * cos_e1 * cos_e2
		+ sin_e0 * sin_e1 * sin_e2;
	q[1] = sin_e0 * cos_e1 * cos_e2
		- cos_e0 * sin_e1 * sin_e2;
	q[2] = cos_e0 * sin_e1 * cos_e2
		+ sin_e0 * cos_e1 * sin_e2;
	q[3] = cos_e0 * cos_e1 * sin_e2
		- sin_e0 * sin_e1 * cos_e2;
}

// 四元数模量
float Quarternion_math::quart_length(Quarternion q)
{
	return __sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

// 四元数共轭
void Quarternion_math::quart_conjugation(Quarternion q)
{
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
}

// 四元数求逆(四元数求逆)
void Quarternion_math::quart_inverse(Quarternion q)
{
	float len = quart_length(q);
	if (len == 0)
	{
		q[0] = q[1] = q[2] = 0;
		return;
	}
	quart_conjugation(q);
	len = len * len;
	q[0] /= len;
	q[1] /= len;
	q[2] /= len;
	q[3] /= len;
}

// 四元数转欧拉角
void Quarternion_math::quart_to_eular(Quarternion q, Eular e)
{	
	float s1 = 2 * q[0] * q[2] - 2 * q[3] * q[1];
	
	if(s1 > 1)
		s1 = 1;
	if(s1 < -1)
		s1 = -1;
	
	e[0] = atan2f(2 * q[0] * q[1] + 2 * q[2] * q[3], 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]);
	e[1] = asinf(s1);
	e[2] = atan2f(2 * q[0] * q[3] + 2 * q[1] * q[2], 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]);
}

// 四元数归一化
void Quarternion_math::quart_normalize(Quarternion q)
{
	float len = quart_length(q);
	if(len != 0)
	{
		q[0] /= len;
		q[1] /= len;
		q[2] /= len;
		q[3] /= len;
	}
	else
	{
		q[0] = q[1] = q[2] = q[3] = 0;
	}
}

void Quarternion_math::quart_from_vector(Quarternion q, Quarternion v1, Quarternion v2)
{
	float angle_div2;
	float axis[3];
	float alen;
	
	quart_normalize(v1);
	quart_normalize(v2);

	angle_div2 = acosf(v1[1] * v2[1] + v1[2] * v2[2] + v1[3] * v2[3]) / 2;
	
	axis[0] = v1[2] * v2[3] - v1[3] * v2[2];
	axis[1] = v1[3] * v2[1] - v1[1] * v2[3];
	axis[2] = v1[1] * v2[2] - v1[2] * v2[1];
	alen = __sqrtf(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
	axis[0] /= alen;
	axis[1] /= alen;
	axis[2] /= alen;

	q[0] = arm_cos_f32(angle_div2);
	q[1] = axis[0] * arm_sin_f32(angle_div2);
	q[2] = axis[1] * arm_sin_f32(angle_div2);
	q[3] = axis[2] * arm_sin_f32(angle_div2);
}
	
void Quarternion_math::quart_slerp(Quarternion pDest, Quarternion pQ1, Quarternion pQ2, float t)
{
	float rot1q[4];
	float omega, cosom, oosinom;
	float scalerot0, scalerot1;

	//Calculate the cosine
	cosom = pQ1[0] * pQ2[0] + pQ1[1] * pQ2[1] + pQ1[2] * pQ2[2] + pQ1[3] * pQ2[3];

	//adjust signs if necessary
	if(cosom < 0.0f)
	{
		cosom = -cosom;
		rot1q[0] = -pQ2[0];
		rot1q[1] = -pQ2[1];
		rot1q[2] = -pQ2[2];
		rot1q[3] = -pQ2[3];
	}
	else  
	{
		rot1q[0] = pQ2[0];
		rot1q[1] = pQ2[1];
		rot1q[2] = pQ2[2];
		rot1q[3] = pQ2[3];
	}

	//calculate interpolating coeffs
	if ( (1.0f - cosom) > 0.0001f ) 
	{ 
		//standard case
		omega   = acosf(cosom);
		oosinom = 1.0f / arm_sin_f32(omega);
		scalerot0 = arm_sin_f32((1.0f - t) * omega) * oosinom;
		scalerot1 = arm_sin_f32(t * omega) * oosinom;
	}
	else
	{ 
		//rot0 and rot1 very close - just do linear interp.
		scalerot0 = 1.0f - t;
		scalerot1 = t;
	}

	//build the new quarternion
	pDest[0] = (scalerot0 * pQ1[0] + scalerot1 * rot1q[0]);
	pDest[1] = (scalerot0 * pQ1[1] + scalerot1 * rot1q[1]);
	pDest[2] = (scalerot0 * pQ1[2] + scalerot1 * rot1q[2]);
	pDest[3] = (scalerot0 * pQ1[3] + scalerot1 * rot1q[3]);
	
	quart_normalize(pDest);
}

// 四元数加
void Quarternion_math::quart_add(Quarternion q, Quarternion _q)
{
	q[0] = q[0] + _q[0];
	q[1] = q[1] + _q[1];
	q[2] = q[2] + _q[2];
	q[3] = q[3] + _q[3];
}

// 四元数减
void Quarternion_math::quart_minus(Quarternion q, Quarternion _q)
{
	q[0] = q[0] - _q[0];
	q[1] = q[1] - _q[1];
	q[2] = q[2] - _q[2];
	q[3] = q[3] - _q[3];
}

// 四元数乘四元数
void Quarternion_math::quart_multi_q(Quarternion q, Quarternion _q)
{
	float qq[4];
	qq[0] = q[0] * _q[0] - q[1] * _q[1] - q[2] * _q[2] - q[3] * _q[3];
	qq[1] = q[1] * _q[0] + q[0] * _q[1] + q[2] * _q[3] - q[3] * _q[2];
	qq[2] = q[2] * _q[0] + q[0] * _q[2] + q[3] * _q[1] - q[1] * _q[3];
	qq[3] = q[3] * _q[0] + q[0] * _q[3] + q[1] * _q[2] - q[2] * _q[1];
	q[0] = qq[0];
	q[1] = qq[1];
	q[2] = qq[2];
	q[3] = qq[3];
}

// 四元数乘浮点数
void Quarternion_math::quart_multi_f(Quarternion q, float f)
{
	q[0] = q[0] * f;
	q[1] = q[1] * f;
	q[2] = q[2] * f;
	q[3] = q[3] * f;
}

//---------------------------------------------------------------------------------------------------
// 梯度下降姿态解算
Madgwick::Madgwick(float bt)
{
    // 采样频率初始化
    this->beta = bt;
	  this->dt_ms = 0;
		// 四元数初始化
    this->q0 = 1.0f;
    this->q1 = 0.0f;
    this->q2 = 0.0f;
    this->q3 = 0.0f;
	    
}


float Madgwick::invSqrt(float x)
{
	/*
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
	*/
	
	return 1.0f / sqrtf(x);
}

// 梯度下降算法
void Madgwick::MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
    float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    
    if(sampleFreq < 0.0001f || sampleFreq > 0.001f)
        sampleFreq = 0.0005f;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickUpdate(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
		// Normalise accelerometer measurement
		recipNorm = sqrtf(ax * ax + ay * ay + az * az);
        if(recipNorm == 0)
            return;
        recipNorm = 1.0f/recipNorm;
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = sqrtf(mx * mx + my * my + mz * mz);
        if(recipNorm == 0)
            return;
        recipNorm = 1.0f/recipNorm;
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        if(recipNorm == 0)
            return;
        recipNorm = 1.0f/recipNorm;
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * sampleFreq;
	q1 += qDot2 * sampleFreq;
	q2 += qDot3 * sampleFreq;
	q3 += qDot4 * sampleFreq;

	// Normalise quaternion
	recipNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if(recipNorm == 0)
            return;
        recipNorm = 1.0f/recipNorm;
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

int opsDT;

void Madgwick::MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	
	static uint32_t tnow, tPrev;
	float dt;
  uint32_t count;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	
	tnow = SysTick->VAL;
  count = (tnow > tPrev)?(SysTick->LOAD + tPrev - tnow) : (tPrev - tnow);
  dt = count / US_T;
  dt = dt / 1000000.0f;
  tPrev=tnow;
  if(dt < 0.0001f) dt = 0.00025f;
	
	dt_ms = dt*1000;
	
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

// 重要:求解欧拉角注意旋转矩阵方向
void Madgwick::getEuler(Vector3f *eular, Vector3f *rad)
{   
    rad->y = asinf(2*q1*q3 - 2*q0*q2);								// pitch
    rad->x = atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1);		// roll
    rad->z = -atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1);		// yaw

    eular->x = rad->x * RAD_TO_DEG;
    eular->y = rad->y * RAD_TO_DEG;
    eular->z = rad->z * RAD_TO_DEG;
}


/* ----------------------------------------卡尔曼滤波-------------------------------------- */

Kalman_AHRS::Kalman_AHRS(int sample)
{
    // 采样率3000Hz
    sample_rate = sample;
    
    position_e[0] = 0.0f;
    position_e[1] = 0.0f;
    position_e[2] = 0.0f;
    
    acc_relative[0] = 0.0f;
    acc_relative[1] = 0.0f;
    acc_relative[2] = 0.0f;
    
    position_function = 0;
}

float Kalman_AHRS::kalman_mix_ring(__Kalman *k, float y0, float u, float mid)
{
	int f_round = 0;
	float y;
	float P01 = k->P + k->Q;
	
	if (y0 > mid && u < -mid)
	{
		f_round = 1;
		y0 -= 2 * mid;
		u += 2 * mid;
	}
	else if (y0 < -mid && u > mid)
	{
		f_round = 1;
		y0 += 2 * mid;
		u -= 2 * mid;
	}
	
	k->K = P01 / (P01 + k->R);
	y = y0 + k->K * (u - y0);
	k->P = (1 - k->K) * P01;

	if (f_round == 1)
	{
		if (y > 0)
			y -= 2 * mid;
		else
			y += 2 * mid;
	}
	return y;
}

float Kalman_AHRS::kalman_mix(__Kalman *k, float y0, float u)
{
	float y;
	float P01 = k->P + k->Q;
	
	k->K = P01 / (P01 + k->R);
	y = y0 + k->K * (u - y0);
	k->P = (1 - k->K) * P01;

	return y;
}

float Kalman_AHRS::sub_ring(float x1, float x2, float mid)
{
	if (x1 > mid && x2 < -mid)
	{
		x1 -= 2 * mid;
		x2 += 2 * mid;
	}
	if (x1 < -mid && x2 > mid)
	{
		x1 += 2 * mid;
		x2 -= 2 * mid;
	}

	return x1 - x2;
}

void Kalman_AHRS::posit_adjust_init()
{
	position_q_adjust[0] = 1;
	position_q_adjust[1] = 0;
	position_q_adjust[2] = 0;
	position_q_adjust[3] = 0;
}

void Kalman_AHRS::posit_adjust_rst()
{
	position_q_adjust[0] = position_q[0];
	position_q_adjust[1] = position_q[1];
	position_q_adjust[2] = position_q[2];
	position_q_adjust[3] = position_q[3];
	_quart.quart_inverse(position_q_adjust);
}

// 解算姿态调整
void Kalman_AHRS::posit_adjust()
{
	position_q_new[0] = position_q[0];
	position_q_new[1] = position_q[1];
	position_q_new[2] = position_q[2];
	position_q_new[3] = position_q[3];
	
	if(position_function & (1 << 1))
		_quart.quart_multi_q(position_q_new, position_q_adjust);

	_quart.quart_to_eular(position_q_new, position_e_new);
}

void Kalman_AHRS::posit_acc(Vector3f *avalf)
{
	Quarternion qa = {0, 0, 0, -CONSTANTS_ONE_G};
	Quarternion qb;
	qb[0] = position_q[0];
	qb[1] = position_q[1];
	qb[2] = position_q[2];
	qb[3] = position_q[3];
	_quart.quart_inverse(qb);
	_quart.quart_multi_q(qb, qa);
	_quart.quart_multi_q(qb, position_q);
	acc_relative[0] = qb[1] + avalf->x;
	acc_relative[1] = qb[2] + avalf->y;
	acc_relative[2] = qb[3] + avalf->z;
}

void Kalman_AHRS::posit_proc_gyro(Vector3f *gvalf)
{
	Quarternion q;
	Eular e;
	
	if(position_function & (1 << 5))
		return;
	
	e[0] = gvalf->x / sample_rate;
	e[1] = gvalf->y / sample_rate;
	e[2] = gvalf->z / sample_rate;
	_quart.quart_from_eular_d(q, e);
	_quart.quart_multi_q(position_q, q);
	_quart.quart_normalize(position_q);

	if(!(position_function & (1 << 6)))
		position_q[3] = 0;
		
	_quart.quart_to_eular(position_q, position_e);
}

void Kalman_AHRS::posit_proc_gyro_mul2(Vector3f *gvalf)
{
	Quarternion q;
	Eular e;
	
	if(position_function & (1 << 5))
		return;
	
	e[0] = gvalf->x / (sample_rate / 2);
	e[1] = gvalf->y / (sample_rate / 2);
	e[2] = gvalf->z / (sample_rate / 2);
	_quart.quart_from_eular_d(q, e);
	_quart.quart_multi_q(position_q, q);
	_quart.quart_normalize(position_q);
	
	if(!(position_function & (1 << 6)))
		position_q[3] = 0;

	_quart.quart_to_eular(position_q, position_e);
}

// 与加速度姿态值，进行自适应线性插值滤波，直接用四元数插值，无万向锁
void Kalman_AHRS::posit_proc_acce_magn(Vector3f *gvalf, Vector3f *avalf, Vector3f *mvalf)
{
	Quarternion qacc;
	Eular eacc;
	float a = 0, b = 0, r, g;
	
	if(position_function & (1 << 4))
	{
		eacc[0] = atan2f(avalf->y, avalf->z);
		eacc[1] = atan2f(-avalf->x, __sqrtf(avalf->y * avalf->y + avalf->z * avalf->z));
		eacc[2] = atan2f(
					-(mvalf->y * arm_cos_f32(eacc[0]) - mvalf->z * arm_sin_f32(eacc[0])),
					(mvalf->x * arm_cos_f32(eacc[1])
					+ mvalf->y * arm_sin_f32(eacc[1]) * arm_sin_f32(eacc[0])
					+ mvalf->z * arm_sin_f32(eacc[1]) * arm_cos_f32(eacc[0])));
					
		_quart.quart_from_eular(qacc, eacc);
		a = fabsf(avalf->x * avalf->x + avalf->y * avalf->y + avalf->z * avalf->z - CONSTANTS_ONE_G * CONSTANTS_ONE_G);
		b = fabsf(mvalf->x * mvalf->x + mvalf->y * mvalf->y + mvalf->z * mvalf->z - M_DEFINE * M_DEFINE);
		r = 36 * (expf(b) - 1) + 25 * (expf(a) - 1) + 25;
	}
	else
	{
		Quarternion qg;
		Quarternion qa;
		_quart.quart_set(qg, 0, 0, 0, -1);
		_quart.quart_set(qa, 0, avalf->x, avalf->y, -avalf->z);
		_quart.quart_from_vector(qacc, qg, qa);
		a = fabsf(avalf->x * avalf->x + avalf->y * avalf->y + avalf->z * avalf->z - CONSTANTS_ONE_G * CONSTANTS_ONE_G);
		r = 25 * (expf(a) - 1) + 25;
	}
	
	g = 0.2f * __sqrtf(gvalf->x * gvalf->x + gvalf->y * gvalf->y + gvalf->z * gvalf->z) + 0.07f;
	
	if(position_function & (1 << 5))
		g = 8;

	_quart.quart_slerp(position_q, position_q, qacc, g / r);
	_quart.quart_to_eular(position_q, position_e);
}

//陀螺仪转四元数积分后，再与加速度姿态值进行卡尔曼滤波，最后再转成四元数
//运算量较大，效果较好
void Kalman_AHRS::posit_proc2(Vector3f *avalf, Vector3f *gvalf, Vector3f *mvalf)
{
	Quarternion q;
	Eular e;
	float roll, pitch, yaw, a, b;
	
	roll = atan2f(avalf->y, avalf->z);
	pitch = atan2f(-avalf->x, __sqrtf(avalf->y * avalf->y + avalf->z * avalf->z));
	yaw = atan2f(
                -(mvalf->y * arm_cos_f32(roll) - mvalf->z * arm_sin_f32(roll)), 
                (mvalf->x * arm_cos_f32(pitch)
				+ mvalf->y * arm_sin_f32(pitch) * arm_sin_f32(roll)
				+ mvalf->z * arm_sin_f32(pitch) * arm_cos_f32(roll))
                );

	e[0] = gvalf->x / sample_rate;
	e[1] = gvalf->y / sample_rate;
	e[2] = gvalf->z / sample_rate;
	_quart.quart_from_eular_d(q, e);
	_quart.quart_multi_q(position_q, q);
	_quart.quart_normalize(position_q);

	_quart.quart_to_eular(position_q, position_e);
	
	a = fabsf(avalf->x * avalf->x + avalf->y * avalf->y + avalf->z * avalf->z - CONSTANTS_ONE_G * CONSTANTS_ONE_G);
	b = fabsf(mvalf->x * mvalf->x + mvalf->y * mvalf->y + mvalf->z * mvalf->z - M_DEFINE * M_DEFINE);
	
	kalman[0].R = 3 * b + 7 * a + expf(0.1f * avalf->x * avalf->x + 0.02f * avalf->y * avalf->y) + 200;
	kalman[1].R = kalman[0].R;
	kalman[2].R = kalman[0].R;

	position_e[0] = position_e[0];//kalman_mix_ring(&kalman[0], position_e[0], roll, PI / 2);
	position_e[1] = position_e[1];//kalman_mix(&kalman[1], position_e[1], pitch);
	position_e[2] = position_e[2];//kalman_mix_ring(&kalman[2], position_e[2], yaw, PI / 2);
	
	_quart.quart_from_eular(position_q, position_e);

}

//加速度姿态值减上次姿态值，算出角速度观测值，与陀螺仪的四元数进行卡尔曼滤波，最后再转成欧拉角
//运算量较小，效果一般
void Kalman_AHRS::posit_proc1(Vector3f *avalf, Vector3f *gvalf)
{
	Quarternion q;
	Eular e;
	float roll, pitch, roll_d, pitch_d, a;
	float roll_m, pitch_m, yaw_m, updown;
	
	roll = atan2f(avalf->y, avalf->z);
	pitch = atan2f(-avalf->x, sqrt(avalf->y * avalf->y + avalf->z * avalf->z));

	roll_d = sub_ring(roll, position_e[0], PI / 2);
	pitch_d = sub_ring(pitch, position_e[1], PI / 4);
	
	a = fabsf(avalf->x * avalf->x + avalf->y * avalf->y + avalf->z * avalf->z - CONSTANTS_ONE_G * CONSTANTS_ONE_G);
	kalman[0].R = (5 * a + expf(0.3f * avalf->x * avalf->x + 0.2f * avalf->y * avalf->y) - 1);
	kalman[1].R = kalman[0].R;

	roll_m = kalman_mix_ring(&kalman[0], gvalf->x / sample_rate, roll_d, PI / 2);
	arm_sqrt_f32(position_q[1] * position_q[1] + position_q[2] * position_q[2], &updown);
	if (updown > PI / 4)
		pitch_d = -pitch_d;
	pitch_m = kalman_mix_ring(&kalman[1], gvalf->y / sample_rate, pitch_d, PI / 4);
	yaw_m = gvalf->z / sample_rate;

	e[0] = roll_m;
	e[1] = pitch_m;
	e[2] = yaw_m;
	_quart.quart_from_eular_d(q, e);
	_quart.quart_multi_q(position_q, q);
	_quart.quart_normalize(position_q);
	_quart.quart_to_eular(position_q, position_e);
}

void Kalman_AHRS::posit_proc_init(Vector3f *avalf, Vector3f *mvalf)
{	
	position_e[0] = atan2f(avalf->y, avalf->z);
	position_e[1] = atan2f(-avalf->x, __sqrtf(avalf->y * avalf->y + avalf->z * avalf->z));
	position_e[2] = 0;

	_quart.quart_from_eular(position_q, position_e);

	/*
    kalman[0].P = 100.0f;
	kalman[0].Q = 0.000014f;
	kalman[0].R = 1.0f;

	kalman[1].P = 100.0f;
	kalman[1].Q = 0.000014f;
	kalman[1].R = 1.0f;

	kalman[2].P = 100.0f;
	kalman[2].Q = 0.000014f;
	kalman[2].R = 1.0f;
    */
}

void Kalman_AHRS::posit_eular(Vector3f *eular, Vector3f *rad)
{
    eular->x = position_e_new[0] * RAD_TO_DEG;
    eular->y = position_e_new[1] * RAD_TO_DEG;
    eular->z = position_e_new[2] * RAD_TO_DEG;
    
    rad->x = position_e_new[0];
    rad->y = position_e_new[1];
    rad->z = position_e_new[2];
}

/* -------------------------------------------------------------------------- */
/*                                  Mahony                                    */
/* -------------------------------------------------------------------------- */
// table of user settable parameters
const AP_Param::GroupInfo MahonyAHRS::var_info[] PROGMEM =
{
		AP_GROUPINFO("M_P", 0, MahonyAHRS, Kp ,0.8),
	
		AP_GROUPINFO("M_I", 1, MahonyAHRS, Ki ,0.05),
    
		AP_GROUPEND
};

float MahonyAHRS::invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

float MahonyAHRS::atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (PI / 2.0f) - res;
    if (x < 0) res = PI - res;
    if (y < 0) res = -res;
    return res;
}

float MahonyAHRS::sin_approx(float x)
{
#define sinPolyCoef3 -1.666665710e-1f                                          // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f                                          // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f                                          // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f                                          // Double:  2.600054767890361277123254766503271638682e-6
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                     // Stop here on error input (5 * 360 Deg)
    while (x >  PI) x -= (2.0f * PI);                             // always wrap input angle to -PI..PI
    while (x < -PI) x += (2.0f * PI);
    if (x >  (0.5f * PI)) x =  (0.5f * PI) - (x - (0.5f * PI));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * PI)) x = -(0.5f * PI) - ((0.5f * PI) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

#define DEGREES_TO_DECIDEGREES(angle) (angle * 10)
#define DECIDEGREES_TO_DEGREES(angle) (angle / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle / 10.0f) * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)

float MahonyAHRS::acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return PI - result;
    else
        return result;
}

MahonyAHRS::MahonyAHRS(float kp, float ki)
{
	q0 = 1.0f;
	q1=q2=q3 = 0.0f;
	
	Kp = kp;
	Ki = ki;
	
	accTimeSum = 0;
	accSumCount = 0;
}

void MahonyAHRS::MahonyAHRSUpdate(float gx, float gy, float gz,  float ax, float ay, float az,  float yawError)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
	
    float recipNorm;	// 
	static uint32_t tnow, tPrev;
	float dt;
    uint32_t count;
	
    //float hx, hy, bx;
    float vx,vy,vz;
    float ex = 0, ey = 0, ez = 0;
    
	float qa, qb, qc;
    
	tnow = SysTick->VAL;
    count = (tnow > tPrev)?(SysTick->LOAD + tPrev - tnow) : (tPrev - tnow);
    dt = count / US_T;
    dt = dt / 1000000.0f;
    tPrev=tnow;
    if(dt < 0.0001f)
    {
        dt = 0.00025f;
    }
    yawError = To_180_degrees(yawError);
    //
    float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));
    
    // yaw
	while (yawError >  PI) yawError -= (2.0f * PI);
	while (yawError < -PI) yawError += (2.0f * PI);
	ez += sin_approx(yawError / 2.0f);

	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	/* ---------------------------------------------------------------------- */
    recipNorm = sq(ax) + sq(ay) + sq(az);
    if (recipNorm > 0.01f) {
        
        recipNorm = invSqrt(recipNorm);
		
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

		ex = (ay*vz - az*vy);
		ey = (az*vx - ax*vz);
		//ez += (ax*vy - ay*vx);
		
    }
	
	/* ---------------------------------------------------------------------- */
#define SPIN_RATE_LIMIT 20
    
    if(Ki > 0.0f) {
        
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))
		{
			float dcmKiGain = Ki;
			
            integralFBx += dcmKiGain * ex * dt;
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    }
    else
	{
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

	float dcmKpGain = Kp;

    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void MahonyAHRS::request_eular(Vector3f *eular, Vector3f *rad)
{
    rad->y = asinf(2*q1*q3 - 2*q0*q2);								// pitch
    rad->x = atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1);		// roll
    rad->z = -atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1);		// yaw

    eular->x = rad->x * RAD_TO_DEG;
    eular->y = rad->y * RAD_TO_DEG;
    eular->z = rad->z * RAD_TO_DEG;
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (ABS(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

void MahonyAHRS::BodyToEarth(t_fp_vector * v)
{
    float x,y,z;

    /* From body frame to earth frame */
    x = rMat[0][0] * v->V.X + rMat[0][1] * v->V.Y + rMat[0][2] * v->V.Z;
    y = rMat[1][0] * v->V.X + rMat[1][1] * v->V.Y + rMat[1][2] * v->V.Z;
    z = rMat[2][0] * v->V.X + rMat[2][1] * v->V.Y + rMat[2][2] * v->V.Z;

    v->V.X = x;
    v->V.Y = -y;
    v->V.Z = z;
}

float MahonyAHRS::To_180_degrees(float x)
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}

