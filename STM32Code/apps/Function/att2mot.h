#ifndef __ATT2MOT_H__
#define __ATT2MOT_H__

#include <AP_Math.h>

// Matrix operation
class ChMatrix3
{
	public:
		float m[3][3];
	  ChMatrix3();
	  ChMatrix3(float a00, float a01, float a02, float a10, float a11, float a12, float a20, float a21, float a22);
	
	Vector3f row(int i) { Vector3f vec; vec.x = m[i][0]; vec.y = m[i][1]; vec.z = m[i][2]; return vec;}
	Vector3f col(int j) { Vector3f vec; vec.x = m[0][j]; vec.y = m[1][j]; vec.z = m[2][j]; return vec;}
	float get(int i, int j){ return m[i][j];}
	void set(int i, int j, float value) { m[i][j] = value;}
	void eye(void);
	void set_all(float a, float b, float c, float d, float e, float f, float g, float h, float i);
	void from_quat(float q0, float q1, float q2, float q3);
	void left_rot_x_180deg(void);
	
	ChMatrix3 operator*(const ChMatrix3& B);
};

class KF
{
	public:
		KF(float q, float r);
		float input;
	  float last_x;
	  float dx;
	  float p;
	  float k;
	  float q;
	  float r;
	  float output;
	  void init(void);
	  float update(float input);
};

void transpose(ChMatrix3 *A, ChMatrix3 *At);
void inv(ChMatrix3 *A, ChMatrix3 *A_);
void mul(ChMatrix3 *A, ChMatrix3 *B, ChMatrix3 *C);
Vector3f mul_vec(ChMatrix3 *A, Vector3f *vec);

// Attitude map to motors
// differential move
Vector3f pryrot2dangle(ChMatrix3 ROT, float enc_x, float enc_y, float enc_z);

ChMatrix3 quat2dcm(float q0, float q1, float q2, float q3);


void get_body_dcm(Vector3f enc, ChMatrix3 A, ChMatrix3 *A_b);


// Map attitude control to 3 motors
Vector3f speed_decouple(Vector3f enc, Vector3f speed, Vector3f enc_speed);

// Tracking kalman
void tracking_init(void);
// Tracking update 
float tracking_update(Vector3f *euler, Vector3f enc, Vector3f dead_zone, Vector3f &angle_feedback);

Vector3f get_angle_err_bodyframe(Vector3f setpoint, Vector3f euler, Vector3f enc);

Vector3f get_angle_err_body_tracking(Vector3f setpoint, Vector3f euler, Vector3f enc, Vector3f deadzone);

Vector3f get_angle_err_body_tracking_t2(Vector3f setpoint, Vector3f euler, Vector3f enc, Vector3f deadzone);

Vector3f get_angle_err_body_xy(Vector3f setpoint, Vector3f euler, Vector3f enc);

Vector3f get_angle_err_body_quat(float q0, float q1, float q2, float q3, float angle_zero, Vector3f setpoint, Vector3f enc);

Vector3f get_tracking_angle_e(Vector3f setpoint, Vector3f euler, Vector3f enc, Vector3f deadzone);

Vector3f vector_decouple(Vector3f v, Vector3f enc);

void vel_remap(Vector3f enc, Quaternion q, Vector3f vel_drone, Vector3f* vel_camera, Vector3f* acc_camera);

void vel_remap_t2(Vector3f enc, Quaternion q, Vector3f euler, Vector3f vel_drone, Vector3f* vel_camera, Vector3f* acc_camera);

Quaternion drone_quat_compensation(float q0, float q1, float q2, float q3, Quaternion drone_quat, Vector3f enc, float k, Vector3f *euler);
#endif
