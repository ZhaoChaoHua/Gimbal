#include "att2mot.h"

KF kf(0.0001, 0.04);

#define RAD2DEG   57.295779513082320f        // deg = rad * RAD2DEG

ChMatrix3::ChMatrix3()
{
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			this->m[i][j] = (i==j?1:0);
		}
	}
}


ChMatrix3::ChMatrix3(float a00, float a01, float a02, float a10, float a11, float a12, float a20, float a21, float a22)
{
	this->m[0][0] = a00;
	this->m[0][1] = a01;
	this->m[0][2] = a02;
	
	this->m[1][0] = a10;
	this->m[1][1] = a11;
	this->m[1][2] = a12;
	
	this->m[2][0] = a20;
	this->m[2][1] = a21;
	this->m[2][2] = a22;
}

void ChMatrix3::set_all(float a, float b, float c, float d, float e, float f, float g, float h, float i)
{
	m[0][0] = a;
	m[0][1] = b;
	m[0][2] = c;
	m[1][0] = d;
	m[1][1] = e;
	m[1][2] = f;
	m[2][0] = g;
	m[2][1] = h;
	m[2][2] = i;
}

void ChMatrix3::eye(void)
{
	this->m[0][0] = 1;
	this->m[0][1] = 0;
	this->m[0][2] = 0;
	
	this->m[1][0] = 0;
	this->m[1][1] = 1;
	this->m[1][2] = 0;
	
	this->m[2][0] = 0;
	this->m[2][1] = 0;
	this->m[2][2] = 1;
}

void ChMatrix3::from_quat(float q0, float q1, float q2, float q3)
{
	float qr = q0;
	float qi = q1;
	float qj = q2;
	float qk = q3;
	float qii = qi*qi;
	float qjj = qj*qj;
	float qkk = qk*qk;
	float s = 1.0f;
	
	m[0][0] = 1-2*s*(qjj+qkk);
	m[0][1] = 2*s*(qi*qj-qk*qr);
	m[0][2] = 2*s*(qi*qk+qj*qr);
	
	m[1][0] = 2*s*(qi*qj+qk*qr);
	m[1][1] = 1-2*s*(qii+qkk);
	m[1][2] = 2*s*(qj*qk-qi*qr);
	
	m[2][0] = 2*s*(qi*qk-qj*qr);
	m[2][1] = 2*s*(qj*qk+qi*qr);
	m[2][2] = 1-2*s*(qii+qjj);
}

void transpose(ChMatrix3 *A, ChMatrix3 *At)
{
	At->m[0][0] = A->m[0][0];
	At->m[0][1] = A->m[1][0];
	At->m[0][2] = A->m[2][0];
	
	At->m[1][0] = A->m[0][1];
	At->m[1][1] = A->m[1][1];
	At->m[1][2] = A->m[2][1];
	
	At->m[2][0] = A->m[0][2];
	At->m[2][1] = A->m[1][2];
	At->m[2][2] = A->m[2][2];
}
	

void inv(ChMatrix3 *A, ChMatrix3 *A_)
{
	float det;
	float a11, a12, a13, a21, a22, a23, a31, a32, a33;
	a11 = A->m[0][0];
	a12 = A->m[0][1];
	a13 = A->m[0][2];
	a21 = A->m[1][0];
	a22 = A->m[1][1];
	a23 = A->m[1][2];
	a31 = A->m[2][0];
	a32 = A->m[2][1];
	a33 = A->m[2][2];
	det = a11*a22*a33 + a21*a32*a13 + a31*a12*a23 - a11*a32*a23 - a31*a22*a13 - a21*a12*a33;
	if(det != 0)
	{
		A_->m[0][0] = (a22*a33-a23*a32)/det;
		A_->m[0][1] = (a13*a32-a12*a33)/det;
		A_->m[0][2] = (a12*a23-a13*a22)/det;
		
		A_->m[1][0] = (a23*a31-a21*a33)/det;
		A_->m[1][1] = (a11*a33-a13*a31)/det;
		A_->m[1][2] = (a13*a21-a11*a23)/det;
		
		A_->m[2][0] = (a21*a32-a22*a31)/det;
		A_->m[2][1] = (a12*a31-a11*a32)/det;
		A_->m[2][2] = (a11*a22-a12*a21)/det;
	}
}


void mul(ChMatrix3 *A, ChMatrix3 *B, ChMatrix3 *C)
{
	float c00, c01, c02, c10, c11, c12, c20, c21, c22;
	c00 = A->m[0][0] * B->m[0][0] + A->m[0][1]*B->m[1][0] + A->m[0][2]*B->m[2][0];
	c01 = A->m[0][0] * B->m[0][1] + A->m[0][1]*B->m[1][1] + A->m[0][2]*B->m[2][1];
	c02 = A->m[0][0] * B->m[0][2] + A->m[0][1]*B->m[1][2] + A->m[0][2]*B->m[2][2];
	
	c10 = A->m[1][0] * B->m[0][0] + A->m[1][1]*B->m[1][0] + A->m[1][2]*B->m[2][0];
	c11 = A->m[1][0] * B->m[0][1] + A->m[1][1]*B->m[1][1] + A->m[1][2]*B->m[2][1];
	c12 = A->m[1][0] * B->m[0][2] + A->m[1][1]*B->m[1][2] + A->m[1][2]*B->m[2][2];
	
	c20 = A->m[2][0] * B->m[0][0] + A->m[2][1]*B->m[1][0] + A->m[2][2]*B->m[2][0];
	c21 = A->m[2][0] * B->m[0][1] + A->m[2][1]*B->m[1][1] + A->m[2][2]*B->m[2][1];
	c22 = A->m[2][0] * B->m[0][2] + A->m[2][1]*B->m[1][2] + A->m[2][2]*B->m[2][2];
	
	C->m[0][0] = c00;
	C->m[0][1] = c01;
	C->m[0][2] = c02;
	
	C->m[1][0] = c10;
	C->m[1][1] = c11;
	C->m[1][2] = c12;
	
	C->m[2][0] = c20;
	C->m[2][1] = c21;
	C->m[2][2] = c22;		
}

ChMatrix3 ChMatrix3::operator*(const ChMatrix3& B)
{
	ChMatrix3 C;
	float c00, c01, c02, c10, c11, c12, c20, c21, c22;
	c00 = this->m[0][0] * B.m[0][0] + this->m[0][1]*B.m[1][0] + this->m[0][2]*B.m[2][0];
	c01 = this->m[0][0] * B.m[0][1] + this->m[0][1]*B.m[1][1] + this->m[0][2]*B.m[2][1];
	c02 = this->m[0][0] * B.m[0][2] + this->m[0][1]*B.m[1][2] + this->m[0][2]*B.m[2][2];
	
	c10 = this->m[1][0] * B.m[0][0] + this->m[1][1]*B.m[1][0] + this->m[1][2]*B.m[2][0];
	c11 = this->m[1][0] * B.m[0][1] + this->m[1][1]*B.m[1][1] + this->m[1][2]*B.m[2][1];
	c12 = this->m[1][0] * B.m[0][2] + this->m[1][1]*B.m[1][2] + this->m[1][2]*B.m[2][2];
	
	c20 = this->m[2][0] * B.m[0][0] + this->m[2][1]*B.m[1][0] + this->m[2][2]*B.m[2][0];
	c21 = this->m[2][0] * B.m[0][1] + this->m[2][1]*B.m[1][1] + this->m[2][2]*B.m[2][1];
	c22 = this->m[2][0] * B.m[0][2] + this->m[2][1]*B.m[1][2] + this->m[2][2]*B.m[2][2];
		
	C.set_all(c00, c01, c02,
	          c10, c11, c12,
	          c20, c21, c22);
	return C;
}

void ChMatrix3::left_rot_x_180deg(void)
{
	m[0][1] = -m[0][1];
	m[0][2] = -m[0][2];
	m[1][0] = -m[1][0];
	m[2][0] = -m[2][0];
}

Vector3f mul_vec(ChMatrix3 *A, Vector3f *vec)
{
	Vector3f v;
	v.x = A->m[0][0]*vec->x + A->m[0][1]*vec->y + A->m[0][2]*vec->z;
	v.y = A->m[1][0]*vec->x + A->m[1][1]*vec->y + A->m[1][2]*vec->z;
	v.z = A->m[2][0]*vec->x + A->m[2][1]*vec->y + A->m[2][2]*vec->z;
	return v;
}

void correct_angle(Vector3f *angle, Vector3f *langle)
{
	if(angle->x - langle->x > 1.5f*PI)
	{
		while(angle->x - langle->x > 1.5f*PI) angle->x -= 2*PI;
	}
	else if(angle->x - langle->x < -1.5f*PI)
	{
		while(angle->x - langle->x < -1.5f*PI) angle->x += 2*PI;
	}
	
	if(angle->y - langle->y > 1.5f*PI)
	{
		while(angle->y - langle->y > 1.5f*PI) angle->y -= 2*PI;
	}
	else if(angle->y - langle->y < -1.5f*PI)
	{
		while(angle->y - langle->y < -1.5f*PI) angle->y += 2*PI;
	}
	
	if(angle->z - langle->z > 1.5f*PI)
	{
		while(angle->z - langle->z > 1.5f*PI) angle->z -= 2*PI;
	}
	else if(angle->z - langle->z < -1.5f*PI)
	{
		while(angle->z - langle->z < -1.5f*PI) angle->z += 2*PI;
	}
}

Vector3f vector_fusion(Vector3f v1, Vector3f v2, float k1)
{
	Vector3f v;
	if(k1<0.0f) k1 = 0.0f;
	else if(k1>1) k1 = 1.0f;
	float k2 = 1-k1;
	v.x = k1*v1.x + k2*v2.x;
	v.y = k1*v1.y + k2*v2.y;
	v.z = k1*v1.z + k2*v2.z;
	return v;
}

float absf(float x)
{
	return x < 0 ? -x : x;
}

ChMatrix3 quat2dcm(float q0, float q1, float q2, float q3)
{
	ChMatrix3 dcm;
	float qr = q0;
	float qi = q1;
	float qj = q2;
	float qk = q3;
	float qii = qi*qi;
	float qjj = qj*qj;
	float qkk = qk*qk;
	float s = 1.0f;
	
	dcm.m[0][0] = 1-2*s*(qjj+qkk);
	dcm.m[0][1] = 2*s*(qi*qj-qk*qr);
	dcm.m[0][2] = 2*s*(qi*qk+qj*qr);
	
	dcm.m[1][0] = 2*s*(qi*qj+qk*qr);
	dcm.m[1][1] = 1-2*s*(qii+qkk);
	dcm.m[1][2] = 2*s*(qj*qk-qi*qr);
	
	dcm.m[2][0] = 2*s*(qi*qk-qj*qr);
	dcm.m[2][1] = 2*s*(qj*qk+qi*qr);
	dcm.m[2][2] = 1-2*s*(qii+qjj);
	return dcm;
}

void euler2dcm_zxy(Vector3f e, ChMatrix3* m)
{
	float sx = sinf(e.x);
	float cx = cosf(e.x);
	float sy = sinf(e.y);
	float cy = cosf(e.y);
	float sz = sinf(e.z);
	float cz = cosf(e.z);
	m->m[0][0] = cz*cy - sz*sx*sy;
	m->m[0][1] = -sz*cx;
	m->m[0][2] = cz*sy + sz*sx*cy;
	m->m[1][0] = sz*cy + cz*sx*sy;
	m->m[1][1] = cz*cx;
	m->m[1][2] = sz*sy - cz*sx*cy;
	m->m[2][0] = -cx*sy;
	m->m[2][1] = sx;
	m->m[2][2] = cx*cy;
}

void euler2dcm_xy(Vector3f e, ChMatrix3 *m)
{
	float sx = sinf(e.x);
	float cx = cosf(e.x);
	float sy = sinf(e.y);
	float cy = cosf(e.y);
	float sz = sinf(e.z);
	float cz = cosf(e.z);
	m->set_all(cy, 0.0f, sy,
	          sx*sy, cx, -sx*cy,
	          -cx*sy, sx, cx*cy);
}

Vector3f dcm2euler_zxy(ChMatrix3 m)
{
	Vector3f v;
	v.x = asinf(m.m[2][1]);
	v.y = atan2f(-m.m[2][0], m.m[2][2]);
	v.z = atan2f(-m.m[0][1], m.m[1][1]);
	return v;
}

Vector3f decouple_full(Vector3f axis, Vector3f euler)
{
	Vector3f v;
	v.x = cosf(axis.y) * euler.x + sinf(axis.y) * euler.z;
	v.y = sinf(axis.y) * tanf(axis.x) * euler.x + euler.y + cosf(axis.y) * tanf(axis.x) * euler.z;
	v.z = -sinf(axis.y) * euler.x / cosf(axis.x) + cosf(axis.y) * euler.z /cos(axis.x);
	return v;
}

float loop_limit(float x, float lim)
{
	if(x > lim) x -= 2.0f * lim;
	else if(x < -lim) x += 2.0f * lim;
	return x;
}

Vector3f loop_limit_vec3(Vector3f v, float lim)
{
	Vector3f y;
	y.x = loop_limit(v.x, lim);
	y.y = loop_limit(v.y, lim);
	y.z = loop_limit(v.z, lim);
	return y;
}

int scop_d_angle_x, scop_d_angle_y, scop_d_angle_z;
int scop_angle_ex, scop_angle_ey, scop_angle_ez;
Vector3f get_angle_err_bodyframe(Vector3f setpoint, Vector3f euler, Vector3f enc)
{
	ChMatrix3 Ad, An, An_t, T, Tbc, Temp;
	Vector3f angle_e, angle;
	euler2dcm_zxy(enc, &Tbc);
	euler2dcm_zxy(setpoint, &Ad);
	euler2dcm_zxy(euler, &An);
	transpose(&An, &An_t);
	mul(&Tbc, &An_t, &Temp);
	mul(&Temp, &Ad, &T);
	angle = dcm2euler_zxy(T);
	angle = loop_limit_vec3(angle, PI);
	angle_e = angle - enc;
	angle_e = loop_limit_vec3(angle_e, PI);
	
  scop_d_angle_x = angle.x * 5729.0f;
	scop_d_angle_y = angle.y * 5729.0f;
	scop_d_angle_z = setpoint.z * 5729.0f;
	scop_angle_ex = angle_e.x * 5729.0f;
	scop_angle_ey = angle_e.y * 5729.0f;
	scop_angle_ez = angle_e.z * 5729.0f;
	
	return angle_e;
}

ChMatrix3 dcm_rot_z(ChMatrix3 dcm, float angle_rad)
{
	ChMatrix3 rot, dcm_out;
	float sz = sinf(angle_rad);
	float cz = cosf(angle_rad);
	rot.set_all(cz, -sz, 0, 
	            sz,  cz, 0,
	             0,   0, 1);
	mul(&rot, &dcm, &dcm_out);
	return dcm_out;
}
	

Vector3f get_angle_err_body_quat(float q0, float q1, float q2, float q3, float angle_zero, Vector3f setpoint, Vector3f enc)
{
	ChMatrix3 Ad, An, An_t, T, Tbc, Temp;
	Vector3f angle_e, angle;
	euler2dcm_zxy(enc, &Tbc);
	euler2dcm_zxy(setpoint, &Ad);
	An = quat2dcm(q0, q1, q2, q3);
	An = dcm_rot_z(An, -angle_zero);
	transpose(&An, &An_t);
	mul(&Tbc, &An_t, &Temp);
	mul(&Temp, &Ad, &T);
	angle = dcm2euler_zxy(T);
	angle_e = angle - enc;

	return angle_e;
}
	

Vector3f get_angle_err_body_xy(Vector3f setpoint, Vector3f euler, Vector3f enc)
{
	ChMatrix3 A_de, A_ne, A_db, R_a, A_nt, Temp;
	Vector3f angle, angle_err;
	euler2dcm_zxy(setpoint, &A_de);
	euler2dcm_xy(euler, &A_ne);
	euler2dcm_zxy(enc, &R_a);
	transpose(&A_ne, &A_nt);
	mul(&R_a, &A_nt, &Temp);
	mul(&Temp, &A_de, &A_db);
	angle = dcm2euler_zxy(A_db);
//	angle = loop_limit_vec3(angle, PI);
	angle_err = angle - enc;
//	angle_err = loop_limit_vec3(angle, PI);
	return angle_err;
}


Vector3f get_tracking_angle_e(Vector3f setpoint, Vector3f euler, Vector3f enc, Vector3f deadzone)
{
	Vector3f angle_be, angle, angle_e, angle_c;
	ChMatrix3 A_ne, A_ne_t, A_cb, A_cb_t, A_be, A_de, T, Temp;
	float k_filter = 0.02;
	
	euler2dcm_xy(euler, &A_ne);
	euler2dcm_zxy(enc, &A_cb);
	transpose(&A_cb, &A_cb_t);
	mul(&A_ne, &A_cb_t, &A_be);
	angle_be = dcm2euler_zxy(A_be);
	angle_be = loop_limit_vec3(angle_be, PI);
	
	k_filter = angle_be.z * 0.01 / deadzone.z;
	if(k_filter < 0) k_filter = -k_filter;
	angle_be.z = -k_filter*angle_be.z;
	
	angle(euler.x, euler.y, angle_be.z);
	euler2dcm_zxy(angle, &A_ne);
	euler2dcm_zxy(setpoint, &A_de);
	transpose(&A_ne, &A_ne_t);
	mul(&A_ne_t, &A_de, &T);
	angle_e = dcm2euler_zxy(T);

	return angle_e;
}


Vector3f vector_decouple(Vector3f v, Vector3f enc)
{
	float sx, cx, sy, cy;
	Vector3f vo;
	sx = sinf(enc.x);
	sy = sinf(enc.y);
	cx = cosf(enc.x);
	cy = cosf(enc.y);
	
	vo.x = cy * v.x + sy * v.z;
	vo.y = v.y;
	vo.z = -cx * sy * v.x + sx * v.y + cx * cy * v.z;
	
	return vo;
}

int scop_drone2cam_accx, scop_drone2cam_accy, scop_drone2cam_accz;
void vel_remap(Vector3f enc, Quaternion q, Vector3f vel_drone, Vector3f* vel_camera, Vector3f* acc_camera)
{
	static Vector3f last_vel_camera;
	float freq = 40.0f;
	ChMatrix3 A_cb, A_bc, A_drone;
	 Vector3f v, vc;
	 A_drone = quat2dcm(-q.q1, q.q2, q.q3, q.q4);
	 v = mul_vec(&A_drone, &vel_drone);
	 v.y = -v.y;
	 v.z = -v.z;
	 euler2dcm_zxy(enc, &A_cb);
	 transpose(&A_cb, &A_bc);
	 vc = mul_vec(&A_bc, &v);
	 
	 vel_camera->x = vc.x;
	 vel_camera->y = vc.y;
	 vel_camera->z = vc.z;
	
	acc_camera->x = (vel_camera->x - last_vel_camera.x) * freq;
	acc_camera->y = (vel_camera->y - last_vel_camera.y) * freq;
	acc_camera->z = (vel_camera->z - last_vel_camera.z) * freq;
	
	last_vel_camera.x = vel_camera->x;
	last_vel_camera.y = vel_camera->y;
	last_vel_camera.z = vel_camera->z;
	
	scop_drone2cam_accx = acc_camera->x * 1000;
	scop_drone2cam_accy = acc_camera->y * 1000;
	scop_drone2cam_accz = acc_camera->z * 1000;
}

int scop_euler_camx, scop_euler_camy, scop_euler_camz;
void vel_remap_t2(Vector3f enc, Quaternion q, Vector3f euler, Vector3f vel_drone, Vector3f* vel_camera, Vector3f* acc_camera)
{
	ChMatrix3 A_b, A_bc, A_cb, A_c, A_cn;
	Vector3f euler_cam, v, v_c;
	static Vector3f last_v_c;
	float freq = 40.0f;
	
	euler2dcm_zxy(enc, &A_cb);
	A_b = quat2dcm(q.q1, q.q2, q.q3, q.q4);
	A_b.left_rot_x_180deg();
	mul(&A_b, &A_cb, &A_c);
	euler_cam = dcm2euler_zxy(A_c);
	euler.z = -euler_cam.z;
	euler.x = -euler.x;
	euler.y = -euler.y;
	
	euler2dcm_zxy(euler, &A_cn);
	v.x = vel_drone.x;
	v.y = -vel_drone.y;
	v.z = -vel_drone.z;
	v_c = mul_vec(&A_cn, &v);
	
	vel_camera->x = v_c.x;
	vel_camera->y = v_c.y;
	vel_camera->z = v_c.z;
	
	acc_camera->x = (vel_camera->x - last_v_c.x) * freq;
	acc_camera->y = (vel_camera->y - last_v_c.y) * freq;
	acc_camera->z = (vel_camera->z - last_v_c.z) * freq;
	
	last_v_c.x = vel_camera->x;
	last_v_c.y = vel_camera->y;
	last_v_c.z = vel_camera->z;
	
	scop_drone2cam_accx = acc_camera->x * 1000;
	scop_drone2cam_accy = acc_camera->y * 1000;
	scop_drone2cam_accz = acc_camera->z * 1000;
	
	
	scop_euler_camx = euler_cam.x * 57290;
	scop_euler_camy = euler_cam.y * 57290;
	scop_euler_camz = euler_cam.z * 57290;
}

int scop_cam_from_dronex, scop_cam_from_droney, scop_cam_from_dronez;
int scop_drone_eulerx, scop_drone_eulery, scop_drone_eulerz;
Quaternion drone_quat_compensation(float q0, float q1, float q2, float q3, Quaternion drone_quat, Vector3f enc, float k, Vector3f *euler)
{
	ChMatrix3 A_cb, A_bc, A_b, A_c;
	Vector3f euler_cam_from_drone, drone_euler;
	euler2dcm_zxy(enc, &A_cb);
	A_b = quat2dcm(drone_quat.q1, drone_quat.q2, drone_quat.q3, drone_quat.q4);
	A_b.left_rot_x_180deg();
	drone_euler = dcm2euler_zxy(A_b);
	euler2dcm_xy(drone_euler, &A_b);
	mul(&A_b, &A_cb, &A_c);
	
	euler_cam_from_drone = dcm2euler_zxy(A_c);
	euler->x = euler_cam_from_drone.x;
	euler->y = euler_cam_from_drone.y;
	euler->z = euler_cam_from_drone.z;
	
	scop_cam_from_dronex = euler_cam_from_drone.x * 57290;
	scop_cam_from_droney = euler_cam_from_drone.y * 57290;
	scop_cam_from_dronez = euler_cam_from_drone.z * 57290;
	
	scop_drone_eulerx = drone_euler.x * 57290;
	scop_drone_eulery = drone_euler.y * 57290;
	scop_drone_eulerz = drone_euler.z * 57290;
}


Quaternion attitude_compensation(float q0, float q1, float q2, float q3, Vector3f v)
{
	Quaternion q;
	Matrix3f rot_mat;
	float k = 0.7;
	q(q0, q1, q2, q3);
	q.rotation_matrix(rot_mat);
	v.x = k * v.x;
	v.y = k * v.y;
	v.z = k * v.z;
	rot_mat.rotate(v);
	q.from_rotation_matrix(rot_mat);
	return q;
}
	

int scop_dcm_angle_tsz, scop_dcm_angle_lsz;
Vector3f get_angle_err_body_tracking(Vector3f setpoint, Vector3f euler, Vector3f enc, Vector3f deadzone)
{
	static Vector3f last_tracking_setpoint, tracking_setpoint,  last_enc_angle, enc_angle;
  static Vector3f	enc_rot_cnt(0.0f, 0.0f, 0.0f);
	static Vector3f setpoint_rot_cnt(0.0f, 0.0f, 0.0f);

	static Vector3f angle, last_angle, last_angle_be, angle_be;
	Vector3f angle_e, angle_c;
	float k_tracking = 0.1;
	static bool tracking = false;
	static bool last_tracking = false;
	float stop_tracking_angle = 0.001f * deadzone.z;
	static float euler_set_z;
	static float tracking_angle;
	float tracking_step = 0.01f / RAD2DEG;
	float end_angle;
	Vector3f angle_e_lockhead, angle_e_output, lockhead_setpoint;
	
	ChMatrix3 A_ne, A_ne_t, A_cb, A_cb_t, A_be, A_de, T, Temp;
	
	// Correct angles
	last_tracking_setpoint = tracking_setpoint;
	last_enc_angle = enc_angle;
	tracking_setpoint = setpoint;
	enc_angle = enc;
	correct_angle(&tracking_setpoint, &last_tracking_setpoint);
	correct_angle(&enc_angle, &last_enc_angle);
	
	// Cal head of body
	angle_c = euler;
	euler2dcm_xy(angle_c, &A_ne);
	euler2dcm_zxy(enc_angle, &A_cb);

	transpose(&A_cb, &A_cb_t);
	mul(&A_ne, &A_cb_t, &A_be);
	last_angle_be = angle_be;
	angle_be = dcm2euler_zxy(A_be);
	correct_angle(&angle_be, &last_angle_be);
	
	
	// Tracking logic
	last_tracking = tracking;

	if(angle_be.z > deadzone.z || angle_be.z < -deadzone.z)	tracking = true;

	if(angle_be.z > -stop_tracking_angle && angle_be.z < stop_tracking_angle)	tracking = false;

	if(last_tracking && (!tracking))	euler_set_z = euler.z;
	
	if(tracking && (!last_tracking))	tracking_angle = setpoint.z/k_tracking;
	

	// Tracking error
	end_angle = k_tracking * kf.update(angle_be.z);
	if(tracking_angle > end_angle) tracking_angle -= tracking_step * deadzone.z;
	else if(tracking_angle < end_angle) tracking_angle += tracking_step * deadzone.z;
	
	tracking_setpoint.z = tracking_setpoint.z * k_tracking + tracking_angle;
	euler2dcm_zxy(tracking_setpoint, &A_de);
	transpose(&A_ne, &A_ne_t);
	mul(&A_cb ,&A_ne_t, &Temp);
	mul(&Temp, &A_de, &T);
	last_angle = angle;
	angle = dcm2euler_zxy(T);
	correct_angle(&angle, &last_angle);
	angle_e = angle - enc_angle;

	// Lockhead error
	lockhead_setpoint = setpoint;
	lockhead_setpoint.z += euler_set_z;
	angle_e_lockhead = get_angle_err_bodyframe(lockhead_setpoint, euler, enc);

	// Fusion error
	if(tracking) angle_e_output = angle_e;
	else angle_e_output = vector_fusion(angle_e, angle_e_lockhead, absf(angle_be.z*angle_be.z/deadzone.z*deadzone.z));

	scop_dcm_angle_tsz = tracking_setpoint.z * 5729.0f;
	scop_dcm_angle_lsz = lockhead_setpoint.z * 5729.0f;

	return angle_e_output;
}



void get_body_dcm(Vector3f enc, ChMatrix3 A, ChMatrix3 *A_b)
{
	float x = -enc.x;
	float y = -enc.y;
	float z = -enc.z;
	
	ChMatrix3 T( cos(y)*cos(z)+sin(y)*sin(x)*sin(z), -cos(y)*sin(z)+sin(y)*sin(x)*cos(z), sin(y)*cos(x),
	             cos(x)*sin(z), cos(x)*cos(z), -sin(x),
	             -sin(y)*cos(z)+cos(y)*sin(x)*sin(z), sin(y)*sin(z)+cos(y)*sin(x)*cos(z), cos(y)*cos(x));
	
	mul(&A, &T, A_b);
}

int js_sx, js_sy, js_sz;
int js_vx, js_vy, js_vz;
int js_vox, js_voy, js_voz;
int js_speed_z;
int js_gyro_z;

// Map the attitue speed to 3 motors
Vector3f speed_decouple(Vector3f enc, Vector3f vec, Vector3f enc_speed)
{
	Vector3f mot_out;
	float sx, cx, sy, cy, sz, cz, vx, vy, vz;
	static float enc_z_last, speed_z;
	
	speed_z = 0.15*enc_speed.z * 1000;
	enc_z_last = enc.z;
	js_speed_z = speed_z * 10000;
	js_gyro_z = vec.z * 10000;
	
	sx = sinf(enc.x);
	cx = cosf(enc.x);
	sy = sinf(enc.y);
	cy = cosf(enc.y);
	sz = sinf(enc.z);
	cz = cosf(enc.z);
	vx = vec.x;
	vy = vec.y;
	vz = vec.z;
	mot_out.x = cy*vx + sy*vz;
	mot_out.y = vy;
	mot_out.z = (-cx*sy)*vx + (sx)*vy + (cx*cy)*vz;
	
	js_sx = sx * 10000;
	js_sy = sy * 10000;
	js_sz = sz * 10000;
	
	js_vx = vx * 10000;
	js_vy = vy * 10000;
	js_vz = vz * 10000;
	
	js_vox = mot_out.x * 10000;
	js_voy = mot_out.y * 10000;
	js_voz = mot_out.z * 10000;

	return mot_out;
}

Vector3f angle_decouple(Vector3f enc, Vector3f angle, Vector3f euler)
{
	Vector3f angle_out;
	angle_out(cos(enc.y-euler.y)*angle.x + sin(enc.y-euler.y)*angle.z,
	        angle.y,
	        cos(enc.y-euler.y)*angle.z/cos(enc.x-euler.x) - sin(enc.y-euler.y)*angle.x/cos(enc.x-euler.x));///cos(enc.x));

	return angle_out;
}




// Kalman filter

KF::KF(float q, float r)
{
	this->q = q;
	this->r = r;
}

void KF::init(void)
{
	this->p = 0.1f;
	this->k = 0.1f;
	this->dx = 0.0f;
	this->last_x = 0.0f;
}

float KF::update(float input)
{
	float x_, p_;
	last_x = output;
	x_ = output;
	p_ = p + q;
	k = p_/(p_+r);
	output = x_ + k*(input - x_);
	p = p_*(1-k);
//	dx = output - last_x;
	return output;
}

void init_kf()
{
	kf.init();
}

float update_kf(float input)
{
	return kf.update(input);
}

// Tracking 
void tracking_init(void)
{
	kf.init();
}


int scop_tr_temp;
int scop_tr_k;
int scop_tr_angle;
int scop_tr_d, scop_tr_d_;
int scop_tr_set;
float tracking_update(Vector3f *euler, Vector3f enc, Vector3f dead_zone, Vector3f &angle_feedback)
{
	// Tracking state
	static bool tracking, last_tracking;
	// Output
	Vector3f fb;
	// Spring parameter, is proportional to the Force parameter
	float kspring = 0.6f;
	// Minimum force parameter
//	float k_min = 0.3f;
	// Zero position of tracking angle
	float tracking_zero = 0.1f/RAD2DEG;
	// Dead zone in rad
	float dead = dead_zone.z;//5.0f/RAD2DEG;
	// Zero position of angle setpoint
	float setpoint_zero = 0.1f/RAD2DEG;
	// Force parameter, is proportional to the pull back force
	float k_tracking;
	// Angle at this loop
	static float tracking_angle;
	static float tracking_setpoint, last_angle;
	// Velocity of the decrease of angle setpoint, it can be change 
	float setpoint_vel = 0.03f/RAD2DEG;
	//
	float koef_acc = 0.3f;
	
	// Update state and angle
	last_tracking = tracking;
	last_angle = tracking_angle;
	tracking_angle = enc.z*cos(enc.y-euler->y) + enc.x*sin(enc.y-euler->y);//kf.update(enc.z*cos(enc.y) + enc.x*sin(enc.y));
	
	// Jscope display
	scop_tr_angle = 100*tracking_angle * RAD2DEG;
	scop_tr_d = 1000*(enc.y-euler->y) * RAD2DEG;
	scop_tr_d_ = 1000*sin(enc.y-euler->y);
	scop_tr_set = 1000 * cos(enc.y-euler->y);
	
	// Enable Tracking mode if angle is bigger than dead zone
	if(tracking_angle > dead || tracking_angle < -dead)
	{
		tracking = true;
		// Update the setpoint angle when the state is not in tracking mode and angle is right on dead zone
		if(last_angle <= dead && last_angle >= -dead && last_tracking == false)
		{
			tracking_setpoint = tracking_angle;
		}
		// Compute pull back force parameter
		k_tracking = kspring*(tracking_angle - tracking_setpoint);
		// Make sure this parameter is positive
		if(k_tracking < 0) k_tracking *= -1.0f;
	}
	
	// When angle is smaller than dead zoon
	else
	{
		// Disable pull back force if the state is not in Tracking mode
		if(!tracking) k_tracking = 0.0f;
		// Keep compute the pull back fore parameter if the state is in Tracking mode
		else 
		{
			k_tracking = kspring*(tracking_angle - tracking_setpoint);
			// Make sure the force is positive
			if(k_tracking < 0) k_tracking *= -1.0f;
		}
	}
	
	// Disable tracking and clear euler yaw angle if enc angle is at zero
	if(tracking && tracking_angle < tracking_zero && tracking_angle > - tracking_zero) 
	{
		k_tracking = 0.0f;
		tracking = false;
		euler->z = 0.0f;
	}
	
	// Decrease setpoint angle for the next force calculation
	if(tracking_setpoint > setpoint_zero) tracking_setpoint -= setpoint_vel;
	else if(tracking_setpoint < -setpoint_zero) tracking_setpoint += setpoint_vel;
	scop_tr_k = 5729 * k_tracking * 0.1f * tracking_angle;
	
	// Calculate Output 
	angle_feedback(euler->x,
		             euler->y, 
		             k_tracking * koef_acc * tracking_angle);
	return k_tracking;
}







	
	
	
	
	
	

















