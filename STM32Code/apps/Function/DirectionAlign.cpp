#include "DirectionAlign.h"

#define RAD2DEG  57.2957828f

const AP_Param::GroupInfo DirectionAlign::var_info[] PROGMEM = {
	// Flag of not aligned
	AP_GROUPINFO("NOT_ALIGNED",        0, DirectionAlign, _not_aligned, 1.0f),
	// Encoder X direction
	AP_GROUPINFO("ENC_XDIR",    1, DirectionAlign, _enc_x_direction, 1.0f),
	// Encoder Y direction
	AP_GROUPINFO("ENC_YDIR",    2, DirectionAlign, _enc_y_direction, 1.0f),
	// Encoder Z direction
	AP_GROUPINFO("ENC_ZDIR",    3, DirectionAlign, _enc_z_direction, 1.0f),
	// Motor X direction
	AP_GROUPINFO("MOT_XDIR",    4, DirectionAlign, _mot_x_direction, 1.0f),
	// Motor X direction
	AP_GROUPINFO("MOT_YDIR",    5, DirectionAlign, _mot_y_direction, 1.0f),
	// Motor X direction
	AP_GROUPINFO("MOT_ZDIR",    6, DirectionAlign, _mot_z_direction, 1.0f),
	
  AP_GROUPEND
};

DirectionAlign::DirectionAlign(void)
{

}

// 求角度差
float small_angle_sub(float a, float b)
{
	float c;
	c = a - b;
	if(c > PI) c -= 2*PI;
	else if(c < -PI) c += 2*PI;
	return c;
}

// 计算编码器和电机方向
void DirectionAlign::aligning_update(Vector3f *euler, Vector3f *enc, Vector3f *mot)
{
	static Vector3f euler0, euler1, enc0, enc1;
	static Vector3f mot_value(0.0, 0.0, 0.0);
	static int cnt;
	int inv_time = 1;
	
	static int state = 0;
	float mot_step = 0.03;
	float mot_max = 100;
	float euler_max = 10.0f / RAD2DEG;
	
	if(enc->x == 0 && enc->y == 0 && enc->z == 0) return;
	
	switch(state)
	{
		// Align Y axis
		case 0:
			if(mot_value.y == 0.0) 
			{
				enc0.y = enc->y;
				euler0.y = euler->y;
				_enc_x_direction = 1.0f;
				_enc_y_direction = 1.0f;
				_enc_z_direction = 1.0f;
				_mot_x_direction = 1.0f;
				_mot_y_direction = 1.0f;
				_mot_z_direction = 1.0f;
			}
			mot_value.y += mot_step;
			if(mot_value.y > mot_max) 
				mot_value.y = mot_max;
			if(abs(small_angle_sub(euler->y, euler0.y)) > euler_max)
			{
				mot_value.y = -mot_value.y;
				enc1.y = enc->y;
				euler1.y = euler->y;
				state = 1;
			}
			break;
		case 1:
			cnt++;
		  if(cnt >= inv_time)
			{
				cnt = 0;
			  mot_value.y = 0;
		    state = 2;
			}
		  break;
		// Align X axis
		case 2:
			if(mot_value.x == 0.0)
			{
				enc0.x = enc->x;
				euler0.x = euler->x;
			}
			mot_value.x += mot_step;
			if(mot_value.x > mot_max)
				mot_value.x = mot_max;
			if(abs(small_angle_sub(euler->x, euler0.x)) > euler_max)
			{
				mot_value.x = -mot_value.x;
				enc1.x = enc->x;
				euler1.x = euler->x;
				state = 3;
			}
			break;
		case 3:
			cnt++;
		  if(cnt >= inv_time)
			{
				cnt = 0;
			  mot_value.x = 0;
		    state = 4;
			}
		  break;
		// Align Z axis
		case 4:
			if(mot_value.z == 0.0)
			{
				enc0.z = enc->z;
				euler0.z = euler->z;
			}
			mot_value.z += mot_step;
			if(mot_value.z > mot_max)
				mot_value.z = mot_max;
			if(abs(small_angle_sub(euler->z, euler0.z)) > euler_max)
			{
				mot_value.z = -mot_value.z;
				enc1.z = enc->z;
				euler1.z = euler->z;
				state = 5;
			}
			break;
		case 5:
			cnt++;
		  if(cnt >= inv_time)
			{
				cnt = 0;
			  mot_value.z = 0;
		    state = 6;
			}
		  break;
		// Comput align value
		case 6:
			// Motor direction align
			if(small_angle_sub(euler1.x, euler0.x) > 0) 
				_mot_x_direction = 1.0f;
		  else
				_mot_x_direction = -1.0f;
		  if(small_angle_sub(euler1.y, euler0.y) > 0) 
				_mot_y_direction = 1.0f;
		  else 
				_mot_y_direction = -1.0f;
		  if(small_angle_sub(euler1.z, euler0.z) > 0) 
				_mot_z_direction = 1.0f;
		  else 
				_mot_z_direction = -1.0f;
		  // Encoder direction align
		  if(small_angle_sub(enc1.x, enc0.x) > 0) 
				_enc_x_direction = _mot_x_direction;
			else 
				_enc_x_direction = -_mot_x_direction;
			if(small_angle_sub(enc1.y, enc0.y) > 0) 
				_enc_y_direction = _mot_y_direction;
			else 
				_enc_y_direction = -_mot_y_direction;
			if(small_angle_sub(enc1.z, enc0.z) > 0) 
				_enc_z_direction = _mot_z_direction;
			else 
				_enc_z_direction = -_mot_z_direction;
		
		  _not_aligned = 0.0f;
		  state = 7;
			break;
		case 7:
			mot_value(0.0f, 0.0f, 0.0f);
		default:
			mot_value(0.0f, 0.0f, 0.0f);
			break;
	}
	
	mot->x = mot_value.x;
	mot->y = mot_value.y;
	mot->z = mot_value.z;
};