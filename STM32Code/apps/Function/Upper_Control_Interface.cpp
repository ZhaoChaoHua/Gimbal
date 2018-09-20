#include "Upper_Control_Interface.h"

#define DEG2RAD   0.017453292519943f         // rad = deg * DEG2RAD
#define RAD2DEG   57.295779513082320         // deg = rad * RAD2DEG

Upper_Control_Interface::Upper_Control_Interface()
{
	this->angle_setpoint.x = this->angle_setpoint.y = this->angle_setpoint.z = 0;
	this->intergral_axis_control.x = this->intergral_axis_control.y = this->intergral_axis_control.z = 0;
	this->limit_speed = false;
	this->limit_angle = false;
	this->angle_limit.x = 10*DEG2RAD;
	this->angle_limit.y = 100*DEG2RAD;
	this->angle_limit.z = 360*DEG2RAD;
	this->gimbal_mode = TRACKING_MODE;
	this->speed_cof.x = 0.00002;
	this->speed_cof.y = 0.0001;
	this->speed_cof.z = 0.00002;
	this->speed_limit.x = 50*DEG2RAD*this->speed_cof.x;
	this->speed_limit.y = 50*DEG2RAD*this->speed_cof.y;
	this->speed_limit.z = 50*DEG2RAD*this->speed_cof.z;
	this->disable_motors = 1;
	this->dead_zone(12.0f*DEG2RAD, 12.0f*DEG2RAD, 12.0f*DEG2RAD);
	// drone_state 0:lock, 1: unlosk
	this->drone_state = 0;
	this->last_drone_state = 0;
}

float limit_value(float value, float min, float max)
{
	value = value > max ? max : value;
	value = value < min ? min : value;
	return value;
}

int scop_ctrl_x, scop_ctrl_y, scop_ctrl_z;
int scop_ing_x, scop_ing_y, scop_ing_z;
int scop_lm_x, scop_lm_y, scop_lm_z;

Vector3f Upper_Control_Interface::update_angle_setpoint(void)
{
	Vector3f speed;
	// Cal speed
	speed.x = this->axis_control.x*this->speed_cof.x;
	speed.y = this->axis_control.y*this->speed_cof.y;
	speed.z = this->axis_control.z*this->speed_cof.z;
	
	if(speed.x == 0.0f && speed.y == 0.0f && speed.z == 0.0f) return angle_setpoint;
	
	// Limit speed or not
	if(this->limit_speed)
	{
		speed.x = limit_value(speed.x, -this->speed_limit.x, this->speed_limit.x);
		speed.y = limit_value(speed.y, -this->speed_limit.y, this->speed_limit.y);
		speed.z = limit_value(speed.z, -this->speed_limit.z, this->speed_limit.z);
	}
	scop_ctrl_x = (int)(speed.x * 1000000);
	scop_ctrl_y = (int)(speed.y * 1000000);
  scop_ctrl_z = (int)(speed.z * 1000000);	
	// Cal angle change
	this->intergral_axis_control.x += speed.x;
	this->intergral_axis_control.y += speed.y;
	this->intergral_axis_control.z += speed.z;
	// Get angle setpoint
	this->angle_setpoint.x += speed.x;
	this->angle_setpoint.y += speed.y;
	this->angle_setpoint.z += speed.z;
	
	scop_ing_x = this->angle_setpoint.x*1000;
	scop_ing_y = this->angle_setpoint.x*1000;
	scop_ing_z = this->angle_setpoint.x*1000;
	// Limit angle or not
	
	scop_lm_x = angle_limit.x*1000;
	scop_lm_y = angle_limit.y*1000;
	scop_lm_z = angle_limit.z*1000;
	if(this->limit_angle)
	{
		this->angle_setpoint.x = limit_value(this->angle_setpoint.x, -this->angle_limit.x, this->angle_limit.x);
		this->angle_setpoint.y = limit_value(this->angle_setpoint.y, -this->angle_limit.y+50.0f*DEG2RAD, this->angle_limit.y);
		this->angle_setpoint.z = limit_value(this->angle_setpoint.z, -this->angle_limit.z, this->angle_limit.z);
	}
	return angle_setpoint;
}

void Upper_Control_Interface::reset_intergral_axis_control(void)
{
	this->intergral_axis_control.x = this->intergral_axis_control.y = this->intergral_axis_control.z = 0;
}

int Upper_Control_Interface::set_control_value(uint8_t param_id, Vector3f vec)
{
	switch(param_id)
	{
		// Angle Setpoint
		case 0:
			this->angle_setpoint.x = vec.x * DEG2RAD;
		  this->angle_setpoint.y = vec.y * DEG2RAD;
		  this->angle_setpoint.z = vec.z * DEG2RAD;
		  this->reset_intergral_axis_control();
		  break;
		// Axis Control
		case 1:
			this->axis_control.x = vec.x * DEG2RAD;
		  this->axis_control.y = vec.y * DEG2RAD;
		  this->axis_control.z = vec.z * DEG2RAD;
		  break;
		// Speed Limit
		case 2:
      this->speed_limit.x = vec.x * DEG2RAD;
		  this->speed_limit.y = vec.y * DEG2RAD;
		  this->speed_limit.z = vec.z * DEG2RAD;
		  break;
		// Dead Zone
		case 3:
			this->dead_zone.x = vec.x * DEG2RAD;
		  this->dead_zone.y = vec.y * DEG2RAD;
		  this->dead_zone.z = vec.z * DEG2RAD;
		  break;
		// Angle Limit
		case 4:
			this->angle_limit.x = vec.x * DEG2RAD;
		  this->angle_limit.y = vec.y * DEG2RAD;
		  this->angle_limit.z = vec.z * DEG2RAD;
		  break;
		case 5:
			this->imu_beta = vec.y;
		  break;
		default:
			break;
	}
	return param_id;
}

int Upper_Control_Interface::set_command_value(int command)
{
	switch(command)
	{
		// Tracking mode
		case 0: 
			this->gimbal_mode = 0;
		   break;
		// Free mode
		case 1: 
			this->gimbal_mode = 1;
			break;
		// Back to zero
		case 2:
			this->angle_setpoint.x = 0;
		  this->angle_setpoint.y = 0;
		  this->angle_setpoint.z = 0;
		  this->reset_intergral_axis_control();
		  break;
		// Zero pos calibration
		case 3:
			break;
		// IMU Calibration
		case 4:
			break;
		// Controller Parameter Auto Calibration
		case 5:
			break;
		// Enable Speed Limit
		case 6:
			this->disable_motors = true;
		  break;
		case 7:
			this->disable_motors = false;
		  break;
		// Drone state is lock
		case 8:
			this->last_drone_state = drone_state;
			this->drone_state = 0; //
		  break;
		// Drone state is unlock
		case 9:
			this->last_drone_state = drone_state;
			this->drone_state = 1;
		  break;
		// Disable Angle Limit
		case 10:
			this->last_drone_state = drone_state;
			this->drone_state = 2;
		  break;
		// Acc calibration
		case 11:
			this->limit_angle = true;
			break;
		case 12:
			this->limit_angle = false;
		case 13:
			this->limit_speed = true;
			break;
		case 14:
			this->limit_speed = false;
			break;
    default:
			break;
	}
	return command;
}
