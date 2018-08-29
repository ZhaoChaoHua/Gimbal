#include "AP_Math.h"
#include <stm32f4xx.h>
#include <rtthread.h>

#define NO_USER_COMMAND 1000

#define TRACKING_MODE  0
#define LOCKHEAD_MODE  1

class Upper_Control_Interface
{
	public:
	  Upper_Control_Interface();
	
		Vector3f		angle_setpoint;
	  Vector3f		axis_control;
	  Vector3f    speed_limit;
	  Vector3f    dead_zone;
	  Vector3f    angle_limit;
	  
	  //integral = integral + speed_cof*axis_control
	  Vector3f       speed_cof;
	  Vector3f    intergral_axis_control;
	
	
	  bool        limit_speed;
	  bool        limit_angle;
	  uint8_t     gimbal_mode;
	  float       imu_beta;
	  bool        disable_motors;
	// Drone state 0: lock; 1: unlock;
	  uint8_t        drone_state;
	  uint8_t        last_drone_state;
	
	  int set_control_value(uint8_t param_id, Vector3f vec);
	  int set_command_value(int command);
	  Vector3f update_angle_setpoint(void);
		void reset_intergral_axis_control(void);
	
};
	  
