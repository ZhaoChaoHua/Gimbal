#include "Gimbal.h"
#include "thread_config.h"

//#define FIRST_TIME_INIT

int scop_tms;
int scop_roll;
int scop_pitch;
int scop_yaw;
int scop_rollspeed;
int scop_pitchspeed;
int scop_yawspeed;
int scop_ax_id_rx;
int scop_param_id_rx;
int scop_param_rx;
int scop_eff_set_ready;

int scop_r_mot;
int scop_p_mot;
int scop_y_mot;

Gimbal::Gimbal():
    mpu("mpu65"),
		bmm("bmm150"),
		mpu1("mpu65_1"),
		mahony(0.8, 0.05),
		madgwick(0.007),//0.002
		ekf(),
		small_ekf(),
		
//		fc_speed_x(0, 8, 8, 1.8),
//		fc_angle_x(0, 500, 50, 0.05),
//		fc_speed_y(0, 8, 8, 0.7),
//		fc_angle_y(0, 600, 100, 0.05),
//		fc_speed_z(0, 6.5, 6, 2.5),
//		fc_angle_z(0, 800, 100, 0.05),
		
		// Test Camera 1
//		fc_speed_x(0, 2, 0, 2.5),
//		fc_angle_x(0, 150, 0, 0.07),
//		fc_speed_y(0, 2, 0, 2),
//		fc_angle_y(0, 150, 0, 0.07),
//		fc_speed_z(0, 2.5, 0, 8),
//		fc_angle_z(0, 150, 0, 0.07),
		
//		// Canon
//		fc_speed_x(0, 4, 0, 4),
//		fc_angle_x(0, 150, 0, 0.07),
//		fc_speed_y(0, 4, 0, 3),
//		fc_angle_y(0, 150, 0, 0.07),
//		fc_speed_z(0, 2.8, 0, 7),
//		fc_angle_z(0, 150, 0, 0.07),
		
		//D1
		fc_speed_x(0, 2, 9, 0.9),
		fc_angle_x(0, 70, 0, 0.12),
		fc_speed_y(0, 1.6, 6, 0.8),
		fc_angle_y(0, 70, 0, 0.12),
		fc_speed_z(0, 2.4, 6, 1.5),
		fc_angle_z(0, 70, 0, 0.12),
		
    attitude_control(attitude, 
                     param.pd_stabilize_roll, param.pd_stabilize_pitch, param.pd_stabilize_yaw,
                     param.pid_rate_roll, param.pid_rate_pitch, param.pid_rate_yaw),
    mavlink("uart2", gringbuf),
    param(),
    param_loader(var_info),
		acc_caltor(),
		acc_cal(&acc_caltor),
    calibration(attitude),
		attitude(madgwick,mahony,ekf, mpu,bmm,mpu1, acc_cal, acc_caltor),
		direction_align()
{

    exit_attitude_thread = false;
    exit_control_thread = false;
	  exit_sekf_thread = false;
    exit_can_thread = false;
    exit_packing_thread = false;
	  exit_log_thread = false;
	  exit_acc_cal_thread = false;
    exit_fuzzy_logic_thread = false;
	  exit_direction_thread = false;
	  
	  exit_serial_thread = false;
    serial_thread_init = false;
	  //led = rt_device_find("led_dev");
	  //rt_device_open(led, RT_DEVICE_OFLAG_WRONLY);
	  //rt_device_write(led, 0, 
	  
}

void Gimbal::setup(const Scheduler::Task *tasks, uint8_t num_tasks)
{
    scheduler.init(tasks, num_tasks);
    attitude.start_init();
    can.start_init();
	  mavlink_decode_info.data_type = 0;
	  mavlink_decode_info.msg_total = 1;
	  mavlink_decode_info.msg_cnt = 0;
	  
	  exciter.init(SINE, 0.001); //sin funtion, ts = 0.001s
    
    if(!AP_Param::check_var_info())
        rt_kprintf("[err]Bad var table!\n");
    else {
			
#ifdef FIRST_TIME_INIT
			  param_loader.erase_all();
        param_loader.save_default_all();
#endif
        // Load all parameters
			  //param_loader.save_all();
        param_loader.load_all();
    }
		
		//imu read mode
	  data_box.read_mode = 0;
		
		// Tracking
		tracking_init();
		
		// Command init
		gimbal_user_command = NO_USER_COMMAND;
		
}

int scop_serial_update_flag;
// serial update
void Gimbal::serial_update(void *parameter)
{
	if(!serial_thread_init)
	{
		mavlink.start_init();
		serial_thread_init = true;
	}
	mavlink.receive_mavlink();
	static int cnt;
	cnt++;
	scop_serial_update_flag = cnt%2;
}

// packing update
void Gimbal::packing_update(void *parameter)
{
	  mavlink_decode_msg(parameter);
	  respond_to_computer(parameter);
}


// attitude update
int scop_accx, scop_accy, scop_accz;
int scop_gxf, scop_gyf, scop_gzf;
int scop_delta_time;
void Gimbal::attitude_update(void *parameter)
{	
		static uint32_t tnow, tPrev;
		float dt;
		uint32_t count;
	
    attitude.update();
	
	  // Convert to gimbal fram
	  attitude.Gyro_af.x = -attitude.Gyro_af.x;
	  attitude.Gyro_af.z = -attitude.Gyro_af.z;
	
	  tnow = SysTick->VAL;
    count = (tnow > tPrev)?(SysTick->LOAD + tPrev - tnow) : (tPrev - tnow);
    dt = count / US_T;
    dt = dt / 1000000.0f;
    tPrev=tnow;
	
	  delta_time += dt;
	  delta_angle.x += attitude.Gyro_raw.x * dt;
	  delta_angle.y += attitude.Gyro_raw.y * dt;
	  delta_angle.z += attitude.Gyro_raw.z * dt;
	
	  delta_vel.x += attitude.Acc_raw.x * dt;
	  delta_vel.y += attitude.Acc_raw.y * dt;
	  delta_vel.z += attitude.Acc_raw.z * dt;
	
	
	
	  scop_gxf = attitude.Gyro_af.x * 57295.8f;
	  scop_gyf = attitude.Gyro_af.y * 57295.8f;
	  scop_gzf = attitude.Gyro_af.z * 57295.8f;
	
	  scop_accx = attitude.Acc_raw.x * 10000;
	  scop_accy = attitude.Acc_raw.y * 10000;
	  scop_accz = attitude.Acc_raw.z * 10000;
		
		
}

int scop_euler_x;
int scop_euler_y;
int scop_euler_z;
void Gimbal::sekf_update(void *parameter)
{
	Vector3f euler_deg, euler_rad;
	Vector3f enc(-axis_angle.x,axis_angle.y,-axis_angle.z);
	scop_delta_time = delta_time*1000000;
	small_ekf.RunEKF(delta_time, delta_angle, delta_vel, enc);
	small_ekf.getEuler(&euler_deg, &attitude.euler_rad);
	delta_time = 0;
	delta_angle(0.0f, 0.0f, 0.0f);
	delta_vel(0.0f, 0.0f, 0.0f);
	
	
	scop_euler_x = euler_deg.x * 1000;
  scop_euler_y = euler_deg.y * 1000;
	scop_euler_z = euler_deg.z * 1000;
}


int scop_ax_x, scop_ax_y, scop_ax_z;
int scop_afb_x1, scop_afb_y1, scop_afb_z1;
int scop_afb_x2, scop_afb_y2, scop_afb_z2;
int scop_exciting_signal_x, scop_exciting_signal_y, scop_exciting_signal_z;
int scop_exciting_gx, scop_exciting_gy, scop_exciting_gz;
int scop_exciting_motx, scop_exciting_moty, scop_exciting_motz, scop_exciting_ex, scop_exciting_ey, scop_exciting_ez;
int scope_not_align;
void Gimbal::control_update(void *parameter)
{
	static Vector3f motor_setpoint;
//	static int enc_zero_done = 0;
	Vector3f speed_input , angle_e, angle_ce;
	Vector3f speed_e, speed_ce;
	static Vector3f last_angle_feedback, last_speed_feedback;
	
	// Head
	static float euler_z_zero;
	static bool init_head = false;
	
	// Excitation var
	float excitation_signal;
  
	if(attitude.calibration_Done())
	{
		// Get angles of roll, pitch and yaw motor (has been calibrated)
		can.get_enc(axis_angle, enc_speed);
					  // JSCOPE PARAMETER	
		// Direction align
		direction_align.align_enc_direction(&axis_angle);
		
		if(!init_head && axis_angle.z != 0) 
		{
			Vector3f enc;
			enc = can.get_enc_raw();
			// Direction align
		  direction_align.align_enc_direction(&enc);

			
			euler_z_zero = attitude.euler_rad.z - enc.z;
			init_head = true;
		}
		
		scop_ax_x = axis_angle.x*5729.0f;
		scop_ax_y = axis_angle.y*5729.0f;
		scop_ax_z = axis_angle.z*5729.0f;
		
		// Angle setpoint
		angle_setpoint = interface.update_angle_setpoint();

		// Angle feedback

		angle_feedback = attitude.euler_rad;
		
#ifndef FIRST_TIME_INIT
		angle_feedback.z = angle_feedback.z - euler_z_zero;
#endif
		
//		circadjust(angle_feedback.z, PI);
		
		if(interface.gimbal_mode == TRACKING_MODE)
		{
//			angle_e = get_angle_err_body_tracking_t2(angle_setpoint, angle_feedback, axis_angle, interface.dead_zone);
			angle_e = get_tracking_angle_e(angle_setpoint, angle_feedback, axis_angle, interface.dead_zone);
		}
		else if(interface.gimbal_mode == LOCKHEAD_MODE)
		{
//			angle_e = get_angle_err_bodyframe(angle_setpoint, angle_feedback, axis_angle);
			angle_e = get_angle_err_body_quat(attitude.q0(), attitude.q1(), attitude.q2(), attitude.q3(), euler_z_zero, angle_setpoint, axis_angle);
		}
//		angle_e(0,0,0);
		
		
#ifdef FIRST_TIME_INIT
		// FOR Calibrating encoder
		angle_e = angle_setpoint - angle_feedback;
#endif

		
		// Angle fuzzy control
		speed_input.x = fc_angle_x.run(angle_e.x);
		speed_input.y = fc_angle_y.run(angle_e.y);
		speed_input.z = fc_angle_z.run(angle_e.z);

		
#ifndef FIRST_TIME_INIT
		speed_input = vector_decouple(speed_input, axis_angle);
#endif
		
		// Excition signal for identification Angle_speed
		if(exciter.is_exciting())
		{
			// axis to exciting
			excitation_signal = exciter.update(20.0f * PI / 180.0f);
			speed_input.z = excitation_signal;
		}
		scop_exciting_signal_x = 57295.8f * speed_input.x;
		scop_exciting_signal_y = 57295.8f * speed_input.y;
		scop_exciting_signal_z = 57295.8f * speed_input.z;

		scop_exciting_gx = 57295.8f * attitude.Gyro_af.x;
		scop_exciting_gy = 57295.8f * attitude.Gyro_af.y;
		scop_exciting_gz = 57295.8f * attitude.Gyro_af.z;
		
    
		// Speed feedback
		speed_feedback = attitude.Gyro_af;
		
#ifndef FIRST_TIME_INIT
		speed_feedback = speed_decouple(axis_angle, speed_feedback, enc_speed);
#endif
		
		// Speed fuzzy control
		speed_e = speed_input - speed_feedback;
		
		// Excitation
		scop_exciting_ex = speed_e.x * 57295.8f;
		scop_exciting_ey = speed_e.y * 57295.8f;
		scop_exciting_ez = speed_e.z * 57295.8f;
		
		
		motors_output.x = fc_speed_x.run(speed_e.x);
		motors_output.y = fc_speed_y.run(speed_e.y);
		motors_output.z = fc_speed_z.run(speed_e.z);
		
		
		// Excitation
		scop_exciting_motx = motors_output.x * 10000;
		scop_exciting_moty = motors_output.y * 10000;
		scop_exciting_motz = motors_output.z * 10000;
		
		// Direction align
		
		
		// If direction not align
		if(direction_align.not_align()) return;
		
		// If disable motors
		if(interface.disable_motors)
		{
			can.send_control_rpy(0, 0, 0);
			return;
		}
		
		// SEND MOTOR OUTPUT
		if(!acc_cal.cal_done)
		{
			can.send_control_rpy(0.0f, 0.0f, 0.0f);
		}
		else
		{
			direction_align.align_mot_direction(&motors_output);
			can.send_control_rpy(motors_output.x, motors_output.y, motors_output.z);
		}
		
		

			

	}  
}


int scope_align, scope_almx, scope_almy, scope_almz;
int scope_dmx, scope_dmy, scope_dmz;
int scope_dex, scope_dey, scope_dez; 
// Direction update
void Gimbal::direction_update(void *parameter)
{
	Vector3f mot, dm, de;
	static int cnt;
	
//	if(interface.disable_motors) return;

	if(direction_align.not_align())
	{
		cnt++;
		if(cnt > 20)
		{
			cnt = 21;
			direction_align.aligning_update(&attitude.euler_rad, &axis_angle, &mot);
			scope_almx = mot.x * 10000;
			scope_almy = mot.y * 10000;
			scope_almz = mot.z * 10000;
			
			can.send_control_rpy(mot.x, mot.y, mot.z);
			
			if(!direction_align.not_align())
				param_loader.save_all();
			
		}
	}
//	else
	{
		de = direction_align.get_enc_direction();
		dm = direction_align.get_mot_direction();
	}
	scope_dmx = dm.x;
	scope_dmy = dm.y;
	scope_dmz = dm.z;
	scope_dex = de.x;
	scope_dey = de.y;
	scope_dez = de.z;
	
}


// can update
void Gimbal::can_update(void *parameter)
{
    can.update();
}

int scop_gx, scop_gy, scop_gz;


void Gimbal::generate_attitude_quat_pck(void *parameter)
{
	attitude_quat_pck.time_boot_ms = 0;
#ifdef USING_MADGWICK
	attitude_quat_pck.q1 = madgwick.get_q0();
	attitude_quat_pck.q2 = madgwick.get_q1();
	attitude_quat_pck.q3 = madgwick.get_q2();
	attitude_quat_pck.q4 = madgwick.get_q3();
#endif
#ifdef USING_MAHONY
	attitude_quat_pck.q1 = mahony.get_q0();
	attitude_quat_pck.q2 = mahony.get_q1();
	attitude_quat_pck.q3 = mahony.get_q2();
	attitude_quat_pck.q4 = mahony.get_q3();
#endif
#ifdef USING_EKF
	attitude_quat_pck.q1 = 1;
	attitude_quat_pck.q2 = 0;
	attitude_quat_pck.q3 = 0;
	attitude_quat_pck.q4 = 0;
#endif
	
	// Motors Output
	attitude_quat_pck.rollspeed = motors_output.x;
	attitude_quat_pck.pitchspeed = motors_output.y;
	attitude_quat_pck.yawspeed = motors_output.z;
	// Angle
//	attitude_quat_pck.rollspeed = attitude.euler_rad.x * 57.29577951308232f;
//	attitude_quat_pck.pitchspeed = attitude.euler_rad.y * 57.29577951308232f;
//	attitude_quat_pck.yawspeed = attitude.euler_rad.z * 57.29577951308232f;
//	// Gyro
//	attitude_quat_pck.rollspeed = attitude.Gyro_af.x*57.29577951308232f;
//	attitude_quat_pck.pitchspeed = attitude.Gyro_af.y*57.29577951308232f;
//	attitude_quat_pck.yawspeed = attitude.Gyro_af.z*57.29577951308232f;
	// Acc earth
//	attitude_quat_pck.rollspeed = attitude.ax_earth();
//	attitude_quat_pck.pitchspeed = attitude.ay_earth();
//	attitude_quat_pck.yawspeed = attitude.acc_norm();
	
		scop_gx = (int)(attitude.Gyro_af.x*1000);
	  scop_gy = (int)(attitude.Gyro_af.y*1000);
	  scop_gz = (int)(attitude.Gyro_af.z*1000);
}

void Gimbal::send_attitude_quat_pck(void *parameter)
{
	mavlink.attitude_quaternion_pack(MAV_TYPE_GIMBAL, MAV_COMP_ID_IMU, &mavlink_msg_tx,
	                                 attitude_quat_pck.time_boot_ms,
	                                 attitude_quat_pck.q1, attitude_quat_pck.q2, attitude_quat_pck.q3, attitude_quat_pck.q4, 
	                                 attitude_quat_pck.rollspeed, attitude_quat_pck.pitchspeed, attitude_quat_pck.yawspeed);
	mavlink.send_mavlink(&mavlink_msg_tx);
}           



bool generate_motor_state_ing = false;
void Gimbal::generate_motor_state_data_pck(void *parameter)
{
	int time_out;

	can.motor_state_ready_to_send = false;
	can.read_param(mavlink_msg_rx.comp_id-MAV_COMP_ID_ROLL_MOTOR+1, 22, 1.0f);
	while(!can.motor_state_ready_to_send)
	{
		rt_thread_delay(1);
		time_out++;
		if(time_out > 10)
		{
			time_out = 0;
			break;
		}
	}
}

void Gimbal::send_motor_state_data(void *parameter)
{
		rt_memcpy((void*)(&motors_state[mavlink_decode_info.component_id-MAV_COMP_ID_ROLL_MOTOR]),
		          (void*)(&(can.motors_params_for_serial[mavlink_decode_info.component_id-MAV_COMP_ID_ROLL_MOTOR].params[ENC])), 7*sizeof(float));
		if(can.roll_rps_mcu_ok) can.roll_rpl_mcu_ok = false;
	  if(can.pitch_rpl_mcu_ok) can.pitch_rpl_mcu_ok = false;
	  if(can.yaw_rpl_mcu_ok) can.yaw_rpl_mcu_ok = false;
		mavlink.motor_state_data_packing(MAV_TYPE_GIMBAL, mavlink_decode_info.component_id, &mavlink_msg_tx,
	                                 &motors_state[mavlink_decode_info.component_id-MAV_COMP_ID_ROLL_MOTOR]);
		mavlink.send_mavlink(&mavlink_msg_tx);
		generate_motor_state_ing = false;
}

void Gimbal::generate_controller_config_data_pck(void *parameter)
{
	controller_config_data_pck.roll_rate_p = fc_speed_x.get_ke();//param.pid_rate_roll.kP();
	controller_config_data_pck.roll_rate_i = fc_speed_x.get_kce();//param.pid_rate_roll.kI();
	controller_config_data_pck.roll_rate_d = fc_speed_x.get_kout();//param.pid_rate_roll.kD();
	controller_config_data_pck.roll_rate_max_i = param.pid_rate_roll.imax();
	controller_config_data_pck.roll_f_hz = (float)(param.pid_rate_roll.filt_hz());
	controller_config_data_pck.roll_stab_p = fc_angle_x.get_ke();//param.pd_stabilize_roll.kP();
	controller_config_data_pck.roll_stab_d = fc_angle_x.get_kce();//param.pd_stabilize_roll.kD();
	controller_config_data_pck.roll_speed_limit = fc_angle_x.get_kout();//param.pd_stabilize_roll.Limit();
	
	controller_config_data_pck.pitch_rate_p = fc_speed_y.get_ke();//param.pid_rate_pitch.kP();
	controller_config_data_pck.pitch_rate_i = fc_speed_y.get_kce();//param.pid_rate_pitch.kI();
	controller_config_data_pck.pitch_rate_d = fc_speed_y.get_kout();//param.pid_rate_pitch.kD();
	controller_config_data_pck.pitch_rate_max_i = param.pid_rate_pitch.imax();
	controller_config_data_pck.pitch_f_hz = (float)(param.pid_rate_pitch.filt_hz());
	controller_config_data_pck.pitch_stab_p = fc_angle_y.get_ke();//param.pd_stabilize_pitch.kP();
	controller_config_data_pck.pitch_stab_d = fc_angle_y.get_kce();//param.pd_stabilize_pitch.kD();
	controller_config_data_pck.pitch_speed_limit = fc_angle_y.get_kout();//param.pd_stabilize_pitch.Limit();
	
	controller_config_data_pck.yaw_rate_p = fc_speed_z.get_ke();//param.pid_rate_yaw.kP();
	controller_config_data_pck.yaw_rate_i = fc_speed_z.get_kce();//param.pid_rate_yaw.kI();
	controller_config_data_pck.yaw_rate_d = fc_speed_z.get_kout();//param.pid_rate_yaw.kD();
	controller_config_data_pck.yaw_rate_max_i = param.pid_rate_yaw.imax();
	controller_config_data_pck.yaw_f_hz = (float)(param.pid_rate_yaw.filt_hz());
	controller_config_data_pck.yaw_stab_p = fc_angle_z.get_ke();//param.pd_stabilize_yaw.kP();
	controller_config_data_pck.yaw_stab_d = fc_angle_z.get_kce();//param.pd_stabilize_yaw.kD();	
  controller_config_data_pck.yaw_speed_limit = fc_angle_z.get_kout();//param.pd_stabilize_yaw.Limit();	
}

void Gimbal::send_controller_config_data(void *parameter)
{
	mavlink.controller_config_data_packing(MAV_TYPE_GIMBAL, MAV_COMP_ID_CONTROLLER, &mavlink_msg_tx,
	                                       &controller_config_data_pck);	
	mavlink.send_mavlink(&mavlink_msg_tx);
}

bool generate_motors_config_ing = false;
bool motors_config_ready_to_send = false;
int motor_id_of_generating = 0;
void Gimbal::generate_motors_config_data_pck(void *parameter)
{
	int time_out;

	can.motor_config_ready_to_send = false;
	can.read_param(mavlink_msg_rx.comp_id-MAV_COMP_ID_ROLL_MOTOR+1, 30, 0.0f);
	while(!can.motor_config_ready_to_send)
	{
		rt_thread_delay(1);
		time_out++;
		if(time_out > 10)
		{
			time_out = 0;
			break;
		}
	}
}
// Send motors config data pck
void Gimbal::send_motors_config_data(void *parameter)
{
	rt_memcpy((void*)(&(motors_config_data_pck)), (void*)(&(can.motors_params_for_serial[mavlink_msg_rx.comp_id-MAV_COMP_ID_ROLL_MOTOR].params)), 15*sizeof(float));
  mavlink.motors_config_data_packing(MAV_TYPE_GIMBAL, MAV_COMP_ID_ROLL_MOTOR, &mavlink_msg_tx,
																        &motors_config_data_pck);	
	mavlink.send_mavlink(&mavlink_msg_tx);
}


void Gimbal::respond_to_computer(void *parameter)
{
	if(mavlink_decode_info.data_type == NONE) 
	{
//		rt_thread_delay(1);
		return;
	}
	switch(mavlink_decode_info.data_type)
	{
		case MAVLINK_ATTITUDE_QUAT_TYPE:
			generate_attitude_quat_pck(parameter);
		  send_attitude_quat_pck(parameter);
		  break;
		case MAVLINK_MOTOR_STATE_TYPE:
			mavlink_decode_info.data_type = NONE;
			generate_motor_state_data_pck(parameter);
			if(can.motor_state_ready_to_send)
			{
						can.motor_state_ready_to_send = false;
						send_motor_state_data(parameter);
						mavlink_decode_info.data_type = NONE;
			}
		  break;
		case MAVLINK_CONTROLLER_CONFIG_TYPE:
			if(mavlink_decode_info.write == 1)
			{
				param_loader.save_all();
				mavlink_decode_info.data_type = NONE;
			}
			else
			{
				if(mavlink_decode_info.msg_cnt < mavlink_decode_info.msg_total)
				{
					param_loader.load_all();
					mavlink_decode_info.msg_cnt++;
					generate_controller_config_data_pck(parameter);
					send_controller_config_data(parameter);
					mavlink_decode_info.data_type = NONE;
				}
			}
			break;
		case MAVLINK_MOTOR_CONFIG_TYPE:
			if(mavlink_decode_info.write == 1)
			{
				
			}
			else
			{
				if(mavlink_decode_info.msg_cnt < mavlink_decode_info.msg_total)
				{
					mavlink_decode_info.msg_cnt++;
					generate_motors_config_data_pck(parameter);
					if(can.motor_config_ready_to_send)
					{
						can.motor_config_ready_to_send = false;
						send_motors_config_data(parameter);
						mavlink_decode_info.data_type = NONE;
					}
				}
			}
			break;
		case MAVLINK_WRITE_PARAM_TYPE:
			mavlink_write_param(parameter);
			break;
		case MAVLINK_READ_PARAM_TYPE:
			mavlink_read_param(parameter);
			break;
		// Gimbal control 
		case MAVLINK_GIMBAL_CONTROL_TYPE:
			mavlink_decode_info.data_type = MAVLINK_ATTITUDE_QUAT_TYPE;
			break;
		// Gimbal command
		case MAVLINK_GIMBAL_COMMAND_TYPE:
			if(gimbal_user_command != NO_USER_COMMAND)
			{
				switch(gimbal_user_command)
				{
					// Tracking mode
					case 0:
						break;
					// Free mode
					case 1:
						break;
					// Back to zero position
					case 2:
						break;
					// Encoder zero position calibration
					case 3:
						enc_zero_calibration();
						break;
					// IMU Calibration
					case 4:
						break;
					// Exciting signal generating
					case 5:
						exciter.start_exciting();
						break;
					case 6:
						break;
					case 7:
						break;
					// Drone state is lock, Set tripod motor tracking to drone
					case 8:
						if(interface.last_drone_state != interface.drone_state && interface.drone_state == 0)
						{
							interface.disable_motors = true;
							can.send_control_rpy(0.0f, 0.0f, 0.0f);
							rt_thread_delay(1);
							can.write_param(TRIPOD_MOTOR_ADDR, TRACKING_OBJECT, 2);
							interface.disable_motors = false;
						}
						break;
					// Drone state is unlock, Set tripod motor tracking to gimbal
					case 9:
						if(interface.last_drone_state != interface.drone_state && interface.drone_state == 1)
						{
							interface.disable_motors = true;
							can.send_control_rpy(0.0f, 0.0f, 0.0f);
							rt_thread_delay(1);
							can.write_param(TRIPOD_MOTOR_ADDR, TRACKING_OBJECT, 1);
							interface.disable_motors = false;
						}
						break;
					// Disable tripod motor
					case 10:
						if(interface.last_drone_state != interface.drone_state && interface.drone_state == 2)
						{
							interface.disable_motors = true;
							can.send_control_rpy(0.0f, 0.0f, 0.0f);
							rt_thread_delay(1);
							can.write_param(TRIPOD_MOTOR_ADDR,TRACKING_OBJECT,2);
							interface.disable_motors = false;
						}
					  break;
					// Enable tripod motor
					case 11:
						can.write_param(TRIPOD_MOTOR_ADDR,DISABLE_MOTOR,0);
					  break;
					
					default:
						break;
				}
				mavlink_decode_info.data_type = MAVLINK_ATTITUDE_QUAT_TYPE;
				gimbal_user_command = NO_USER_COMMAND;
			}
			break;
		case NONE:
			break;
		default:
			break;
	}
}


void Gimbal::set_controller_param_from_mavlink(void *parameter)
{
		switch(mavlink_param_rx.param_id)
		{
			case 0:
				// Set x fuzzy controller ke
				fc_speed_x.set_ke(mavlink_param_rx.param_value);
//				param.pid_rate_roll.kP(mavlink_param_rx.param_value);
			  controller_config_data_pck.roll_rate_p = mavlink_param_rx.param_value;
				break;
			case 1:
				// Set x fuzzy controller kce
				fc_speed_x.set_kce(mavlink_param_rx.param_value);
//				param.pid_rate_roll.kI(mavlink_param_rx.param_value);
			  controller_config_data_pck.roll_rate_i = mavlink_param_rx.param_value;
				break;
			case 2:
				// Set x fuzzy controller kout
				fc_speed_x.set_kout(mavlink_param_rx.param_value);
//				param.pid_rate_roll.kD(mavlink_param_rx.param_value);
			  controller_config_data_pck.roll_rate_d = mavlink_param_rx.param_value;
				break;
			case 3:
				param.pid_rate_roll.imax(mavlink_param_rx.param_value);
			  controller_config_data_pck.roll_rate_max_i = mavlink_param_rx.param_value;
				break;
			case 4:
				param.pid_rate_roll.filt_hz(mavlink_param_rx.param_value);
			  controller_config_data_pck.roll_f_hz = mavlink_param_rx.param_value;
				break;
			case 5:
				// Set x fuzzy controller ke
				fc_angle_x.set_ke(mavlink_param_rx.param_value);
//				param.pd_stabilize_roll.kP(mavlink_param_rx.param_value);
			  controller_config_data_pck.roll_stab_p = mavlink_param_rx.param_value;
				break;
			case 6:
				// Set x fuzzy controller kce
				fc_angle_x.set_kce(mavlink_param_rx.param_value);
//				param.pd_stabilize_roll.kD(mavlink_param_rx.param_value);
			  controller_config_data_pck.roll_stab_d = mavlink_param_rx.param_value;
				break;
			case 7:
				// Set x fuzzy controller kout
				fc_angle_x.set_kout(mavlink_param_rx.param_value);
//				param.pd_stabilize_roll.Limit(mavlink_param_rx.param_value);
		    controller_config_data_pck.roll_speed_limit = mavlink_param_rx.param_value;
			  break;
			
			case 8:
				// Set y fuzzy controller ke
				fc_speed_y.set_ke(mavlink_param_rx.param_value);
//				param.pid_rate_pitch.kP(mavlink_param_rx.param_value);
			  controller_config_data_pck.pitch_rate_p = mavlink_param_rx.param_value;
				break;
			case 9:
				// Set y fuzzy controller kce
				fc_speed_y.set_kce(mavlink_param_rx.param_value);
//				param.pid_rate_pitch.kI(mavlink_param_rx.param_value);
			  controller_config_data_pck.pitch_rate_i = mavlink_param_rx.param_value;
				break;
			case 10:
				// Set y fuzzy controller kout
				fc_speed_y.set_kout(mavlink_param_rx.param_value);
//				param.pid_rate_pitch.kD(mavlink_param_rx.param_value);
			  controller_config_data_pck.pitch_rate_d = mavlink_param_rx.param_value;
				break;
			case 11:
				param.pid_rate_pitch.imax(mavlink_param_rx.param_value);
			  controller_config_data_pck.pitch_rate_max_i = mavlink_param_rx.param_value;
				break;
			case 12:
				param.pid_rate_pitch.filt_hz(mavlink_param_rx.param_value);
			  controller_config_data_pck.pitch_f_hz = mavlink_param_rx.param_value;
				break;
			case 13:
				// Set y fuzzy controller ke
				fc_angle_y.set_ke(mavlink_param_rx.param_value);
//				param.pd_stabilize_pitch.kP(mavlink_param_rx.param_value);
			  controller_config_data_pck.pitch_stab_p = mavlink_param_rx.param_value;
				break;
			case 14:
				// Set y fuzzy controller ke
				fc_angle_y.set_kce(mavlink_param_rx.param_value);
//				param.pd_stabilize_pitch.kD(mavlink_param_rx.param_value);
			  controller_config_data_pck.pitch_stab_d = mavlink_param_rx.param_value;
				break;
			case 15:
				// Set y fuzzy controller kout
				fc_angle_y.set_kout(mavlink_param_rx.param_value);
//				param.pd_stabilize_pitch.Limit(mavlink_param_rx.param_value);
			  controller_config_data_pck.pitch_speed_limit = mavlink_param_rx.param_value;
			  break;

			case 16:
				// Set z fuzzy controller ke
				fc_speed_z.set_ke(mavlink_param_rx.param_value);
//				param.pid_rate_yaw.kP(mavlink_param_rx.param_value);
			  controller_config_data_pck.yaw_rate_p = mavlink_param_rx.param_value;
				break;
			case 17:
				// Set z fuzzy controller kce
				fc_speed_z.set_kce(mavlink_param_rx.param_value);
//				param.pid_rate_yaw.kI(mavlink_param_rx.param_value);
			  controller_config_data_pck.yaw_rate_i = mavlink_param_rx.param_value;
				break;
			case 18:
				// Set z fuzzy controller kout
				fc_speed_z.set_kout(mavlink_param_rx.param_value);
//				param.pid_rate_yaw.kD(mavlink_param_rx.param_value);
			  controller_config_data_pck.yaw_rate_d = mavlink_param_rx.param_value;
				break;
			case 19:
				param.pid_rate_yaw.imax(mavlink_param_rx.param_value);
			  controller_config_data_pck.yaw_rate_max_i = mavlink_param_rx.param_value;
				break;
			case 20:
				param.pid_rate_yaw.filt_hz(mavlink_param_rx.param_value);
			  controller_config_data_pck.yaw_f_hz = mavlink_param_rx.param_value;
				break;
			case 21:
				// Set z fuzzy controller ke
				fc_angle_z.set_ke(mavlink_param_rx.param_value);
//				param.pd_stabilize_yaw.kP(mavlink_param_rx.param_value);
			  controller_config_data_pck.yaw_stab_p = mavlink_param_rx.param_value;
				break;
			case 22:
				// Set z fuzzy controller kce
				fc_angle_y.set_kce(mavlink_param_rx.param_value);
//				param.pd_stabilize_yaw.kD(mavlink_param_rx.param_value);
			  controller_config_data_pck.yaw_stab_d = mavlink_param_rx.param_value;
				break;
			case 23:
				// Set z fuzzy controller kout
				fc_angle_z.set_kout(mavlink_param_rx.param_value);
//				param.pd_stabilize_yaw.Limit(mavlink_param_rx.param_value);
			  controller_config_data_pck.yaw_speed_limit = mavlink_param_rx.param_value;
			  break;
			default:
				break;
			}							
}

void Gimbal::mavlink_write_param(void *parameter)
{
	rt_memcpy(&mavlink_param_rx, &mavlink_msg_rx.data, 2*sizeof(float));
	switch(mavlink_decode_info.component_id)
	{
		case MAV_COMP_ID_CONTROLLER:
			set_controller_param_from_mavlink(parameter);
		  break;
		case MAV_COMP_ID_ROLL_MOTOR:
			can.write_param(ROLL_MOTOR_ADDR, mavlink_param_rx.param_id, mavlink_param_rx.param_value);
		  break;
		case MAV_COMP_ID_PITCH_MOTOR:
			can.write_param(PITCH_MOTOR_ADDR, mavlink_param_rx.param_id, mavlink_param_rx.param_value);
		  break;
		case MAV_COMP_ID_YAW_MOTOR:
			can.write_param(YAW_MOTOR_ADDR, mavlink_param_rx.param_id, mavlink_param_rx.param_value);
		  break;
		default:
			break;
	}
	mavlink_decode_info.data_type = NONE;
}


void Gimbal::mavlink_read_param(void *parameter)
{
	rt_memcpy(&mavlink_param_rx, &mavlink_msg_rx.data, 2*sizeof(float));
	switch(mavlink_decode_info.component_id)
	{
		case MAV_COMP_ID_CONTROLLER:
      
		  break;
		case MAV_COMP_ID_ROLL_MOTOR:
			can.read_param(ROLL_MOTOR_ADDR, mavlink_param_rx.param_id, 0.0f);
		  break;
		case MAV_COMP_ID_PITCH_MOTOR:
			can.read_param(PITCH_MOTOR_ADDR, mavlink_param_rx.param_id, 0.0f);
		  break;
		case MAV_COMP_ID_YAW_MOTOR:
			can.read_param(YAW_MOTOR_ADDR, mavlink_param_rx.param_id, 0.0f);
		  break;
		default:
			break;
	}
	mavlink_decode_info.data_type = NONE;
}

int scop_magx, scop_magy, scop_magz;
int scop_emagx, scop_emagy, scop_emagz;
int scop_bmagx, scop_bmagy, scop_bmagz;
int scop_velx, scop_vely, scop_velz;
void Gimbal::mavlink_decode_msg(void *parameter)
{
	Vector3f vec;
	uint8_t param_id;
	int command;
	scop_eff_set_ready = (int)mavlink.ready_to_decode;
	if(!this->mavlink.ringbuf.isEmpty())
	{
		this->mavlink.ringbuf.read(&this->mavlink_msg_rx, 1);
//		mavlink.ready_to_decode = false;
		mavlink_decode_info.component_id = mavlink_msg_rx.comp_id;
		mavlink_decode_info.last_data_type = mavlink_decode_info.data_type;
		switch(mavlink_msg_rx.msg_id)
		{
			case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
				mavlink_decode_info.data_type = MAVLINK_ATTITUDE_QUAT_TYPE;
			  break;
			case MAVLINK_MSG_ID_MOTOR_STATE_DATA:
			  mavlink_decode_info.data_type = MAVLINK_MOTOR_STATE_TYPE;
			  break;
			case MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA:
			  mavlink_decode_info.data_type = MAVLINK_CONTROLLER_CONFIG_TYPE;
			  mavlink_decode_info.write = mavlink_msg_rx.data[0];
			  mavlink_decode_info.msg_cnt = 0;
			  break;
			case MAVLINK_MSG_ID_MOTORS_CONFIG_DATA:
			  mavlink_decode_info.data_type = MAVLINK_MOTOR_CONFIG_TYPE;
			  mavlink_decode_info.write = mavlink_msg_rx.data[0];
			  mavlink_decode_info.msg_cnt = 0;
			  break;
	    case MAVLINK_MSG_ID_WRITE_PARAM_DATA:
			  mavlink_decode_info.data_type = MAVLINK_WRITE_PARAM_TYPE;
			  break;
			case MAVLINK_MSG_ID_READ_PARAM_DATA:
				mavlink_decode_info.data_type = MAVLINK_READ_PARAM_TYPE;
				mavlink_decode_info.multi_param = mavlink_msg_rx.data[4];
			  break;
			case MAVLINK_MSG_ID_GIMBAL_CONTROL:
				param_id = mavlink.gimbal_control_data_decode(&mavlink_msg_rx, &vec);
			  interface.set_control_value(param_id, vec);
				mavlink_decode_info.data_type = MAVLINK_GIMBAL_CONTROL_TYPE;
			  break;
			case MAVLINK_MSG_ID_GIMBAL_MAIN_COMMAND:
				command = mavlink.gimbal_command_data_decode(&mavlink_msg_rx);
			  gimbal_user_command = interface.set_command_value(command);
				mavlink_decode_info.data_type = MAVLINK_GIMBAL_COMMAND_TYPE;
			  break;
			case MAVLINK_MSG_ID_GIMBAL_DRONE_DATA:
				mavlink.drone_data_decode(&mavlink_msg_rx, &Mag_drone, &EMag, &BMag, &Vel_drone);
			  small_ekf.setMagData(Mag_drone);
			  small_ekf.setMeasVelNED(Vel_drone);
			  small_ekf.setEMag(EMag);
			  small_ekf.setBMag(BMag);
			  
			  scop_magx = Mag_drone.x*1000;
				scop_magy = Mag_drone.y*1000;
				scop_magz = Mag_drone.z*1000;
			  scop_emagx = EMag.x*1000;
				scop_emagy = EMag.y*1000;
				scop_emagz = EMag.z*1000;
			  scop_bmagx = BMag.x*1000;
				scop_bmagy = BMag.y*1000;
				scop_bmagz = BMag.z*1000;
			  scop_velx = Vel_drone.x*1000;
				scop_vely = Vel_drone.y*1000;
				scop_velz = Vel_drone.z*1000;
			  break;
			default:
				break;
		}
	}
}

rt_event_t Gimbal::Sys_event()
{
    return &Scheduler::Sys_Event;
}

void Gimbal::scheduler_run()
{
    // tell the scheduler one tick has passed
    scheduler.tick();
    scheduler.run(500);
}

// Encoder Zero Pos Calibration
void Gimbal::enc_zero_calibration(void)
{
	interface.disable_motors = 1;
	can.encoder_zero_calibration();
	param_loader.save_all();
	interface.disable_motors = 0;
}



Gimbal gimbal;

#ifdef RT_USING_FINSH
#include <finsh.h>

// ∏Ò ΩªØEEPROM
void do_accel()
{
    gimbal.calibration.do_accel_calibration();
}
FINSH_FUNCTION_EXPORT(do_accel, Accel calibration.)

void acccal()
{
	gimbal.acc_cal.start();
}
FINSH_FUNCTION_EXPORT(acccal, Accel calibration.)

#endif
