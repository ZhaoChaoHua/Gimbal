#ifndef __GIMBAL_H__
#define __GIMBAL_H__
 
#include <rtthread.h>
#include "stm32f4xx.h"
#include "parameter.h"
#include "AttitudeControl.h"
#include "Attitude.h"
#include "accelerometer_calibration.h"
#include "CAN_com.h"
#include "mavlink_com.h"
#include "time_measure.h"
#include "bgc_param.h"
#include "scheduler.h"
#include "Upper_Control_Interface.h"
#include "dataBox.h"
#include "att2mot.h"
#include "DCM_EKF_IMU.h"
#include "FuzzyLogic.h"
#include "fuzzy_control.h"
#include "excitation.h"
#include "DirectionAlign.h"
                  
#include "rtwtypes.h"

#ifdef __cplusplus




// 模糊规则组
const uint8_t p_rule[49][3] PROGMEM = 
{ // E   Ec  Kp
    {PB, PB, NB},{PB, PM, NB},{PB, PS, NM},{PB, ZO, NM},{PB, NS, NM},{PB, NM, ZO},{PB, NB, ZO},
    {PM, PB, NB},{PM, PM, NM},{PM, PS, NM},{PM, ZO, NM},{PM, NS, NS},{PM, NM, ZO},{PM, NB, PS},
    {PS, PB, NM},{PS, PM, NM},{PS, PS, NS},{PS, ZO, NS},{PS, NS, ZO},{PS, NM, PS},{PS, NB, PS},
    {ZO, PB, NM},{ZO, PM, NM},{ZO, PS, NS},{ZO, ZO, ZO},{ZO, NS, PS},{ZO, NM, PM},{ZO, NB, PM},
    {NS, PB, NS},{NS, PM, NS},{NS, PS, ZO},{NS, ZO, PS},{NS, NS, PM},{NS, NM, PM},{NS, NB, PM},
    {NM, PB, NS},{NM, PM, ZO},{NM, PS, PS},{NM, ZO, PS},{NM, NS, PM},{NM, NM, PB},{NM, NB, PB},
    {NB, PB, ZO},{NB, PM, ZO},{NB, PS, PS},{NB, ZO, PM},{NB, NS, PM},{NB, NM, PB},{NB, NB, PB}
};

const uint8_t i_rule[49][3] PROGMEM = 
{ // E   Ec  Ki
    {PB, PB, PB},{PB, PM, PB},{PB, PS, PM},{PB, ZO, PM},{PB, NS, PS},{PB, NM, ZO},{PB, NB, ZO},
    {PM, PB, PB},{PM, PM, PB},{PM, PS, PM},{PM, ZO, PS},{PM, NS, PS},{PM, NM, ZO},{PM, NB, ZO},
    {PS, PB, PB},{PS, PM, PM},{PS, PS, PS},{PS, ZO, PS},{PS, NS, ZO},{PS, NM, NS},{PS, NB, NM},
    {ZO, PB, PM},{ZO, PM, PM},{ZO, PS, PS},{ZO, ZO, ZO},{ZO, NS, NS},{ZO, NM, NM},{ZO, NB, NM},
    {NS, PB, PS},{NS, PM, PS},{NS, PS, ZO},{NS, ZO, NS},{NS, NS, NS},{NS, NM, NM},{NS, NB, NB},
    {NM, PB, ZO},{NM, PM, ZO},{NM, PS, NS},{NM, ZO, NS},{NM, NS, NM},{NM, NM, NB},{NM, NB, NB},
    {NB, PB, ZO},{NB, PM, ZO},{NB, PS, NS},{NB, ZO, NM},{NB, NS, NM},{NB, NM, NB},{NB, NB, NB}
};

const uint8_t d_rule[49][3] PROGMEM = 
{ // E   Ec  Kd
    {PB, PB, PB},{PB, PM, PS},{PB, PS, PS},{PB, ZO, PM},{PB, NS, PM},{PB, NM, PM},{PB, NB, PB},
    {PM, PB, PB},{PM, PM, PS},{PM, PS, PS},{PM, ZO, PS},{PM, NS, PS},{PM, NM, NS},{PM, NB, PB},
    {PS, PB, ZO},{PS, PM, ZO},{PS, PS, ZO},{PS, ZO, ZO},{PS, NS, ZO},{PS, NM, ZO},{PS, NB, ZO},
    {ZO, PB, ZO},{ZO, PM, NS},{ZO, PS, NS},{ZO, ZO, NS},{ZO, NS, NS},{ZO, NM, NS},{ZO, NB, ZO},
    {NS, PB, ZO},{NS, PM, NS},{NS, PS, NS},{NS, ZO, NM},{NS, NS, NM},{NS, NM, NS},{NS, NB, ZO},
    {NM, PB, ZO},{NM, PM, NS},{NM, PS, NM},{NM, ZO, NM},{NM, NS, NB},{NM, NM, NS},{NM, NB, PS},
    {NB, PB, PS},{NB, PM, NM},{NB, PS, NB},{NB, ZO, NB},{NB, NS, NB},{NB, NM, NS},{NB, NB, PS}
};

// 误差隶属函数
const mf_node e_mf[7] PROGMEM =
{
    //                名称                         隶属函数
    MF_GAUSSIAN(NB, "gaussmf", 0.65f, -3.0f,       FuzzyLogic::fisGaussianMf),  // NB
    MF_TRIANGLE(NM, "trimf",  -3.0f,  -2.0f, 0.0f, FuzzyLogic::fisTriangleMf),  // NM
    MF_TRIANGLE(NS, "trimf",  -3.0f,  -1.0f, 1.0f, FuzzyLogic::fisTriangleMf),  // NS
    MF_TRIANGLE(ZO, "trimf",  -2.0f,   0.0f, 2.0f, FuzzyLogic::fisTriangleMf),  // ZO
    MF_TRIANGLE(PS, "trimf",  -1.0f,   1.0f, 3.0f, FuzzyLogic::fisTriangleMf),  // PS
    MF_TRIANGLE(PM, "trimf",   0.0f,   2.0f, 3.0f, FuzzyLogic::fisTriangleMf),  // PM
    MF_GAUSSIAN(PB, "gaussmf", 0.65f,  3.0f,       FuzzyLogic::fisGaussianMf)   // PB
};

// 误差变化率隶属函数
const mf_node ec_mf[7] PROGMEM =
{
    //                名称                         隶属函数
    MF_GAUSSIAN(NB, "gaussmf",  0.65f, -3.0f,       FuzzyLogic::fisGaussianMf),  // NB
    MF_TRIANGLE(NM, "trimf",   -3.0f,  -2.0f, 0.0f, FuzzyLogic::fisTriangleMf),  // NM
    MF_TRIANGLE(NS, "trimf",   -3.0f,  -1.0f, 1.0f, FuzzyLogic::fisTriangleMf),  // NS
    MF_TRIANGLE(ZO, "trimf",   -2.0f,   0.0f, 2.0f, FuzzyLogic::fisTriangleMf),  // ZO
    MF_TRIANGLE(PS, "trimf",   -1.0f,   1.0f, 3.0f, FuzzyLogic::fisTriangleMf),  // PS
    MF_TRIANGLE(PM, "trimf",    0.0f,   2.0f, 3.0f, FuzzyLogic::fisTriangleMf),  // PM
    MF_GAUSSIAN(PB, "gaussmf",  0.65f,  3.0f,       FuzzyLogic::fisGaussianMf)   // PB
};

// Kp输出隶属函数
const mf_node kp_mf[7] PROGMEM =
{
    //                名称                        隶属函数
    MF_LINEAR(NB, "linear", 0.7f, -3.0f, -3.0f),  // NB
    MF_LINEAR(NM, "linear", 0.1f,  0.5f, -2.0f),  // NM
    MF_LINEAR(NS, "linear", 0.1f,  0.5f, -1.0f),  // NS
    MF_LINEAR(ZO, "linear", 0.1f,  0.7f,  0.0f),  // ZO
    MF_LINEAR(PS, "linear", 0.1f,  0.5f,  1.0f),  // PS
    MF_LINEAR(PM, "linear", 0.1f,  0.5f,  2.0f),  // PM
    MF_LINEAR(PB, "linear", 0.7f,  3.0f,  3.0f)   // PB
};


class Gimbal
{
	
	private:
		Time_point         time;
		CAN                can;
		//	SerialManager      serial;
    MPU6500            mpu;
    BMM150             bmm;
    MPU6500            mpu1;
    MahonyAHRS       mahony;
    Madgwick         madgwick;
    DCM_EKF                ekf;

		// New Fuzzy Controller
		FuzzyController     fc_speed_x;
		FuzzyController     fc_angle_x;
		FuzzyController     fc_speed_y;
		FuzzyController     fc_angle_y;
		FuzzyController     fc_speed_z;
		FuzzyController     fc_angle_z;
	
	  QRingBuffer        gringbuf;
	
		// Drone state
		Vector3f              Err_drone;
    Vector3f              Vel_drone;
		Vector3f              Err_camera;
		Vector3f              Vel_camera;
		Vector3f              Acc_camera;
		Quaternion            Drone_quat;
		
		Vector3f              Euler_rad_from_drone;

		// Fuzzy Controller
		Vector3f            angle_err;
		Vector3f            angle_derr;
		Vector3f            angle_speed_err;
		Vector3f            angle_speed_derr;
		Vector3f            fuzzy_logic_angle_p_out;
		Vector3f            fuzzy_logic_angle_speed_p_out;
		Vector3f            angle_setpoint;
		Vector3f            angle_feedback;
		Vector3f            speed_feedback; 
		Vector3f            motors_output;
		
		// Excitation signal for identification
		Exciter             exciter;
		
		// Direction Align
		DirectionAlign      direction_align;
	
		AC_AttitudeControl attitude_control;
		Mavlink            mavlink;
		BGC_PARAM          param;
		AP_Param           param_loader;
		Scheduler          scheduler;
		Vector3f           axis_angle;
		Vector3f           enc_speed;
		Upper_Control_Interface   interface;
		DataBox              data_box;

		Vector3f            enc_zeros;
	
		// Force
		Vector3f            inertia;
		Vector3f            angle_acc;
		Vector3f            torque;
	
		// GIMBAL COMMAND FROM USER
		int gimbal_user_command;
	
    Mavlink_msg_t     mavlink_msg_rx;
    Mavlink_msg_t     mavlink_msg_tx;
    Mavlink_attitude_quaternion_t    attitude_quat_pck;
    Mavlink_control_param_t  control_param_pck;
		Mavlink_controller_config_data_t controller_config_data_pck;
		Mavlink_motors_config_data_t  motors_config_data_pck;
		Mavlink_motor_state_data_t  motors_state[3];
		Mavlink_param_t     mavlink_param_rx;
		Mavlink_decode_info mavlink_decode_info;
	  

		// 参数
		static const AP_Param::Info var_info[];
	
	public:
		// Accelerometer calibration
		AccelCalibrator    acc_caltor;
    AccelCal           acc_cal;
		Accel_Cal          calibration;
		Attitude           attitude;
	
		friend class BGC_PARAM;
		rt_device_t       led;
		Gimbal(void);
		void setup(const Scheduler::Task *tasks, uint8_t num_tasks);
    // thread control parameter
    bool exit_serial_thread;
    bool exit_attitude_thread;
    bool exit_control_thread;
    bool exit_can_thread;
    bool exit_packing_thread;
    bool exit_log_thread;
    bool exit_acc_cal_thread;
    bool exit_fuzzy_logic_thread;
	  bool exit_direction_thread;

    // thread control parameter
    bool serial_thread_init;

    // serial update
    void serial_update(void *parameter);
    // attitude update
    void attitude_update(void *parameter);
    // control update
    void control_update(void *parameter);
    // can update
    void can_update(void *parameter);	
		// direction update
		void direction_update(void *parameter);

    // packing update
    void packing_update(void *parameter);

    // get attitude pck from Attitude
    void generate_attitude_pck(void *parameter);
    // send an attitude data(Mavlink);
    void send_attitude_pck(void *parameter);
    // Get attitude quaternion pck
    void generate_attitude_quat_pck(void *parameter);
    // Send an attitude quaternion pck
    void send_attitude_quat_pck(void *parameter);
    // Decode msg to pck
    void mavlink_decode_msg(void *parameter);
		// Generate Motor state data pck
		void generate_motor_state_data_pck(void *parameter);
		// Send Motor state data pck
		void send_motor_state_data(void *parameter);
		// Generate controller config data pck
		void generate_controller_config_data_pck(void *parameter);
		// Send controller config data pck
		void send_controller_config_data(void *parameter);
		// Generate motors config data pck
		void generate_motors_config_data_pck(void *parameter);
		// Send motors config data pck
		void send_motors_config_data(void *parameter);
		// SEND DATA TO COMPUTER 
		void respond_to_computer(void *parameter);
		// Mavlink write parameter
		void mavlink_write_param(void *parameter);
		// Mavlink read parameter
		void mavlink_read_param(void *parameter);
		// Set controller param from mavlink
		void set_controller_param_from_mavlink(void *parameter);
    // return sys events
    rt_event_t Sys_event();
    // scheduler run
    void scheduler_run();
    // gimbal
    static const Scheduler::Task scheduler_tasks[];
		
		// Encoder zero pos calibration
		void enc_zero_calibration(void);

};

extern Gimbal gimbal;

#endif
#endif
