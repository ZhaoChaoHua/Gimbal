#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <can.h>
#include <bxcan.h>

// Write or Read
#define WRITE 														(1<<2)
#define READ  														(0<<2)
// Priority type
#define PRIORITY_CONTROL 									(0<<9)
#define PRIORITY_ENCODER  								(1<<9)
#define PRIORITY_RQST    									(2<<9)   
// Device address
#define CONTROLLER_ADDR  									0
#define ROLL_MOTOR_ADDR  									1
#define PITCH_MOTOR_ADDR 									2
#define YAW_MOTOR_ADDR   									3

// Roll Pitch Motor control head
#define CONTROL_MSG_RP                		(PRIORITY_CONTROL | (CONTROLLER_ADDR << 6) | (ROLL_MOTOR_ADDR << 3) | WRITE)
#define CONTROT_MSG_Y                 		(PRIORITY_CONTROL | (CONTROLLER_ADDR << 6) | (YAW_MOTOR_ADDR << 3) | WRITE)

// Encoder msg head
#define ENC_MSG_ROLL                      (PRIORITY_ENCODER | (ROLL_MOTOR_ADDR << 6) | (CONTROLLER_ADDR << 3) | WRITE)
#define ENC_MSG_PITCH                     (PRIORITY_ENCODER | (PITCH_MOTOR_ADDR << 6) | (CONTROLLER_ADDR << 3) | WRITE)
#define ENC_MSG_YAW												(PRIORITY_ENCODER | (YAW_MOTOR_ADDR << 6) | (CONTROLLER_ADDR << 3) | WRITE)

// Write/Read request
#define WRITE_RQST                        (PRIORITY_RQST | (CONTROLLER_ADDR << 6) | WRITE)
#define READ_RQST													(PRIORITY_RQST | (CONTROLLER_ADDR << 6) | READ)
#define ROLL_MOTOR_W_RQST                 (WRITE_RQST | (ROLL_MOTOR_ADDR << 3))
#define PITCH_MOTOR_W_RQST                (WRITE_RQST | (PITCH_MOTOR_ADDR << 3))
#define YAW_MOTOR_W_RQST                  (WRITE_RQST | (YAW_MOTOR_ADDR << 3))
#define ROLL_MOTOR_R_RQST                 (READ_RQST | (ROLL_MOTOR_ADDR << 3))
#define PITCH_MOTOR_R_RQST                (READ_RQST | (PITCH_MOTOR_ADDR << 3))
#define YAW_MOTOR_R_RQST                  (READ_RQST | (YAW_MOTOR_ADDR << 3))

typedef struct
{
	rt_uint32_t priority		:	2;
	rt_uint32_t src_addr    : 3;
	rt_uint32_t dst_addr    : 3;
	rt_uint32_t write       : 1;
	rt_uint32_t long_msg     : 1;
	rt_uint32_t long_msg_head : 1;
}__attribute__((packed)) can_pro_head;


void can_pro_send_control_rp(float roll_motor, float pitch_motor);

void can_pro_send_control_y(float yaw_motor);

void can_pro_write(rt_uint32_t dst_addr, rt_uint32_t param_addr, float value);

void can_pro_read(rt_uint32_t dst_addr, rt_uint32_t param_addr);

void can_pro_request(rt_can_msg* msg, rt_uint8_t read_write);

void can_pro_decode(rt_can_msg* msg);




	