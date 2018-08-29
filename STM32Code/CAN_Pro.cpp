#include "CAN_Pro.h"
/*
typedef struct
{
	rt_uint32_t priority		:	2;
	rt_uint32_t src_addr    : 3;
	rt_uint32_t dst_addr    : 3;
	rt_uint32_t write       : 1;
	rt_uint32_t long_msg     : 1;
	rt_uint32_t long_msg_head : 1;
}__attribute__((packed)) can_pro_head;
*/

void can_pro_send_control_rp(float roll_motor, float pitch_motor)
{
	rt_can_msg msg;
	msg.id = CONTROL_MSG_RP;
}
	

void can_pro_send_control_y(float yaw_motor);

void can_pro_write(rt_uint32_t dst_addr, rt_uint32_t param_addr, float value);

void can_pro_read(rt_uint32_t dst_addr, rt_uint32_t param_addr);

void can_pro_request(rt_can_msg* msg, rt_uint8_t read_write);

void can_pro_decode(rt_can_msg* msg);