#include "mavlink_com.h"

Mavlink::Mavlink(const char *rw_name, QRingBuffer &ring):
	SerialManager(rw_name),
  ringbuf(ring)
{
	_mavlink_rx_sta = MAVLINK_RX_STOP;
  _mavlink_rx_data_num = 0;
	_overdrop = 0;
}

// Convert a Mavlink msg to a buffer[] to send
rt_size_t Mavlink::mavlink2serial(Mavlink_msg_t *msg, uint8_t *buf)
{
	rt_size_t len;
	uint8_t i;
	len = (rt_size_t)(msg->payload_lth+8);
	buf[0] = msg->stx;
	buf[1] = msg->payload_lth;
	buf[2] = msg->packet_seq;
	buf[3] = msg->sys_id;
	buf[4] = msg->comp_id;
	buf[5] = msg->msg_id;
	for(i = 0; i < msg->payload_lth; i++)
	{
		buf[6+i] = msg->data[i];
	}
	buf[len-2] = msg->crcl;
	buf[len-1] = msg->crch;
	return len;
}


// Send a mavlink msg from serial
rt_size_t Mavlink::send_mavlink(Mavlink_msg_t *msg)
{
	uint8_t buf[MAX_MAV_LENTH];
	rt_size_t len;
	len = mavlink2serial(msg,buf);
	return rt_device_write(rwdevice,0,buf,len);
}

// Convert a buffer to Mavlink msg
void Mavlink::receive_mavlink(void)
{
	int len;
    rt_size_t ret;
	uint8_t buf[256];
    rt_device_control(rwdevice, RT_DEVICE_CTRL_DATA_READY, &len);
    
	if(len > 0)
	{
        ret = rt_device_read(rwdevice,0,buf,len);
        for(int i=0; i< ret; i++)
        {
            switch(_mavlink_rx_sta)
            {
                case MAVLINK_RX_STOP:
                    if(buf[i] == 0xFE)
                    {
                        _mavlink_rx_sta = MAVLINK_RX_STX;
                        mavlink_msg_rx.stx = buf[i];
                    }
                    break;
                case MAVLINK_RX_STX:
                    _mavlink_rx_sta = MAVLINK_RX_PLTH;
                    mavlink_msg_rx.payload_lth = buf[i];
                    break;
                case MAVLINK_RX_PLTH:
									_mavlink_rx_sta = MAVLINK_RX_PSEQ;
								  mavlink_msg_rx.packet_seq = buf[i];
								  break;
								case MAVLINK_RX_PSEQ:
									_mavlink_rx_sta = MAVLINK_RX_SYSID;
								  mavlink_msg_rx.sys_id = buf[i];
								  break;
								case MAVLINK_RX_SYSID:
									_mavlink_rx_sta = MAVLINK_RX_COMPID;
								  mavlink_msg_rx.comp_id = buf[i];
								  break;
								case MAVLINK_RX_COMPID:
									_mavlink_rx_sta = MAVLINK_RX_MSGID;
                  _mavlink_rx_data_num = 0;
                  mavlink_msg_rx.msg_id = buf[i];
                  break;
                case MAVLINK_RX_MSGID:	
                    if(_mavlink_rx_data_num < mavlink_msg_rx.payload_lth)
                    {
                        mavlink_msg_rx.data[_mavlink_rx_data_num] = buf[i];
                        _mavlink_rx_data_num++;
                    }
                    if(_mavlink_rx_data_num == mavlink_msg_rx.payload_lth)
                    {
                        _mavlink_rx_data_num = 0;
                        _mavlink_rx_sta = MAVLINK_RX_DATA;
                    }
                    break;
                case MAVLINK_RX_DATA:
                    _mavlink_rx_sta = MAVLINK_RX_CRCL;
                    mavlink_msg_rx.crcl = buf[i];
                    break;
                case MAVLINK_RX_CRCL:
                    _mavlink_rx_sta = MAVLINK_RX_CRCH;
                    mavlink_msg_rx.crch = buf[i];
                    _mavlink_rx_sta = MAVLINK_RX_STOP;
                    ready_to_decode = true;
                    rt_device_control(rwdevice, RT_DEVICE_CTRL_DATA_CLEAN, 0);
                    
//                    if(mavlink_crc_check(&mavlink_msg_rx, MSG_CRC))
                    {
                        if(!ringbuf.isFull())
                        {
                            ringbuf.write(&mavlink_msg_rx, 1);
                        }
                        else
                            _overdrop++;
                    }
								
                    break;
                default:
                    break;
            }
        }
	}
}


// Extract a float number frome Mavlink msg->data
float return_float(const Mavlink_msg_t *msg, uint8_t ofs) 
{ 
	return *(const float *)(&msg->data[ofs]);
}
// Extract a uint32 number frome Mavlink msg->data
uint32_t return_uint32(const Mavlink_msg_t *msg, uint8_t ofs)
{  
	return *(const uint32_t *)(&msg->data[ofs]);
}


// Encoding an attitude package bo a Mavlink msg
uint16_t Mavlink::attitude_quaternion_pack(uint8_t system_id, uint8_t component_id, Mavlink_msg_t* msg,
																					 uint32_t time_boot_ms,
																					 float q1, float q2, float q3, float q4,
																					 float rollspeed, float pitchspeed, float yawspeed)
{
    Mavlink_attitude_quaternion_t pck;
    pck.time_boot_ms = time_boot_ms;
    pck.q1 = q1;
    pck.q2 = q2;
    pck.q3 = q3;
    pck.q4 = q4;
    pck.rollspeed = rollspeed;
    pck.pitchspeed = pitchspeed;
    pck.yawspeed  = yawspeed;

    msg->stx = MAVLINK_STX;
    msg->payload_lth = MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN;
    msg->packet_seq = 0;
    msg->sys_id = system_id;
    msg->comp_id = component_id;
    msg->msg_id = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
    rt_memcpy(msg->data,&pck,MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
    mavlink_crc_get(msg,MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC);
	
	 return 0;
}

// Decoding a Mavlink msg to an attitude package 
void Mavlink::attitude_quaternion_decode(const Mavlink_msg_t* msg, Mavlink_attitude_quaternion_t* pck)
{
	uint8_t len = msg->payload_lth < MAVLINK_MSG_ID_ATTITUDE_QUATERNION ? msg->payload_lth : MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
	rt_memcpy(pck,msg->data,len);
}


// Encoding a Control parameter to a Mavlink message
uint16_t Mavlink::control_param_pack(uint8_t system_id, uint8_t component_id, Mavlink_msg_t* msg,
																uint8_t axis_id, float rate_p, float rate_i, float rate_d, float rate_i_max, float rate_filt_hz,
																float stabilize_p, float stabilize_d)
{
    uint16_t res;
    Mavlink_control_param_t pck;
    pck.axis_id = axis_id;
    pck.rate_p = rate_p;
    pck.rate_i = rate_i;
    pck.rate_d = rate_d;
    pck.rate_i_max = rate_i_max;
    pck.rate_filt_hz = rate_filt_hz;
    pck.stabilize_p = stabilize_p;
    pck.stabilize_d = stabilize_d;

    msg->stx = MAVLINK_STX;
    msg->payload_lth = MAVLINK_MSG_ID_CONTROL_PARAM_LEN;
    msg->packet_seq = 0;
    msg->sys_id = system_id;
    msg->comp_id = component_id;
    msg->msg_id = MAVLINK_MSG_ID_CONTROL_PARAM;
    rt_memcpy(msg->data, &pck, MAVLINK_MSG_ID_CONTROL_PARAM_LEN);
    res = mavlink_crc_get(msg, MAVLINK_MSG_ID_CONTROL_PARAM_CRC);
    return res;
}

// Decode a Mavlink message to a Control parameter pck
void Mavlink::control_param_decode(const Mavlink_msg_t* msg, Mavlink_control_param_t* control_param)	
{
		uint8_t len = msg->payload_lth < MAVLINK_MSG_ID_CONTROL_PARAM_LEN ? msg->payload_lth : MAVLINK_MSG_ID_CONTROL_PARAM_LEN;
		rt_memcpy(control_param, msg->data, len);
}



uint16_t Mavlink::motor_state_data_packing(uint8_t system_id, uint8_t component_id, Mavlink_msg_t* msg,
                                           Mavlink_motor_state_data_t *pck)
{
	uint16_t res;
	msg->stx = MAVLINK_STX;
	msg->payload_lth = MAVLINK_MSG_ID_MOTOR_STATE_DATA_LEN;
	msg->packet_seq = 0;
	msg->sys_id = system_id;
	msg->comp_id = component_id;
	msg->msg_id = MAVLINK_MSG_ID_MOTOR_STATE_DATA;
	rt_memcpy((void*)(&(msg->data)), pck, MAVLINK_MSG_ID_MOTOR_STATE_DATA_LEN);
	res = mavlink_crc_get(msg, MAVLINK_MSG_ID_MOTOR_STATE_DATA_CRC);
	return res;
}	

uint16_t Mavlink::controller_config_data_packing(uint8_t system_id, uint8_t component_id, Mavlink_msg_t* msg,
																            Mavlink_controller_config_data_t *pck)
{
	uint16_t res;
	msg->stx = MAVLINK_STX;
	msg->payload_lth = MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA_LEN;
	msg->packet_seq = 0;
	msg->sys_id = system_id;
	msg->comp_id = component_id;
	msg->msg_id = MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA;
	rt_memcpy(msg->data, pck, MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA_LEN);
	res = mavlink_crc_get(msg, MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA_CRC);
	return res;
	
}
																 
uint16_t Mavlink::motors_config_data_packing(uint8_t system_id, uint8_t component_id, Mavlink_msg_t* msg,
																        Mavlink_motors_config_data_t *pck)
{
	uint16_t res;
	msg->stx = MAVLINK_STX;
	msg->payload_lth = MAVLINK_MSG_ID_MOTORS_CONFIG_DATA_LEN;
	msg->packet_seq = 0;
	msg->sys_id = system_id;
	msg->comp_id = component_id;
	msg->msg_id = MAVLINK_MSG_ID_MOTORS_CONFIG_DATA;
	rt_memcpy(msg->data, pck, MAVLINK_MSG_ID_MOTORS_CONFIG_DATA_LEN);
	res = mavlink_crc_get(msg, MAVLINK_MSG_ID_MOTORS_CONFIG_DATA_CRC);
	return res;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Gimbal control
// return the gimbal param id which will be control
uint8_t Mavlink::gimbal_control_data_decode(const Mavlink_msg_t* msg, Vector3f* vec)
{
	  uint8_t id = msg->data[0];
		uint8_t len = msg->payload_lth < MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN ? msg->payload_lth : MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN;
		rt_memcpy(vec, &(msg->data[1]), len-1);	
	  return id;
}

// Gimbal command
// return command id 
int Mavlink::gimbal_command_data_decode(const Mavlink_msg_t* msg)
{
	Mavlink_gimbal_main_command_data_t pck;
	uint8_t len = msg->payload_lth < MAVLINK_MSG_ID_GIMBAL_MAIN_COMMAND_LEN ? msg->payload_lth : MAVLINK_MSG_ID_GIMBAL_MAIN_COMMAND_LEN;
	rt_memcpy(&pck, msg->data, len);	
	return pck.command;
}

uint8_t Mavlink::drone_data_decode(const Mavlink_msg_t* msg, Vector3f* mag, Vector3f* vel)
{
	Mavlink_gimbal_drone_data_t pck;
	uint8_t len = msg->payload_lth < MAVLINK_MSG_ID_GIMBAL_DRONE_DATA_LEN ? msg->payload_lth : MAVLINK_MSG_ID_GIMBAL_DRONE_DATA_LEN;
	rt_memcpy(&pck, msg->data, len);	
	mag->x = pck.mag[0];
	mag->y = pck.mag[1];
	mag->z = pck.mag[2];
	vel->x = pck.vel[0];
	vel->y = pck.vel[1];
	vel->z = pck.vel[2];
	
	return 0;
}
																							


/**
 * @brief Accumulate the X.25 CRC by adding one char at a time.
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new char to hash
 * @param crcAccum the already accumulated checksum
 **/
void Mavlink::crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        /*Accumulate one byte of data into the CRC*/
        uint8_t tmp;

        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}


/**
 * @brief Initiliaze the buffer for the X.25 CRC
 *
 * @param crcAccum the 16 bit X.25 CRC
 */
void Mavlink::crc_init(uint16_t* crcAccum)
{
        *crcAccum = X25_INIT_CRC;
}


/**
 * @brief Calculates the X.25 checksum on a byte buffer
 *
 * @param  pBuffer buffer containing the byte array to hash
 * @param  length  length of the byte array
 * @return the checksum over the buffer bytes
 **/
uint16_t Mavlink::crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
        uint16_t crcTmp;
        crc_init(&crcTmp);
	while (length--) {
                crc_accumulate(*pBuffer++, &crcTmp);
        }
        return crcTmp;
}


/**
 * @brief Accumulate the X.25 CRC by adding an array of bytes
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 **/
void Mavlink::crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--) {
                crc_accumulate(*p++, crcAccum);
        }
}

uint16_t Mavlink::mavlink_crc_get(Mavlink_msg_t* msg, uint8_t crc_extra)
{
	uint16_t checksum;
	checksum=crc_calculate(((const uint8_t*)(msg))+1,5);
	crc_accumulate_buffer(&checksum, ((const char *)(&((msg)->data[0]))),msg->payload_lth);
	crc_accumulate(crc_extra, &checksum);
	msg->crcl=((checksum)&0xFF);
	msg->crch=((checksum)>>8);
	
	return (msg->payload_lth)+8;
}	

bool Mavlink::mavlink_crc_check(Mavlink_msg_t* msg, uint8_t crc_extra)
{
    uint16_t checksum;

    checksum=crc_calculate(((const uint8_t*)(msg)),3);
    crc_accumulate_buffer(&checksum, ((const char *)(&((msg)->data[0]))),msg->payload_lth);
    crc_accumulate(crc_extra, &checksum);

    if((msg->crcl==((checksum)&0xFF))&&(msg->crch==((checksum)>>8)))
        return true;
    else
        return false;
}

