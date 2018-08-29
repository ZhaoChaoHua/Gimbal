#ifndef __THREAD_CONFIG_H__
#define __THREAD_CONFIG_H__
 
#include <rtthread.h>
#include "stm32f4xx.h"

#ifdef __cplusplus

// serial manager thread
#define SERIAL_PROCESS_STACK       2048
#define SERIAL_PROCESS_PRIORITY    0x10
#define SERIAL_PROCESS_TICK        6

// attitude thread
#define ATTITUDE_PROCESS_STACK     1024
#define ATTITUDE_PROCESS_PRIORITY  0x3
#define ATTITUDE_PROCESS_TICK      2

// control thread
#define CONTROL_PROCESS_STACK      1024
#define CONTROL_PROCESS_PRIORITY   0x5
#define CONTROL_PROCESS_TICK       2

// can thread
#define CAN_PROCESS_STACK          1024
#define CAN_PROCESS_PRIORITY       0x6
#define CAN_PROCESS_TICK           4

// packing thread
#define PACKING_PROCESS_STACK      1024
#define PACKING_PROCESS_PRIORITY   0x11
#define PACKING_PROCESS_TICK       2

// packing thread
#define LOG_PROCESS_STACK      1024
#define LOG_PROCESS_PRIORITY   0x15
#define LOG_PROCESS_TICK       2

// Acc calibration thread
#define ACC_CAL_PROCESS_STACK      1024
#define ACC_CAL_PROCESS_PRIORITY   0x17
#define ACC_CAL_PROCESS_TICK       2

// Direction thread
#define DIRECTION_PROCESS_STACK      1024
#define DIRECTION_PROCESS_PRIORITY   0x16
#define DIRECTION_PROCESS_TICK       2


// system events flags
#define ATTITUDE_EVENT   (1<<0)
#define CONTROL_EVENT    (1<<1)
#define SERIAL_EVENT     (1<<2)
#define PACKET_EVENT     (1<<3)
#define LOG_EVENT        (1<<4)
#define ACC_CAL_EVENT    (1<<5)
#define DIRECTION_EVENT  (1<<6)


#endif
#endif
