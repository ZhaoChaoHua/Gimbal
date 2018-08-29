#ifndef __UART_COM_H_
#define __UART_COM_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <stm32f4xx.h>

#ifdef __cplusplus

class SerialManager
{
private:
	const char *_rw_name;
	
public:
	
  rt_device_t rwdevice;

	SerialManager(const char *rw_name);

	bool start_init();
	
};

#endif
#endif

