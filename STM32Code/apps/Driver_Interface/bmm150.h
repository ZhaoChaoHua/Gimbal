#include <rtthread.h>
#include <stm32f4xx.h>
#include "stm32f40x_spi.h"
#include "bmm050.h"
#include "vector3.h"

class BMM150
{
	private:
		struct rt_spi_device               *bmm150_device;
	  struct bmm050_mag_data_float_t     data_float;
	  struct bmm050_mag_data_s16_t       data_s16;
	  const char                         *bmm_device_name; 
	
	public:
		Vector3f                           Mag;
	
	  BMM150(const char *bmm_name);
		void bmm150_init();
	  void get_mag_data();
	
};

