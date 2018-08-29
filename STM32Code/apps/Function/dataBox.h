#include <rtthread.h>
#include "stm32f4xx.h"
#include <AP_Math.h>
#include "AP_Progmem.h"

#define BOX_MAX_DATA_LEN 1024
#define WRITE_DATA_THREASHOLD  BOX_MAX_DATA_LEN - 20

typedef struct
{
	float imu_axis_acc[6][BOX_MAX_DATA_LEN];
}__attribute__((packed)) imu_acc_data_t;
	
class DataBox
{
	public:
		DataBox();
		imu_acc_data_t  imu_acc;
	  int  time_stamp;
	  bool read_mode;
	
	  void imu_acc_append(float ax, float ay, float az, float ax1, float ay1, float az1);
	  void write_imu_acc_data(void);
	  void read_imu_acc_data(void);
	  void print_imu_acc(int ax);
	  void imu_data_reset(void);
	  void print_imu_acc_all(void);
};

