#include "dataBox.h"
extern "C"
{
#include "e2pfs.h"
}

DataBox::DataBox()
{
	this->time_stamp = 0;
	this->read_mode = false;
}

void DataBox::imu_acc_append(float ax, float ay, float az, float ax1, float ay1, float az1)
{
	this->imu_acc.imu_axis_acc[0][this->time_stamp] = ax;
	this->imu_acc.imu_axis_acc[1][this->time_stamp] = ay;
	this->imu_acc.imu_axis_acc[2][this->time_stamp] = az;
	this->imu_acc.imu_axis_acc[3][this->time_stamp] = ax1;
	this->imu_acc.imu_axis_acc[4][this->time_stamp] = ay1;
	this->imu_acc.imu_axis_acc[5][this->time_stamp] = az1;
	this->time_stamp++;
	if(this->time_stamp == BOX_MAX_DATA_LEN) this->time_stamp = 0;
}

void DataBox::write_imu_acc_data(void)
{
  int fd;
	e2pfs_remove(70);
	e2pfs_fexist(70);
	fd = e2pfs_open(70);
	e2pfs_write(fd, &this->imu_acc, sizeof(imu_acc_data_t));
	e2pfs_close(fd);
}

void DataBox::read_imu_acc_data(void)
{
	int fd;
	e2pfs_fexist(70);
	fd = e2pfs_open(70);
	e2pfs_read(fd, &this->imu_acc, sizeof(imu_acc_data_t));
	e2pfs_close(fd);
}

void DataBox::print_imu_acc(int ax)
{
	rt_kprintf("acc axis %d\n", ax);
	for(int i = 0; i < BOX_MAX_DATA_LEN; i++)
		rt_kprintf("%d\t %d\n", (int)(1000*this->imu_acc.imu_axis_acc[ax][i]), (int)(1000*this->imu_acc.imu_axis_acc[ax+3][i]));
}

void DataBox::print_imu_acc_all(void)
{
	rt_kprintf("Acc all axis\n");
	for(int i = 0; i < BOX_MAX_DATA_LEN; i++)
		rt_kprintf("%d\t %d\t %d\t %d\t %d\t %d\n", 
	             (int)(1000*this->imu_acc.imu_axis_acc[0][i]), 
							 (int)(1000*this->imu_acc.imu_axis_acc[1][i]), 
							 (int)(1000*this->imu_acc.imu_axis_acc[2][i]), 
							 (int)(1000*this->imu_acc.imu_axis_acc[3][i]), 
							 (int)(1000*this->imu_acc.imu_axis_acc[4][i]), 
							 (int)(1000*this->imu_acc.imu_axis_acc[5][i]));
}

void DataBox::imu_data_reset(void) 
{ 
	this->time_stamp = 0; 
}
	
