#include "bmm150.h"

void BMM150_delay_msek(u32 msek);
s8 BMM150_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMM150_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);


struct rt_spi_device               *bmm150_for_global;
struct bmm050_t                    bmm150;	

BMM150::BMM150(const char *bmm_name)
{
	this->bmm_device_name = bmm_name;
}

void bmm150_device_init(const char *bmm_device_name)
{
	bmm150_for_global = (struct rt_spi_device *)rt_device_find("bmm150");
  if(bmm150_for_global == RT_NULL)
  {
        // 发送LED状态指示改变
		rt_kprintf("[err]spi device %s not found!\n", bmm_device_name);
    return;
  }
	else
	{
		rt_kprintf("found bmm150!\n");
	}
	bmm150.bus_write = BMM150_SPI_bus_write;
	bmm150.bus_read = BMM150_SPI_bus_read;
	bmm150.delay_msec = BMM150_delay_msek;
}	

void BMM150::bmm150_init()
{
	//this->bmm150_device = (struct rt_spi_device *)rt_device_find("bmm150");
//	if(this->bmm150_device == RT_NULL)
//	{
//		rt_kprintf("[err]spi device %s not found!\n", bmm_device_name);
//		return;
//	}
	s32 com_rslt = ERROR;
	u8 v_data_rate_value_u8 = BMM050_INIT_VALUE;
	
	bmm150_device_init("bmm150");
	com_rslt = bmm050_init(&bmm150);
	com_rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);
	v_data_rate_value_u8 = BMM050_DATA_RATE_30HZ;/* set data rate of 30Hz*/
	com_rslt += bmm050_set_data_rate(v_data_rate_value_u8);
	u8 v_data_rate_u8 = BMM050_INIT_VALUE;

	/* This API used to read back the written value of data rate*/
	com_rslt += bmm050_get_data_rate(&v_data_rate_u8);
	rt_kprintf("Data rate is %d\n", v_data_rate_u8);
	//com_rslt += bmm050_read_mag_data_XYZ(&data_s16);
}


int scop_mx, scop_my, scop_mz;
void BMM150::get_mag_data()
{
	
	Vector3f  mag;
	bmm050_read_mag_data_XYZ(&data_s16);
	
	Mag.x = (float)data_s16.datax;
	Mag.y = (float)data_s16.datay;
	Mag.z = (float)data_s16.dataz;
	
	scop_mx = data_s16.datax;
	scop_my = data_s16.datay;
	scop_mz = data_s16.dataz;
	
}




/************** SPI/I2C buffer length ******/
#define SPI_BUFFER_LEN 8
#define MASK_DATA1	0xFF
#define MASK_DATA2	0x80
#define MASK_DATA3	0x7F
#define	C_BMM050_ONE_U8X	(1)
#define	C_BMM050_TWO_U8X	(2)

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *		will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *              which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMM150_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	
	u8 array[SPI_BUFFER_LEN] = {MASK_DATA1};
//	u8 stringpos;
//	rt_size_t size;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as C_BMM050_ONE_U8X/write as BMM050_INIT_VALUE)*/
	/*read routine is initiated register address is mask with 0x80*/
	array[BMM050_INIT_VALUE] = reg_addr|MASK_DATA2;
	/*
	* Please take the below function as your reference for
	* read the data using SPI communication
	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+C_BMM050_ONE_U8X)"
	* add your SPI read function here
	* iError is an return value of SPI read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMM050_INIT_VALUE
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+C_BMM050_ONE_U8X operation done
	* in the SPI read
	* and write string function
	* For more information please refer data sheet SPI communication:
	*/
//	size = rt_spi_transfer(bmm150_for_global, array, array, cnt+C_BMM050_ONE_U8X);
//	if(size > 0) iError = BMM050_INIT_VALUE;
//	else iError = -1;
//	
//	for (stringpos = BMM050_INIT_VALUE; stringpos < cnt; stringpos++)
//		*(reg_data + stringpos) = array[stringpos+C_BMM050_ONE_U8X];

//	return (s8)iError;

	    struct rt_spi_message msg;
    
    /* initial message */
    msg.send_buf   = array;
    msg.recv_buf   = RT_NULL;
    msg.length     = 1;
    msg.cs_take    = 1;
    msg.cs_release = 0;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(bmm150_for_global, &msg);
    
    msg.send_buf   = RT_NULL;
    msg.recv_buf   = reg_data;
    msg.length     = cnt;
    msg.cs_take    = 0;
    msg.cs_release = 1;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(bmm150_for_global, &msg);
		
//		for (stringpos = BMM050_INIT_VALUE; stringpos < cnt; stringpos++)
//		*(reg_data + stringpos) = array[stringpos+C_BMM050_ONE_U8X];
		
		return BMM050_INIT_VALUE;
}

/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *		will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMM150_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
//	s32 iError = BMM050_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN * C_BMM050_TWO_U8X];
	u8 stringpos = BMM050_INIT_VALUE;
//	rt_size_t size;

	for (stringpos = BMM050_INIT_VALUE; stringpos < cnt; stringpos++) {
		/* the operation of (reg_addr++)&0x7F done: because it ensure
	the BMM050_INIT_VALUE and C_BMM050_ONE_U8X of the given value
	It is done only for 8bit operation*/
		array[stringpos * C_BMM050_TWO_U8X] = (reg_addr++) & MASK_DATA3;
		array[stringpos * C_BMM050_TWO_U8X + C_BMM050_ONE_U8X] =
		*(reg_data + stringpos);
	}
	/* Please take the below function as your reference
	 * for write the data using SPI communication
	 * add your SPI write function here.
	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*C_BMM050_TWO_U8X)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
     * In the driver SUCCESS defined as BMM050_INIT_VALUE
     * and FAILURE defined as -1
	 */

	  struct rt_spi_message msg;
    
    /* initial message */
    msg.send_buf   = array;
    msg.recv_buf   = RT_NULL;
    msg.length     = cnt*C_BMM050_TWO_U8X;
    msg.cs_take    = 1;
    msg.cs_release = 1;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(bmm150_for_global, &msg);
		
		return BMM050_INIT_VALUE;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMM150_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	rt_thread_delay(1);
}
