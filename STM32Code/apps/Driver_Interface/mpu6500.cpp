#include "mpu6500.h"
#include "AP_Math.h"
#ifdef __cplusplus
extern "C"
{
#endif
#include "gpio.h"
#ifdef __cplusplus
}
#endif

// table of user settable parameters
const AP_Param::GroupInfo MPU6500::var_info[] PROGMEM =
{
    // @Param: ACC_F_X
    // @DisplayName: accel offset x
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 0
    // @User: Advanced
    AP_GROUPINFO("ACC_F_X",    0, MPU6500, accel_offs_x, 0.0f),
    
    // @Param: ACC_F_Y
    // @DisplayName: accel offset y
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 0
    // @User: Advanced
    AP_GROUPINFO("ACC_F_Y",    1, MPU6500, accel_offs_y, 0.0f),

    // @Param: ACC_F_Z
    // @DisplayName: accel offset z
    // @Description: 
    // @Units: 
    // @Range: 
    // @Values: 
    // @Increment: 0
    // @User: Advanced
    AP_GROUPINFO("ACC_F_Z",  2, MPU6500, accel_offs_z, 0.0f),

    // @Param: ACC_T_X
    // @DisplayName: 
    // @Description: 
    // @Values: 1
    // @User: Advanced
    AP_GROUPINFO("ACC_T_X", 3, MPU6500, accel_T_x, 1.0f),

    // @Param: ACC_T_Y
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 1
    // @Values: 
    // @User: Advanced
    AP_GROUPINFO("ACC_T_Y", 4, MPU6500, accel_T_y, 1.0f),

    // @Param: ACC_T_Z
    // @DisplayName: 
    // @Description: 
    // @Units: 
    // @Range: 
    // @Increment: 
    // @Values: 
    // @User: Advanced
    AP_GROUPINFO("ACC_T_Z", 5, MPU6500, accel_T_z, 1.0f),

    AP_GROUPEND
};

// MPU6500构造函数
MPU6500::MPU6500(const char *mpu_name)
{
    this->mpu_device_name = mpu_name;
    this->mpu_send_buf[0]  = MPUREG_ACCEL_XOUT_H | READ_FLAG;
}

/* INITIALIZATION
 * usage: call this function at startup, giving the sample rate divider (raging from 0 to 255)
 * 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
 * low pass filter value; suitable values are:
 * BITS_DLPF_CFG_256HZ_NOLPF2
 * BITS_DLPF_CFG_188HZ
 * BITS_DLPF_CFG_98HZ
 * BITS_DLPF_CFG_42HZ
 * BITS_DLPF_CFG_20HZ
 * BITS_DLPF_CFG_10HZ
 * BITS_DLPF_CFG_5HZ 
 * BITS_DLPF_CFG_2100HZ_NOLPF
 */

void MPU6500::mpu_init(uint8_t sample_rate_div,uint8_t low_pass_filter, int id)
{
	if(id == 0) this->mpu6500_device = (struct rt_spi_device *)rt_device_find("mpu65");
	if(id == 1) this->mpu6500_device = (struct rt_spi_device *)rt_device_find("mpu65_1");
    if(this->mpu6500_device == RT_NULL)
    {
        // 发送LED状态指示改变
        rt_kprintf("[err]spi device %s not found!\n", mpu_device_name);
        return;
    }
    if(!this->testConnect())
    {
        rt_kprintf("[err]mpu6500 not found!\n");
        return;
    }
    rt_kprintf("[init]Find mpu6500 Device.\n");
    this->disable_I2C();
    this->reSetMPU6500();
    // delay 500ms
    rt_thread_delay(50);
    //WAKE UP AND SET GYROZ CLOCK
    this->setClockSource(MPU_CLK_SEL_PLLGYROZ);
    this->disable_I2C();
    rt_thread_delay(1);
    //SET SAMPLE RATE
    this->setSampleRate(sample_rate_div);
    rt_thread_delay(1);
    // FS & DLPF
    this->setDLPF(low_pass_filter);
    rt_thread_delay(1);
    //DISABLE INTERRUPTS
    this->disableInterrupt();
    rt_thread_delay(1);
		
		accel_T_x = accel_T_y = accel_T_z = 1.0f;
}

/* MPUREG_USER_CTRL Disable I2C mode, just put the serial interface in SPI
 * [7] 1 - Enables DMP features.
 * [6] 1 - Enable FIFO operation mode.
 * [5] 1 - Enable the I2C Master I/F module
 * [4] 1 - Reset I2C Slave module and put the serial interface in SPI mode only.
 * [3] This bit resets the DMP when set to 1 while DMP_EN equals 0
 * [2] 1 - Reset FIFO module
 * [1] 1 - Reset I2C Master module
 * [0] 1 - Reset all gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers
 */
void MPU6500::disable_I2C()
{
    uint8_t send_buf[2];
    send_buf[0] = MPUREG_USER_CTRL; // 0x6A
    send_buf[1] = BIT_I2C_IF_DIS;   // 0x10
    rt_spi_send_then_send(this->mpu6500_device,&send_buf[0],1,&send_buf[1],1);
}

/* MPUREG_PWR_MGMT_1
 * [7]   1 - Reset the internal registers and restores the default settings. Write a 1 to
 *       set the reset, the bit will auto clear.
 * [6]   When set, the chip is set to sleep mode.
 * [5]
 * [4]   When set, the gyro drive and pll circuitry are enabled, but the sense paths
 *       are disabled. This is a low power mode that allows quick enabling of the
 *       gyros
 * [3]   When set to 1, this bit disables the temperature sensor
 * [2:0] clock source
 */
void MPU6500::reSetMPU6500()
{
    uint8_t send_buf[2];
    send_buf[0] = MPUREG_PWR_MGMT_1; // 0x6B
    send_buf[1] = BIT_H_RESET;       // 0x80
    rt_spi_send_then_send(this->mpu6500_device,&send_buf[0],1,&send_buf[1],1);
}

/** Get clock source setting.
 * @return Current clock source setting
 */
uint8_t MPU6500::getClockSource() 
{
    uint8_t send_buf;
    uint8_t recv_buf;
    send_buf = MPUREG_PWR_MGMT_1 | READ_FLAG;
    rt_spi_send_then_recv(this->mpu6500_device,&send_buf,1,&recv_buf,1);
    return recv_buf;
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 */
void MPU6500::setClockSource(uint8_t source)
{
    uint8_t send_buf[2];
    send_buf[0] = MPUREG_PWR_MGMT_1;   // 0x6B
    send_buf[1] = source;              // 0x03
    rt_spi_send_then_send(this->mpu6500_device,&send_buf[0],1,&send_buf[1],1);
}

/*
 * Divides the internal sample rate (see register CONFIG) to generate the
 * sample rate that controls sensor data output rate, FIFO sample rate.
 * NOTE: This register is only effective when FCHOICE = 2抌11 (FCHOICE_B
 * register bits are 2抌00), and (0 < DLPF_CFG < 7)
 * This is the update rate of sensor register.
 * SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
 * where INTERNAL_SAMPLE_RATE = 1kHz
 */
void MPU6500::setSampleRate(uint8_t sample_rate)
{
    uint8_t send_buf[2];
    send_buf[0] = MPUREG_SMPLRT_DIV;
    send_buf[1] = sample_rate;
    rt_spi_send_then_send(this->mpu6500_device,&send_buf[0],1,&send_buf[1],1);
}
uint8_t MPU6500::getSampleRate()
{
    uint8_t send_buf;
    uint8_t recv_buf;
    send_buf = MPUREG_SMPLRT_DIV | READ_FLAG;
    rt_spi_send_then_recv(this->mpu6500_device,&send_buf,1,&recv_buf,1);
    return recv_buf;
}

void MPU6500::setDLPF(uint8_t low_pass)
{
    uint8_t send_buf[2];
    send_buf[0] = MPUREG_CONFIG;  // 0x1A
    send_buf[1] = low_pass;       // 
    rt_spi_send_then_send(this->mpu6500_device,&send_buf[0],1,&send_buf[1],1);
}

void MPU6500::disableInterrupt()
{
    uint8_t send_buf[2];
    send_buf[0] = MPUREG_INT_ENABLE; // 0x38
    send_buf[1] = 0x00;              // 0 - disable
    rt_spi_send_then_send(this->mpu6500_device,&send_buf[0],1,&send_buf[1],1);
}

/*
                                ACCELEROMETER SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * accelerometers. Suitable ranges are:
 * BITS_FS_2G
 * BITS_FS_4G
 * BITS_FS_8G
 * BITS_FS_16G
 * returns the range set (2,4,8 or 16)
 */
uint8_t MPU6500::set_acc_scale(uint8_t scale)
{
    uint8_t send_buf[2];
    uint8_t recv;
    send_buf[0] = MPUREG_ACCEL_CONFIG; // 0x1C
    send_buf[1] = scale;                // 
    rt_spi_send_then_send(this->mpu6500_device,&send_buf[0],1,&send_buf[1],1);
    switch(scale)
    {
        case BITS_FS_2G:
            accel_scale=16384;
        break;
        case BITS_FS_4G:
            accel_scale=8192;
        break;
        case BITS_FS_8G:
            accel_scale=4096;
        break;
        case BITS_FS_16G:
            accel_scale=2048;
        break; 
    }
    accel_scale = 1.0f/accel_scale * CONSTANTS_ONE_G * 1.0f;
    
    rt_thread_delay(1); // 10ms
    send_buf[0] = MPUREG_ACCEL_CONFIG | READ_FLAG;
    rt_spi_send_then_recv(this->mpu6500_device,&send_buf[0],1,&recv,1);
    switch (recv)
    {
        case BITS_FS_2G:
            recv=2;
        break;
        case BITS_FS_4G:
            recv=4;
        break;
        case BITS_FS_8G:
            recv=8;
        break;
        case BITS_FS_16G:
            recv=16;
        break;   
    }
    rt_kprintf("[init]MPU6500 ACC Scale is %dG\r\n", recv);
    return recv;
}

/*
                                GYROSCOPE SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * gyroscopes. Suitable ranges are:
 * BITS_FS_250DPS
 * BITS_FS_500DPS
 * BITS_FS_1000DPS
 * BITS_FS_2000DPS
 * returns the range set (250,500,1000 or 2000)
 */
uint16_t MPU6500::set_gyro_scale(uint8_t scale)
{
    uint8_t send_buf[2];
    uint8_t recv;
    uint16_t temp;
    send_buf[0] = MPUREG_GYRO_CONFIG;   // 0x1C
    send_buf[1] = scale;                // 
    rt_spi_send_then_send(this->mpu6500_device,&send_buf[0],1,&send_buf[1],1);
    switch(scale)
    {
        case BITS_FS_250DPS:
            gyro_scale=131;
        break;
        case BITS_FS_500DPS:
            gyro_scale=65.5;
        break;
        case BITS_FS_1000DPS:
            gyro_scale=32.8;
        break;
        case BITS_FS_2000DPS:
            gyro_scale=16.4;
        break;
    }
    gyro_scale = 1.0f/gyro_scale * DEG_TO_RAD * 1.0f;
    
    rt_thread_delay(1); // 10ms
    send_buf[0] = MPUREG_GYRO_CONFIG | READ_FLAG;
    rt_spi_send_then_recv(this->mpu6500_device,&send_buf[0],1,&recv,1);
    switch (recv)
    {
        case BITS_FS_250DPS:
            temp=250;
        break;
        case BITS_FS_500DPS:
            temp=500;
        break;
        case BITS_FS_1000DPS:
            temp=1000;
        break;
        case BITS_FS_2000DPS:
            temp=2000;
        break;
    }
    rt_kprintf("[init]MPU6500 GYRO Scale is %dDPS\r\n", temp);
    return temp;
}


void MPU6500::read_Data(void)
{
    struct rt_spi_message msg;
    
    /* initial message */
    msg.send_buf   = this->mpu_send_buf;
    msg.recv_buf   = RT_NULL;
    msg.length     = 1;
    msg.cs_take    = 1;
    msg.cs_release = 0;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(this->mpu6500_device, &msg);
    
    msg.send_buf   = RT_NULL;
    msg.recv_buf   = this->mpu_recv_buf;
    msg.length     = 14;
    msg.cs_take    = 0;
    msg.cs_release = 1;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(this->mpu6500_device, &msg);
    
    // 读取
    Acc_I16.x = (mpu_recv_buf[1]  | (short)(mpu_recv_buf[0] << 8));
    Acc_I16.y = (mpu_recv_buf[3]  | (short)(mpu_recv_buf[2] << 8));
    Acc_I16.z = (mpu_recv_buf[5]  | (short)(mpu_recv_buf[4] << 8));
    
    Tempreature = (mpu_recv_buf[7]  | (short)(mpu_recv_buf[6] << 8));
    
    Gyro_I16.x  = (mpu_recv_buf[9]  | (short)(mpu_recv_buf[8] << 8));
    Gyro_I16.y  = (mpu_recv_buf[11] | (short)(mpu_recv_buf[10] << 8));
    Gyro_I16.z  = (mpu_recv_buf[13] | (short)(mpu_recv_buf[12] << 8));
    
    Gyro_rad.x = (Gyro_I16.x + gyro_offset.x) * gyro_scale;
    Gyro_rad.y = (Gyro_I16.y + gyro_offset.y) * gyro_scale;
    Gyro_rad.z = (Gyro_I16.z + gyro_offset.z) * gyro_scale;
    
    Acc_ms2.x = Acc_I16.x * accel_scale;
    Acc_ms2.y = Acc_I16.y * accel_scale;
    Acc_ms2.z = Acc_I16.z * accel_scale;
    
		  //D1
		Gyro_rad.rotate((enum Rotation)ROTATION_YAW_90);
    Acc_ms2.rotate((enum Rotation)ROTATION_YAW_90);
		Gyro_rad.rotate((enum Rotation)ROTATION_PITCH_270);
    Acc_ms2.rotate((enum Rotation)ROTATION_PITCH_270);
		
//    
    Acc_correct.x = (Acc_ms2.x - accel_offs_x) * accel_T_x;
    Acc_correct.y = (Acc_ms2.y - accel_offs_y) * accel_T_y;
    Acc_correct.z = (Acc_ms2.z - accel_offs_z) * accel_T_z;
    
    temp_deg = Tempreature / 340.0f + 36.53f;
}

float vector_dis_2(Vector3i a, Vector3i b)
{
	return (float)((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
}



// Gyro Calibration
void MPU6500::gyroCalibration(void)
{
	#define COLLECT_NUM  7
	int times_out = 50;
	int times_count = 0;
	int data_len = 800;
	int err_count = 0;
	int err_out = 10;
	int bad_data_out = 20;
	
	float dis_threashold = 500;
	float acc_dis_2[COLLECT_NUM];
	float rot_dis_lim = 100000;
	float rot_dis;
	Vector3i acc_start, acc_end;
	Vector3i gyro;
	Vector3f gyro_o[COLLECT_NUM];
	
	rt_kprintf("Gyro Calibrating...\n");
	
	int i = 0;
	
	// Collect COLLECT_NUM gyro data
	while(i < COLLECT_NUM)
	{
		acc_dis_2[i] = dis_threashold;
		times_count = 0;
		// While acc distand is smaller than threashold, accept the gyro data, if acc distand not ok, keep collecting until times out
		while(acc_dis_2[i] >= dis_threashold)
		{
			gyro_o[i].x = gyro_o[i].y = gyro_o[i].z = 0;
			read_acc(&acc_start.x, &acc_start.y, &acc_start.z);
			for(int j = 0; j < data_len; j++)
			{
				read_gyro((int16_t *)&gyro.x, &gyro.y, &gyro.z);
				gyro_o[i].x += -(float)gyro.x;
				gyro_o[i].y += -(float)gyro.y;
				gyro_o[i].z += -(float)gyro.z;
			}
			read_acc(&acc_end.x, &acc_end.y, &acc_end.z);
			acc_dis_2[i] = vector_dis_2(acc_end, acc_start);
			gyro_o[i] /= (float)data_len;
//			rt_kprintf("collecton %d: \tdistance: %d \t\t", i, (int)acc_dis_2[i]);
//			rt_kprintf("offset: %d\t %d\t %d\t\t bad data: %d\n", (int)gyro_o[i].x, (int)gyro_o[i].y, (int)gyro_o[i].z, times_count); 
			times_count++;
			if(times_count > bad_data_out)
			{
				i=0; 
				rt_thread_delay(6);
			}
			if(times_count >= times_out) 
			{
				times_count = 0;
				break;
			}
			if(acc_dis_2[i] > dis_threashold) 
			{
				rt_thread_delay(6);
			}
		}
		// Accept gyro data
		if(acc_dis_2[i] < dis_threashold)
		{
			rot_dis = gyro_o[i].x*gyro_o[i].x + gyro_o[i].y*gyro_o[i].y + gyro_o[i].z*gyro_o[i].z;
			if(rot_dis < rot_dis_lim)
			{
				rt_kprintf("collecton %d: \tacc distance: %d \t\t", i, (int)acc_dis_2[i]);
				rt_kprintf("offset: \t%d \t %d\t %d\t\t bad data: %d \trotation dis: %d\n", (int)gyro_o[i].x, (int)gyro_o[i].y, (int)gyro_o[i].z, times_count, (int)rot_dis); 
				i++;
			}
			else
				rt_thread_delay(2);
//			rt_kprintf("ok\n");
		}
		// Delay and keep collecting
		else
		{
			rt_kprintf("\n");
			rt_thread_delay(10);
			err_count++;
			if(err_count >= err_out)
			{
				rt_kprintf("times out.\n");
				rt_kprintf("Gyro calibration fail...\n");
				return;
			}
		}
	}
	// Make sure we have collect COLLECT_NUM datas
	if(i == COLLECT_NUM)
	{
		// Take out the best data
		Vector3i temp;
		for(int k = 1; k < COLLECT_NUM; k++)
		{
			if(acc_dis_2[k] < acc_dis_2[0])
			{
				acc_dis_2[0] = acc_dis_2[k];
				gyro_o[0] = gyro_o[k];
			}
		}
		// Set gyro offset
		// put result into integer
    gyro_offset(gyro_o[0].x,gyro_o[0].y,gyro_o[0].z);
		rt_kprintf("Gyro calibration success!\n");
		rt_kprintf("Gyro offset: \t%d \t%d \t%d\n",(int)gyro_offset.x, (int)gyro_offset.y, (int)gyro_offset.z);
		return;
	}
	else
	{
		rt_kprintf("Gyro calibration fail...\n");
		rt_kprintf("Collectin number: %d", i);
		return;
	}	
}
		

// 陀螺仪校准
void MPU6500::gyroOffsetCalibration(void)
{
    int i;
    #define TOL               64
    //#define GYRO_INTERATIONS  2000
    int16_t prevGyro[3],gyro[3];
    float fp_gyroOffset[3];
    uint8_t tiltDetected = 0;
    int calibGCounter = GYRO_INTERATIONS;
    uint32_t timeout = rt_tick_get()+1000; // 30s超时时间
    // wait 1s
    rt_thread_delay(50);

    while(calibGCounter>0)
    {
        if(calibGCounter==GYRO_INTERATIONS) {
            // wait 0.7sec if calibration failed
            rt_thread_delay(7); // 700 ms

            read_gyro(&gyro[0], &gyro[1], &gyro[2]);
            for (i=0; i<3; i++)
            {
                fp_gyroOffset[i] = 0;
                prevGyro[i]=gyro[i];
            }
        }
        read_gyro(&gyro[0], &gyro[1], &gyro[2]);

        for (i=0; i<3; i++) {
            if(abs(prevGyro[i] - gyro[i]) > TOL) {
                tiltDetected++;
                break;
            }
        }
        for (i=0; i<3; i++) {
            fp_gyroOffset[i] += (float)gyro[i] / -GYRO_INTERATIONS;
            prevGyro[i]=gyro[i];
        }

        calibGCounter--;
        if(tiltDetected>=1) {
            rt_kprintf("[cal] Motion detected during Gyro calibration. Starting over.\n");
            calibGCounter=GYRO_INTERATIONS;
            tiltDetected=0;
        }
        if(rt_tick_get()>timeout)
        {

            rt_kprintf("[cal] Timeout Gyro calibration ERROR!\n");
            return;
        }
    }
    // put result into integer
    gyro_offset(fp_gyroOffset[0],fp_gyroOffset[1],fp_gyroOffset[2]);

    rt_kprintf("[cal] Gyro Calibration : %d, %d, %d\n",(int)fp_gyroOffset[0], (int)fp_gyroOffset[1], (int)fp_gyroOffset[2]);
}

/*
                  READ ACCELEROMETER
 * call this function to read accelerometer data. 
 */
void MPU6500::read_acc(int16_t *x,int16_t *y,int16_t *z)
{
    struct rt_spi_message msg;
    uint8_t send_buf[1];
    send_buf[0] = MPUREG_ACCEL_XOUT_H | READ_FLAG;
    /* initial message */
    msg.send_buf   = send_buf;
    msg.recv_buf   = RT_NULL;
    msg.length     = 1;
    msg.cs_take    = 1;
    msg.cs_release = 0;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(this->mpu6500_device, &msg);
    
    msg.send_buf   = RT_NULL;
    msg.recv_buf   = this->mpu_recv_buf;
    msg.length     = 6;
    msg.cs_take    = 0;
    msg.cs_release = 1;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(this->mpu6500_device, &msg);
    
    *x = mpu_recv_buf[1] | (short)(mpu_recv_buf[0] << 8);
    *y = mpu_recv_buf[3] | (short)(mpu_recv_buf[2] << 8);
    *z = mpu_recv_buf[5] | (short)(mpu_recv_buf[4] << 8);
}
/*
                   READ GYROSCOPE
 * call this function to read gyroscope data. 
 */
void MPU6500::read_gyro(int16_t *x,int16_t *y,int16_t *z)
{
    struct rt_spi_message msg;
    uint8_t send_buf[1];
    send_buf[0] = MPUREG_GYRO_XOUT_H | READ_FLAG;
    /* initial message */
    msg.send_buf   = send_buf;
    msg.recv_buf   = RT_NULL;
    msg.length     = 1;
    msg.cs_take    = 1;
    msg.cs_release = 0;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(this->mpu6500_device, &msg);
    
    msg.send_buf   = RT_NULL;
    msg.recv_buf   = this->mpu_recv_buf;
    msg.length     = 6;
    msg.cs_take    = 0;
    msg.cs_release = 1;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(this->mpu6500_device, &msg);
    
    *x = mpu_recv_buf[1] | (short)(mpu_recv_buf[0] << 8);
    *y = mpu_recv_buf[3] | (short)(mpu_recv_buf[2] << 8);
    *z = mpu_recv_buf[5] | (short)(mpu_recv_buf[4] << 8);
}

void MPU6500::set_rotation(enum Rotation rotation)
{
    imu_rotation = rotation;
}

// MPU6500连接状态测试
rt_bool_t MPU6500::testConnect()
{
    rt_uint8_t send_buffer[1];
	rt_uint8_t recv_buffer[1];
    // 确认设备是否挂载总线
    RT_ASSERT(this->mpu6500_device != RT_NULL);
    send_buffer[0] = MPUREG_WHOAMI | READ_FLAG;
    rt_spi_send_then_recv(this->mpu6500_device, send_buffer, 1, recv_buffer, 1);
    if(recv_buffer[0] == 0x98)
    {
        return RT_TRUE;
    }
    return RT_FALSE;
}

