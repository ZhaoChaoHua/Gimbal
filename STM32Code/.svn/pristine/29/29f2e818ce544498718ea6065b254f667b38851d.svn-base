#include "hmc5983.h"


void HMC5983::hmc5983_init(void)
{ 
    this->hmc_device = (struct rt_spi_device *)rt_device_find(hmc_device_name);
    if(this->hmc_device == RT_NULL)
    {
        // 发送LED状态指示改变
        rt_kprintf("[err]HMC5983 device %s not found!\n", hmc_device_name);
        return;
    }
    // 连接测试
    testConnect();
    ConfigRegA();
    ConfigRegB();
    ConfigMode();
}

/* HMC5983_CONFIG_A
 * bits[4:2] 111 (220Hz)
 *           110 75
 *           101 30
 *           100 15
 *           011 7.5
 *           010 3
 *           001 1.5
 *           000 0.75
 */
void HMC5983::ConfigRegA()
{
    rt_uint8_t send_buffer[2];
    uint8_t recv;
    uint8_t temp;
    
    // 确认设备是否挂载总线
    RT_ASSERT(this->hmc_device != RT_NULL);
    
    send_buffer[0] = HMC5983_CONFIG_A;
    send_buffer[1] = 0x9C; // Enable tem, sample 1, rate 220Hz,
    rt_spi_send_then_send(this->hmc_device, &send_buffer[0], 1, &send_buffer[1], 1);
    rt_thread_delay(1); // 10ms
    send_buffer[0] = HMC5983_CONFIG_A | HMC_RD_FLAG;
    rt_spi_send_then_recv(this->hmc_device,&send_buffer[0],1,&recv,1);
    temp = (recv >> 7)&0x1;  // 取出最高位
    if(temp == 0)
    {
        rt_kprintf("[init]HMC5983 Disable temperature sensor\n");
    }
    else
    {
        rt_kprintf("[init]HMC5983 Enable temperature sensor\n");
    }
    temp = (recv >> 5)&0x3; // 取出5,6位
    switch(temp)
    {
        case 0:
            rt_kprintf("[init]HMC5983 Samples averaged 1 per measurement output\n");
            break;
        case 1:
            rt_kprintf("[init]HMC5983 Samples averaged 2 per measurement output\n");
            break;
        case 2:
            rt_kprintf("[init]HMC5983 Samples averaged 4 per measurement output\n");
            break;
        case 3:
            rt_kprintf("[init]HMC5983 Samples averaged 8 per measurement output\n");
            break;
    }
    temp = (recv >> 2)&0x7; // 取出2,3,4位
    switch(temp)
    {
        case 0:
            rt_kprintf("[init]HMC5983 Data output rate: 0.75 Hz\n");
            break;
        case 1:
            rt_kprintf("[init]HMC5983 Data output rate: 1.5 Hz\n");
            break;
        case 2:
            rt_kprintf("[init]HMC5983 Data output rate: 3 Hz\n");
            break;
        case 3:
            rt_kprintf("[init]HMC5983 Data output rate: 7.5 Hz\n");
            break;
        case 4:
            rt_kprintf("[init]HMC5983 Data output rate: 15 Hz\n");
            break;
        case 5:
            rt_kprintf("[init]HMC5983 Data output rate: 30 Hz\n");
            break;
        case 6:
            rt_kprintf("[init]HMC5983 Data output rate: 75 Hz\n");
            break;
        case 7:
            rt_kprintf("[init]HMC5983 Data output rate: 220 Hz\n");
            break;
    }
}

void HMC5983::ConfigRegB()
{
    rt_uint8_t send_buffer[2];
    uint8_t recv;
    uint8_t temp;
    
    // 确认设备是否挂载总线
    RT_ASSERT(this->hmc_device != RT_NULL);
    
    send_buffer[0] = HMC5983_CONFIG_B;
    send_buffer[1] = 0x60;
    rt_spi_send_then_send(this->hmc_device, &send_buffer[0], 1, &send_buffer[1], 1);
    rt_thread_delay(1); // 10ms
    send_buffer[0] = HMC5983_CONFIG_B | HMC_RD_FLAG;
    rt_spi_send_then_recv(this->hmc_device,&send_buffer[0],1,&recv,1);
    temp = (recv >> 5)&0x3;  // 取出最高位
    switch(temp)
    {
        case 0:
            rt_kprintf("[init]HMC5983 Recommended Sensor Field Range:+-0.88 Ga\n");
            hmc_scale = 1370;
            break;
        case 1:
            rt_kprintf("[init]HMC5983 Recommended Sensor Field Range:+-1.3 Ga\n");
            hmc_scale = 1090;
            break;
        case 2:
            rt_kprintf("[init]HMC5983 Recommended Sensor Field Range:+-1.9 Ga\n");
            hmc_scale = 820;
            break;
        case 3:
            rt_kprintf("[init]HMC5983 Recommended Sensor Field Range:+-2.5 Ga\n");
            hmc_scale = 660;
            break;
        case 4:
            rt_kprintf("[init]HMC5983 Recommended Sensor Field Range:+-4.0 Ga\n");
            hmc_scale = 440;
            break;
        case 5:
            rt_kprintf("[init]HMC5983 Recommended Sensor Field Range:+-4.7 Ga\n");
            hmc_scale = 390;
            break;
        case 6:
            rt_kprintf("[init]HMC5983 Recommended Sensor Field Range:+-5.6 Ga\n");
            hmc_scale = 330;
            break;
        case 7:
            rt_kprintf("[init]HMC5983 Recommended Sensor Field Range:+-8.1 Ga\n");
            hmc_scale = 230;
            break;
    }
}

void HMC5983::ConfigMode()
{
    rt_uint8_t send_buffer[2];
    uint8_t recv;
    uint8_t temp;
    
    // 确认设备是否挂载总线
    RT_ASSERT(this->hmc_device != RT_NULL);
    
    send_buffer[0] = HMC5983_MODE;
    send_buffer[1] = 0x00;
    rt_spi_send_then_send(this->hmc_device, &send_buffer[0], 1, &send_buffer[1], 1);
    rt_thread_delay(1); // 10ms
    send_buffer[0] = HMC5983_MODE | HMC_RD_FLAG;
    rt_spi_send_then_recv(this->hmc_device,&send_buffer[0],1,&recv,1);
    temp = recv & 0x3;  // 取出最高位
    switch(temp)
    {
        case 0:
            rt_kprintf("[init]HMC5983 Continuous-Measurement Mode\n");
            break;
        case 1:
            rt_kprintf("[init]HMC5983 Single-Measurement Mode\n");
            break;
        case 2:
            rt_kprintf("[init]HMC5983 Idle Mode\n");
            break;
        case 3:
            rt_kprintf("[init]HMC5983 Idle Mode\n");
            break;
    }
}

void HMC5983::hmc5983_read_data(HMC5983_Struct *data)
{
    struct rt_spi_message msg;
    
    /* initial message */
    msg.send_buf   = this->hmc_send_buf;
    msg.recv_buf   = RT_NULL;
    msg.length     = 1;
    msg.cs_take    = 1;
    msg.cs_release = 0;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(this->hmc_device, &msg);
    
    msg.send_buf   = RT_NULL;
    msg.recv_buf   = this->hmc_recv_buf;
    msg.length     = 6;
    msg.cs_take    = 0;
    msg.cs_release = 1;
    msg.next       = RT_NULL;
    rt_spi_transfer_message(this->hmc_device, &msg);
    
    data->Geom_I16.x = (hmc_recv_buf[1]  | (short)(hmc_recv_buf[0] << 8));
    data->Geom_I16.z = (hmc_recv_buf[3]  | (short)(hmc_recv_buf[2] << 8));
    data->Geom_I16.y = (hmc_recv_buf[5]  | (short)(hmc_recv_buf[4] << 8));
}

rt_bool_t HMC5983::testConnect()
{
    rt_uint8_t send_buffer[1];
	rt_uint8_t recv_buffer[1];
    
    // 确认设备是否挂载总线
    RT_ASSERT(this->hmc_device != RT_NULL);
    
    send_buffer[0] = HMC5983_ID_A | HMC_RD_FLAG;
    rt_spi_send_then_recv(this->hmc_device, send_buffer, 1, recv_buffer, 1);
    rt_kprintf("[init]HMC5983 has been identified: 0x%02x",recv_buffer[0]);
	send_buffer[0] = HMC5983_ID_B | HMC_RD_FLAG;
    rt_spi_send_then_recv(this->hmc_device, send_buffer, 1, recv_buffer, 1);
    rt_kprintf(" 0x%02x",recv_buffer[0]);
	send_buffer[0] = HMC5983_ID_C | HMC_RD_FLAG;
    rt_spi_send_then_recv(this->hmc_device, send_buffer, 1, recv_buffer, 1);
    rt_kprintf(" 0x%02x\n",recv_buffer[0]);
    
    return RT_EOK;
}

