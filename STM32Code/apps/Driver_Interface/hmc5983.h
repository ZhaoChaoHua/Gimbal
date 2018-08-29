#ifndef __HMC5983_H__
#define __HMC5983_H__

#include <rtthread.h>
#include <stm32f4xx.h>
#include "stm32f40x_spi.h"
#include "spi.h"
#include "variables.h"

#define HMC5983_CONFIG_A      0x00
#define HMC5983_CONFIG_B      0x01
#define HMC5983_MODE          0x02

#define DATA_X_MSB            0x03    // 高8位
#define DATA_X_LSB            0x04
#define DATA_Z_MSB            0x05
#define DATA_Z_LSB            0x06
#define DATA_Y_MSB            0x07
#define DATA_Y_LSB            0x08

#define HMC5983_STATUS        0x09

#define HMC5983_ID_A          0x0A
#define HMC5983_ID_B          0x0B
#define HMC5983_ID_C          0x0C

#define HMC_RD_FLAG           0x80
#define HMC_RD_CONTINUE       0xC0

#ifdef __cplusplus

class HMC5983
{
private:
    struct rt_spi_device *hmc_device;
    uint8_t               hmc_recv_buf[6];
    uint8_t               hmc_send_buf[1];
    const char           *hmc_device_name;

public:    
    HMC5983()
    {
        hmc_device_name = "hmc";
        hmc_send_buf[0]  = DATA_X_MSB | HMC_RD_CONTINUE;
    };
    // 量程刻度
    uint16_t hmc_scale;
    void hmc5983_init(void);
    rt_bool_t testConnect();
    void ConfigRegA();
    void ConfigRegB();
    void ConfigMode();
    void hmc5983_read_data(HMC5983_Struct *data);
    
};

#endif
#endif
