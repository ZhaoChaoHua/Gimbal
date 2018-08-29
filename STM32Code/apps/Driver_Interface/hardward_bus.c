#include "hardward_bus.h"
#include "gpio.h"
#include "eeprom.h"
#include "e2pfs_port.h"

/*** SPI1 BUS and device
SPI1_MOSI: PA7
SPI1_MISO: PA6
SPI1_SCK : PA5
SPI1_NSS : PA4
*/
void rt_hw_spi1_init(void)
{
	/* register SPI bus */
	static struct stm32_spi_bus stm32_spi;

	  Set_pinMode(PA5, AF_OUTPUT);
    Set_pinMode(PA6, AF_OUTPUT);
    Set_pinMode(PA7, AF_OUTPUT);

	/* register SPI1 */
	stm32_spi_register(SPI1, &stm32_spi, "spi1");

//	/* attach mpu65 */
//    static struct rt_spi_device rt_spi_device_10;
//    static struct stm32_spi_cs  stm32_spi_cs_10;

//    stm32_spi_cs_10.GPIOx = GPIOD;
//    stm32_spi_cs_10.GPIO_Pin = GPIO_Pin_4;  // HMC5983_CS PD4

//    Set_pinMode(PD4, OUTPUT);
//    GPIO_OUT(PD4, 1);

//    rt_spi_bus_attach_device(&rt_spi_device_10, "hmc", "spi1", (void*)&stm32_spi_cs_10);

//    struct rt_spi_configuration cfg;
//    cfg.data_width = 8;                        // 数据宽度8位
//    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB;     // SPI Compatible Modes 3

//    //SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M,
//    cfg.max_hz = 1000*6000; /* 1000kbit/s */
//    rt_spi_configure(&rt_spi_device_10, &cfg);

    static struct rt_spi_device rt_spi_device_20;
    static struct stm32_spi_cs  stm32_spi_cs_20;
    
    stm32_spi_cs_20.GPIOx = GPIOA;
    stm32_spi_cs_20.GPIO_Pin = GPIO_Pin_0;

    Set_pinMode(PA0, OUTPUT);
    GPIO_SetBits(GPIOA, GPIO_Pin_0);

    rt_spi_bus_attach_device(&rt_spi_device_20, "mpu65", "spi1", (void*)&stm32_spi_cs_20);
    
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;                        // 数据宽度8位
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB;     // SPI Compatible Modes 3
    
    //SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M,
    cfg.max_hz = 1000*5000; /* 8000kbit/s */
    rt_spi_configure(&rt_spi_device_20, &cfg);
		
		
		
			/* BMM150 */
    static struct rt_spi_device rt_spi_device_21;
    static struct stm32_spi_cs  stm32_spi_cs_21;

    stm32_spi_cs_21.GPIOx = GPIOA;
    stm32_spi_cs_21.GPIO_Pin = GPIO_Pin_1;  // HMC5983_CS PD4

    Set_pinMode(PA1, OUTPUT);
    GPIO_SetBits(GPIOA, GPIO_Pin_1);

    rt_spi_bus_attach_device(&rt_spi_device_21, "mpu65_1", "spi1", (void*)&stm32_spi_cs_21);
		
		struct rt_spi_configuration cfg1;
    cfg1.data_width = 8;                        // 数据宽度8位
    cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB;     // SPI Compatible Modes 3
    
    //SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M,
    cfg1.max_hz = 1000*5000; /* 8000kbit/s */
    rt_spi_configure(&rt_spi_device_21, &cfg1);
}

void rt_hw_spi2_init(void)
{
	/* register SPI bus */
	static struct stm32_spi_bus stm32_spi;
	
	Set_pinMode(PB13, AF_OUTPUT);
    Set_pinMode(PB14, AF_OUTPUT);
    Set_pinMode(PB15, AF_OUTPUT);

	/* register SPI1 */
	stm32_spi_register(SPI2, &stm32_spi, "spi2");

    static struct rt_spi_device rt_spi_device_20;
    static struct stm32_spi_cs  stm32_spi_cs_20;
    
    stm32_spi_cs_20.GPIOx = GPIOB;
    stm32_spi_cs_20.GPIO_Pin = GPIO_Pin_12;

    Set_pinMode(PB12, OUTPUT);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
    
//    Set_pinMode(PB8, OUTPUT);
//    GPIO_SetBits(GPIOB, GPIO_Pin_8);
//    
//    Set_pinMode(PB9, OUTPUT);
//    GPIO_SetBits(GPIOB, GPIO_Pin_9);

    rt_spi_bus_attach_device(&rt_spi_device_20, "mpu65", "spi2", (void*)&stm32_spi_cs_20);
    
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;                        // 数据宽度8位
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB;     // SPI Compatible Modes 3
    
    //SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M,
    cfg.max_hz = 1000*11000; /* 8000kbit/s */
    rt_spi_configure(&rt_spi_device_20, &cfg);
		
		
		
			/* BMM150 */
    static struct rt_spi_device rt_spi_device_21;
    static struct stm32_spi_cs  stm32_spi_cs_21;

    stm32_spi_cs_21.GPIOx = GPIOB;
    stm32_spi_cs_21.GPIO_Pin = GPIO_Pin_11;  // HMC5983_CS PD4

    Set_pinMode(PB11, OUTPUT);
    GPIO_SetBits(GPIOB, GPIO_Pin_11);

    rt_spi_bus_attach_device(&rt_spi_device_21, "mpu65_1", "spi2", (void*)&stm32_spi_cs_21);
		
		struct rt_spi_configuration cfg1;
    cfg1.data_width = 8;                        // 数据宽度8位
    cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB;     // SPI Compatible Modes 3
    
    //SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M,
    cfg1.max_hz = 1000*11000; /* 8000kbit/s */
    rt_spi_configure(&rt_spi_device_21, &cfg1);

//    struct rt_spi_configuration cfg_new;
//    cfg_new.data_width = 8;                        // 数据宽度8位
//    cfg_new.mode = RT_SPI_MODE_3 | RT_SPI_MSB;     // SPI Compatible Modes 3

//    //SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M,
//    cfg.max_hz = 1000*8000; /* 1000kbit/s */
//    rt_spi_configure(&rt_spi_device_21, &cfg_new);
}

void rt_hw_eeprom_init(void)
{   

    Set_pinMode(PB1, OUTPUT);
    GPIO_OUT(PB1, 0);
    
    sEE_DeInit();
    sEE_Init();
    parameter_init();
}

