#include "ANO_TC_STM32_I2C.h"
// 地址先左移一位
void ANO_TC_I2C1_ReadFinish_Callback(uint8_t DevAddr,uint8_t RegAddr, uint8_t ReadNum, uint8_t *ReadTemp)
{
	if(DevAddr==0xD0 && RegAddr==0x3B)
    {
        
    }
}

void ANO_TC_I2C1_WriteFinish_Callback(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteNum, uint8_t * WriteTemp)
{
	
}

