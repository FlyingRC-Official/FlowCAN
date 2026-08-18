#include "vl53l1_platform.h"
#include "config.h"
#include "platform.h"

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t reg, uint8_t *data, uint32_t count)
{
    if (count>252U) return -1;
    uint8_t buf[254]; buf[0]=(uint8_t)(reg>>8U); buf[1]=(uint8_t)reg;
    for (uint32_t i=0;i<count;i++) buf[i+2U]=data[i];
    return platform_i2c_write((uint8_t)dev,buf,count+2U,I2C_TIMEOUT_US)?0:-1;
}
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t reg, uint8_t *data, uint32_t count)
{
    uint8_t address[2]={(uint8_t)(reg>>8U),(uint8_t)reg};
    return platform_i2c_write((uint8_t)dev,address,2U,I2C_TIMEOUT_US) && platform_i2c_read((uint8_t)dev,data,count,I2C_TIMEOUT_US)?0:-1;
}
int8_t VL53L1_WrByte(uint16_t d,uint16_t r,uint8_t v){return VL53L1_WriteMulti(d,r,&v,1U);}
int8_t VL53L1_WrWord(uint16_t d,uint16_t r,uint16_t v){uint8_t b[2]={(uint8_t)(v>>8U),(uint8_t)v};return VL53L1_WriteMulti(d,r,b,2U);}
int8_t VL53L1_WrDWord(uint16_t d,uint16_t r,uint32_t v){uint8_t b[4]={(uint8_t)(v>>24U),(uint8_t)(v>>16U),(uint8_t)(v>>8U),(uint8_t)v};return VL53L1_WriteMulti(d,r,b,4U);}
int8_t VL53L1_RdByte(uint16_t d,uint16_t r,uint8_t *v){return VL53L1_ReadMulti(d,r,v,1U);}
int8_t VL53L1_RdWord(uint16_t d,uint16_t r,uint16_t *v){uint8_t b[2];int8_t e=VL53L1_ReadMulti(d,r,b,2U);*v=(uint16_t)((uint16_t)b[0]<<8U|b[1]);return e;}
int8_t VL53L1_RdDWord(uint16_t d,uint16_t r,uint32_t *v){uint8_t b[4];int8_t e=VL53L1_ReadMulti(d,r,b,4U);*v=(uint32_t)b[0]<<24U|(uint32_t)b[1]<<16U|(uint32_t)b[2]<<8U|b[3];return e;}
int8_t VL53L1_WaitMs(uint16_t dev,int32_t ms){(void)dev;if(ms<0)return -1;platform_delay_us((uint32_t)ms*1000U);return 0;}
