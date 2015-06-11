#ifndef _OV7670_H_
#define _OV7670_H_

#include "i2cdev.h"

#define OV7670_ADDRESS	0x42
#define OV_REG_NUM  		116

void ov7670Init(I2C_TypeDef *i2cPort);
bool ov7670Test(void);

bool ov7670TestConnection(void);

uint16_t ov7670GetDeviceID(void);

void ov7670InitReg(void);
#endif
