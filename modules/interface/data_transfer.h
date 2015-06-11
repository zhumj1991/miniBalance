#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_

#include "stm32f4xx.h"

typedef struct int16_rcget{
	int16_t THROTTLE;
	int16_t ROLL;
	int16_t PITCH;
	int16_t YAW;
	int16_t AUX1;
	int16_t AUX2;
	int16_t AUX3;
	int16_t AUX4;
	int16_t AUX5;
	int16_t AUX6;
}RC_DATA;

typedef struct{
	int16_t X;
	int16_t Y;
	int16_t Z;
}INT16_XYZ;

void Data_Receive_Anl(u8 *data_buf,u8 num);
void Data_Exchange(void);

void Data_Send_Status(void);	
void Data_Send_Senser(void);	
void Data_Send_RCData(void);	
void Data_Send_GpsData(void);
void Data_Send_OFFSET(void);	
void Data_Send_PID1(void);
void Data_Send_PID2(void);
void Data_Send_PID3(void);
void Data_Send_MotoPWM(void);

void NRF_Send_Test(void);

#endif
