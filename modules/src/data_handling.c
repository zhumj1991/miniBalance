#include "string.h"

#include "data_handling.h"

#include "uart_syslink.h"
#include "nrf24l01.h"

#include "imu.h"
#include "sensfusion6.h"
#include "motors.h"
#include "commander_balance.h"
#include "controller_balance.h"

extern Axis3f gyro; // Gyro axis data in deg/s
extern Axis3f acc;  // Accelerometer axis data in mG

extern float eulerRollActual;
extern float eulerPitchActual;
extern float eulerYawActual;


static bool checkXor(uint8_t *buf, uint8_t len, uint8_t *check_sum)
{
	uint8_t *data_temp = buf;
	uint8_t check_temp = 0;
	
	/* Generate the checksum */
	while(len) {
		check_temp ^= data_temp[len - 1];
		len--;
	}

	return (check_temp == *check_sum);
}

static void generateXor(uint8_t *buf, uint8_t len, uint8_t *check_sum)
{
	uint8_t *data_temp = buf;
	uint8_t check_temp = 0;
	
	/* Generate the checksum */
	while(len) {
		check_temp ^= data_temp[len - 1];
		len--;
	}
	
	*check_sum = check_temp;
}

void accBack(TransPacket *dataPack)
{
	memcpy(&dataPack->dataBuf[0], (const void *)&acc, sizeof(Axis3f));
	dataPack->len = sizeof(Axis3f);
}

void gyroBack(TransPacket *dataPack)
{
	memcpy(&dataPack->dataBuf[0], (const void *)&gyro, sizeof(Axis3f));
	dataPack->len = sizeof(Axis3f);
}

void magBack(TransPacket *dataPack)
{
	
}

void eulerBack(TransPacket *dataPack)
{
	memcpy(&dataPack->dataBuf[0], (const void *)&eulerRollActual, sizeof(float));
	memcpy(&dataPack->dataBuf[sizeof(float) - 1], (const void *)&eulerPitchActual, sizeof(float));
	memcpy(&dataPack->dataBuf[2*sizeof(float) - 1], (const void *)&eulerYawActual, sizeof(float));
	dataPack->len = 3 * sizeof(float);
}

void motorBack(TransPacket *dataPack)
{

}

bool dataHandler(uint8_t *dataRecv, uint8_t *dataSend, uint8_t *dataSendLen)
{
	TransPacket *dataPack = (TransPacket *)dataRecv;
	TransPacket *dataHandle = (TransPacket *)dataSend;
	
	if(dataPack->header != PC_HEADER)
		return false;

	if(checkXor(&dataPack->dataBuf[0], dataPack->len, &dataPack->dataBuf[dataPack->len]))
		return false;
	
	dataHandle->header = dataPack->header;
	
	switch(dataPack->cmd)
	{
		case 0x01:
			dataHandle->len = 0;
		case 0x02:
			commanderGetPacket(dataPack);
			break;
		case 0x03:
			velocityGetPID(dataPack);
			break;
		case 0x04:
			turnGetPID(dataPack);
			break;
		case 0x10:
			accBack(dataHandle);
			break;
		case 0x11:
			gyroBack(dataHandle);
			break;
		case 0x12:
			magBack(dataHandle);
			break;
		case 0x13:
			eulerBack(dataHandle);
			break;
		case 0x14:
			motorBack(dataHandle);
			break;
		case 0x15:
//			velocityPIDBack(dataHandle);
			break;
		case 0x16:
			turnPIDBack(dataHandle);
			break;
	}
	
	generateXor(&dataHandle->dataBuf[0], dataHandle->len, &dataHandle->dataBuf[dataHandle->len]);
	
	*dataSendLen = dataHandle->len + 5;

	return true;
}
