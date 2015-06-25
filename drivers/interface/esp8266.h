/*
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * nrf24l01.h: nRF24L01(-p) PRX mode low level driver
 */

#ifndef __ESP8266_H__
#define __ESP8266_H__

#include <stdbool.h>
#include "stm32f4xx.h"

typedef enum {
	STA = 0,
	AP,
	STA_AP,
} esp8266Mode;

void esp8266Init(void);

bool esp8266Test(void);

bool esp8266UartGetDataWithTimout(uint8_t *c);
void esp8266GetData(uint8_t *buf);

void esp8266UartSendData(uint8_t* data, uint32_t size);

void esp8266Isr(void);


bool esp8266Cmd(char *cmd, char *reply1, char *reply2);
bool esp8266Reset(void);
bool esp8266ModeChoose(esp8266Mode mode);
bool esp8266JoinAP(char *ssid, char *password);
bool esp8266BuildAP(char *ssid, char *password);
bool esp8266EnableMultiId(bool multiLink);
bool esp8233LinkServer(char *protocol, char *ip, unsigned int port, unsigned char id);
bool esp8266StartOrShutServer(bool mode, unsigned int port, unsigned int timeOver);
bool esp8233CIPStatus(void);
bool esp8266UnvarnishSend(void);
bool esp8266SendData(unsigned char enableUnvarnishTx, unsigned char id,
					unsigned char *sendData, unsigned char length);
bool esp8266ReceiveData(unsigned char enableUnvarnishTx,
				unsigned char *recvData, unsigned char *length);

#endif
