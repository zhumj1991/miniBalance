/*
 * Copyright (C) 2012 BitCraze AB
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
 * radiolink.c: nRF24L01 implementation of the CRTP link
 */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>

#include "config.h"
#include "esp8266.h"
#include "ledseq.h"
#include "data_handling.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static bool isInit;

static char wifiSendData[64] = {'\0'};
static char wifiRecvData[64] = {'\0'};

static void wifilinkInit(void)
{
	sprintf(wifiSendData, "AT+RST\n");
	esp8266SendData((uint8_t *)wifiSendData, strlen(wifiSendData));
	esp8266GetDataWithTimout((uint8_t *)wifiRecvData);
	
	sprintf(wifiSendData, "AT+CWMODE=2\n");
	esp8266SendData((uint8_t *)wifiSendData, strlen(wifiSendData));
	esp8266GetDataWithTimout((uint8_t *)wifiRecvData);
	
	sprintf(wifiSendData, "AT+RST\n");
	esp8266SendData((uint8_t *)wifiSendData, strlen(wifiSendData));
	esp8266GetDataWithTimout((uint8_t *)wifiRecvData);
	
	sprintf(wifiSendData, "AT+CWSAP=\"ESP8266\",\"0123456789\",11,0\n");
	esp8266SendData((uint8_t *)wifiSendData, strlen(wifiSendData));
	esp8266GetDataWithTimout((uint8_t *)wifiRecvData);
	
	sprintf(wifiSendData, "AAT+ CIPMUX=1\n");
	esp8266SendData((uint8_t *)wifiSendData, strlen(wifiSendData));
	esp8266GetDataWithTimout((uint8_t *)wifiRecvData);
	
	sprintf(wifiSendData, "AT+CIPSERVER=1,8080\n");
	esp8266SendData((uint8_t *)wifiSendData, strlen(wifiSendData));
	esp8266GetDataWithTimout((uint8_t *)wifiRecvData);
}

static void wifilinkTask(void * arg)
{
	wifilinkInit();
	
	while(1)
  {
		vTaskDelay(1000);
	}
}

void wifiInit()
{
  if(isInit)
    return;

  esp8266Init();

  vTaskSetApplicationTaskTag(0, (void *)TASK_RADIO_ID_NBR);

    /* Launch the Radio link task */
  xTaskCreate(wifilinkTask, (const char * )"WiFiLink",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  isInit = true;
}

bool wifiTest()
{
  return esp8266Test();
}
