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

static unsigned char wifiSendData[64] = {0};
static unsigned char wifiRecvData[64] = {0};

static unsigned char recvLen;
static unsigned char sendLen;

static void wifiLinkInit()
{
#if 1
	while(!esp8266ModeChoose(AP));

	while(!esp8266Reset());

	while(!esp8266BuildAP("miniBalance","123456789"));
#else
	while(!esp8266ModeChoose(STA));

	while(!esp8266Reset());

	while(esp8266JoinAP("cubesat","cubesatb215215"));
#endif
	
}

static void wifilinkTask(void * arg)
{
//	wifiLinkInit();
	
//	unsigned char id;
//	
//	esp8266EnableMultiId(ENABLE);
////	esp8266StartOrShutServer(1, 8080, 3000);
////	id = esp8233CIPStatus();
//	
//	while(!(esp8233LinkServer("TCP", "192.168.1.110", 8080, 0) ||
//					esp8233LinkServer("TCP", "192.168.1.110", 8080, 1) ||
//					esp8233LinkServer("TCP", "192.168.1.110", 8080, 2) ||
//					esp8233LinkServer("TCP", "192.168.1.110", 8080, 3) ||
//					esp8233LinkServer("TCP", "192.168.1.110", 8080, 4)))
//	{
//		vTaskDelay(200);
//	}


	/* multiLink */
	esp8266EnableMultiId(ENABLE);
	
	/* creat server */
	while(!esp8266StartOrShutServer(ENABLE, 8080, 2000));
	
	ledseqRun(LED_GREEN, seq_linkup);
	
	while(1) {
		memset(wifiRecvData, 0, sizeof(wifiRecvData));
		esp8266GetData(wifiRecvData);
		if(strstr((const char *)wifiRecvData, "CONNECT"))
			break;
		
		vTaskDelay(200);
	}
	
	memset(wifiRecvData, 0,  sizeof(wifiRecvData));
	
	while(1)
	{
		if(esp8266ReceiveData(0, wifiRecvData, &recvLen)) {
			// handle the cmd
			if(dataHandler(wifiRecvData, wifiSendData, &sendLen))
			//ack
				esp8266SendData(0, 0, wifiSendData, sendLen);
		}
		vTaskDelay(500);		
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
              configMINIMAL_STACK_SIZE, NULL, /*priority*/4, NULL);

  isInit = true;
}

bool wifiTest()
{
  return esp8266Test();
}
