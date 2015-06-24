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
 * nrf24l01.c: nRF24L01(-p) PRX mode low level driver
 */

/* TODO:
 *  - Separate the SPI and GPIO driver from here.
 *  - Handle PTX mode
 */
#define DEBUG_MODULE "ESP"

#include "esp8266.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ST includes */
#include "stm32f4xx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

//#include "debug.h"
#include "nvicconf.h"

#define ESP8266_UART_TYPE             USART2
#define ESP8266_UART_PERIF            RCC_APB1Periph_USART2
#define ESP8266_ENABLE_UART_RCC       RCC_APB1PeriphClockCmd
#define ESP8266_UART_IRQ              USART2_IRQn

#define ESP8266_UART_GPIO_PERIF       RCC_AHB1Periph_GPIOA
#define ESP8266_UART_GPIO_PORT        GPIOA
#define ESP8266_UART_GPIO_TX_PIN      GPIO_Pin_2
#define ESP8266_UART_GPIO_RX_PIN      GPIO_Pin_3
#define ESP8266_UART_GPIO_AF_TX_PIN   GPIO_PinSource2
#define ESP8266_UART_GPIO_AF_RX_PIN   GPIO_PinSource3
#define ESP8266_UART_GPIO_AF_TX       GPIO_AF_USART1
#define ESP8266_UART_GPIO_AF_RX       GPIO_AF_USART1


#define UART_DATA_TIMEOUT_MS			1000
#define UART_DATA_TIMEOUT_TICKS		(UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)

static bool isInit = false;

static xSemaphoreHandle waitUntilSendDone = NULL;
static xQueueHandle uartDataDelivery;

static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;

static char recvStr[100] = {0};

void esp8266Init(void)
{
	if(isInit)
		return;
	
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(ESP8266_UART_GPIO_PERIF, ENABLE);
  ESP8266_ENABLE_UART_RCC(ESP8266_UART_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = ESP8266_UART_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ESP8266_UART_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = ESP8266_UART_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(ESP8266_UART_GPIO_PORT, &GPIO_InitStructure);

  /* Map uart to alternate functions */
  GPIO_PinAFConfig(ESP8266_UART_GPIO_PORT, ESP8266_UART_GPIO_AF_TX_PIN, ESP8266_UART_GPIO_AF_TX);
  GPIO_PinAFConfig(ESP8266_UART_GPIO_PORT, ESP8266_UART_GPIO_AF_RX_PIN, ESP8266_UART_GPIO_AF_RX);

  USART_InitStructure.USART_BaudRate            = 115200;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
  USART_Init(ESP8266_UART_TYPE, &USART_InitStructure);

  // TODO: Enable
  // Configure Tx buffer empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = ESP8266_UART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ESP8266_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	vSemaphoreCreateBinary(waitUntilSendDone);
	uartDataDelivery = xQueueCreate(40, sizeof(uint8_t));

  USART_ITConfig(ESP8266_UART_TYPE, USART_IT_RXNE, ENABLE);

  //Enable UART
  USART_Cmd(ESP8266_UART_TYPE, ENABLE);
  
  isInit = true;
}

bool esp8266Test(void)
{
  return isInit;
}

bool esp8266UartGetDataWithTimout(uint8_t *c)
{
  if (xQueueReceive(uartDataDelivery, c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
  {
    return true;
  }
  return false;
}

void esp8266GetData(uint8_t *buf)
{
	while(esp8266UartGetDataWithTimout(buf++))
	{};
}

void esp8266UartSendData(uint8_t* data, uint32_t size)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
#ifdef UART_SPINLOOP_FLOWCTRL
    while(GPIO_ReadInputDataBit(UART_TXEN_PORT, UART_TXEN_PIN) == Bit_SET);
#endif
		while (!(ESP8266_UART_TYPE->SR & USART_FLAG_TXE));
		ESP8266_UART_TYPE->DR = (data[i] & 0x00FF);
  }
}

void esp8266UartSendDataBlock(uint8_t* data, uint32_t size)
{
  outDataIsr = data;
  dataSizeIsr = size;
  dataIndexIsr = 1;
	
  esp8266UartSendData(&data[0], 1);
  USART_ITConfig(ESP8266_UART_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
}

char esp8266UartPutChar(char ch)
{
    esp8266UartSendData((uint8_t *)&ch, 1);
    
    return ch;
}
unsigned char esp8266UartPutString(const char *s)
{
	unsigned char len = 0;
	
	while(*s) {
		esp8266UartPutChar(*s++);
		len++;
	}
	
	return len;
}

void esp8266Isr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint8_t rxDataInterrupt;

  if (USART_GetITStatus(ESP8266_UART_TYPE, USART_IT_TXE))
  {
    if (outDataIsr && (dataIndexIsr < dataSizeIsr))
    {
      USART_SendData(ESP8266_UART_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
      dataIndexIsr++;
    }
    else
    {
      USART_ITConfig(ESP8266_UART_TYPE, USART_IT_TXE, DISABLE);
      xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  USART_ClearITPendingBit(ESP8266_UART_TYPE, USART_IT_TXE);
  if (USART_GetITStatus(ESP8266_UART_TYPE, USART_IT_RXNE))
  {
    rxDataInterrupt = USART_ReceiveData(ESP8266_UART_TYPE) & 0x00FF;
    xQueueSendFromISR(uartDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
}

bool esp8266Cmd(char *cmd, char *reply1, char *reply2)
{
	unsigned char buf[80];
	
	esp8266UartPutString(cmd);
	esp8266UartPutString("\r\n");
	
	memset(buf, 0, 80);
	esp8266GetData(buf);
	
	if(reply1 && reply2)
		return (strstr((const char *)buf, reply1) || \
						strstr((const char *)buf, reply2));
	else if(reply1)
		return strstr((const char *)buf, reply1);
	else
		return strstr((const char *)buf, reply2);
}

bool esp8266Reset(void)
{
	return esp8266Cmd("AT+RST", "OK", "ready");
	
}

bool esp8266ModeChoose(esp8266Mode mode)
{
	switch(mode) {
		case STA:
			return esp8266Cmd("AT+CWMODE=1", "OK", "no change");

		case AP:
			return esp8266Cmd("AT+CWMODE=2", "OK", "no change");
			
		case STA_AP:
			return esp8266Cmd("AT+CWMODE=3", "OK", "no change");
			
		default:
			return esp8266Cmd("AT+CWMODE=1", "OK", "no change");
	}
}

bool esp8266JoinAP(char *ssid, char *password)
{
	char cmd[40];
	
	sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
	
	return esp8266Cmd(cmd, "OK", NULL);
}

bool esp8266BuildAP(char *ssid, char *password)
{
	char cmd[40];
	
	sprintf(cmd, "AT+CWSAP=\"%s\",\"%s\"", ssid, password);
	
	return esp8266Cmd(cmd, "OK", NULL);
}

bool esp8266EnableMultiId(bool multiLink)
{
	char cmd[20];
	
	sprintf(cmd, "AT+CIPMUX=%d", (multiLink ? 1 : 0));
	
	return esp8266Cmd(cmd, "OK", NULL);
}

bool esp8233LinkServer(char *protocol, char *ip, unsigned int port, unsigned char id)
{
	char buf[40] = {0}, cmd[60];

	if(strcmp(protocol, "TCP") && strcmp(protocol, "UDP"))
		return false;
	
	sprintf(buf, "\"%s\",\"%s\",%d", protocol, ip, port);

  if(id < 5 )
    sprintf(cmd, "AT+CIPSTART=%d,%s", id, buf);
  else
	  sprintf(cmd, "AT+CIPSTART=%s", buf);

	return esp8266Cmd(cmd, "OK", "CONNECT");
}

bool esp8266StartOrShutServer(bool mode, unsigned int port, unsigned int timeOver)
{
	char cmd1[40], cmd2[40];

	if(mode) {
		sprintf(cmd1, "AT+CIPSERVER=%d,%d", 1, port);
		sprintf(cmd2, "AT+CIPSTO=%d", timeOver );

		return (esp8266Cmd(cmd1, "OK", NULL) &&
						esp8266Cmd(cmd2, "OK", NULL));
	}	else {
		sprintf(cmd1, "AT+CIPSERVER=%d,%s", 0, port);

		return esp8266Cmd(cmd1, "OK", NULL);
	}
}
unsigned char esp8233CIPStatus(void)
{
	char buf[80];
	char *pStr = buf;
	char id;
	
	esp8266UartPutString("AT+CIPSTATUS\r\n");
	
	memset(buf, 0, 80);
	esp8266GetData((unsigned char *)buf);
	
	pStr = strstr(buf, "CIPSTATUS");
	id = *(pStr+10);
	
	return atoi(&id);
}

bool esp8266UnvarnishSend(void)
{
	return (esp8266Cmd("AT+CIPMODE=1", "OK", NULL) &&
					esp8266Cmd("AT+CIPSEND", "\r\n", ">"));
}

bool esp8266SendData(unsigned char enableUnvarnishTx, unsigned char id,
					unsigned char *sendData, unsigned char length)
{
	char cmd[20];
	bool ret = false;
		
	if(enableUnvarnishTx)
		esp8266UartSendData(sendData, length);
	
	else {
		if(id < 5)
			sprintf(cmd, "AT+CIPSEND=%d,%d", id, length);
		else
			sprintf(cmd, "AT+CIPSEND=%d", length);
		
		if(esp8266Cmd(cmd, "> ", NULL))
			ret = esp8266Cmd((char *)sendData, "SEND OK", NULL);
  }
	
	return ret;
}

bool esp8266ReceiveData(unsigned char enableUnvarnishTx,
				unsigned char *recvData, unsigned char *length)
{
	char *pRecvStr = recvStr;
	char temp[20];
	unsigned char num = 0;
	
	memset(recvStr, 0, sizeof(recvStr));
	
	while(esp8266UartGetDataWithTimout((unsigned char *)pRecvStr)) {
		pRecvStr++;
		num++;
	}

	if(num == 0)
		return false;
	
	if(enableUnvarnishTx)
	{
		if(strstr(recvStr, ">" )) {
			recvData = (unsigned char *)recvStr + 1;
			*length = num - 1;
		}
	} else {
		if(strstr(recvStr, "+IPD")) {
			pRecvStr = strchr(recvStr, ':');
			recvData = (unsigned char *)pRecvStr + 1;
			
			strncpy(temp, recvStr, pRecvStr - recvStr);
			pRecvStr = strrchr(temp, ',');
			*length = (unsigned char)atoi(pRecvStr);
		}
	}

	return true;
}
