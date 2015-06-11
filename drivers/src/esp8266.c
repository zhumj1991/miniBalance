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

bool esp8266GetDataWithTimout(uint8_t *c)
{
  if (xQueueReceive(uartDataDelivery, c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
  {
    return true;
  }
  return false;
}

void esp8266SendData(uint8_t* data, uint32_t size)
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

void esp8266SendDataBlock(uint8_t* data, uint32_t size)
{
  outDataIsr = data;
  dataSizeIsr = size;
  dataIndexIsr = 1;
	
  esp8266SendData(&data[0], 1);
  USART_ITConfig(ESP8266_UART_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
}

char esp8266PutChar(char ch)
{
    esp8266SendData((uint8_t *)&ch, 1);
    
    return ch;
}
unsigned char esp8266PutString(const char *s)
{
	unsigned char len = 0;
	
	while(*s) {
		esp8266PutChar(*s++);
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
