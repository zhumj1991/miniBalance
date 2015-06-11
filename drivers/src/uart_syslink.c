/**
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
 * uart.c - uart CRTP link and raw access functions
 */
#include <string.h>

/*ST includes */
#include "stm32f4xx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

//#include "config.h"
#include "uart_syslink.h"
//#include "crtp.h"
//#include "cfassert.h"
#include "nvicconf.h"
//#include "config.h"
//#include "ledseq.h"


#define UART_DATA_TIMEOUT_MS			1000
#define UART_DATA_TIMEOUT_TICKS		(UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET						((uint32_t)0x00000001)

static bool isInit = false;

xSemaphoreHandle waitUntilSendDone = NULL;
static xQueueHandle uartDataDelivery;

static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;

void uartInit(void)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UART_GPIO_PERIF, ENABLE);
  ENABLE_UART_RCC(UART_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UART_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStructure);

  /* Map uart to alternate functions */
  GPIO_PinAFConfig(UART_GPIO_PORT, UART_GPIO_AF_TX_PIN, UART_GPIO_AF_TX);
  GPIO_PinAFConfig(UART_GPIO_PORT, UART_GPIO_AF_RX_PIN, UART_GPIO_AF_RX);

  USART_InitStructure.USART_BaudRate            = 115200;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
  USART_Init(UART_TYPE, &USART_InitStructure);

  // TODO: Enable
  // Configure Tx buffer empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  vSemaphoreCreateBinary(waitUntilSendDone);
  uartDataDelivery = xQueueCreate(40, sizeof(uint8_t));

  USART_ITConfig(UART_TYPE, USART_IT_RXNE, ENABLE);

  //Enable UART
  USART_Cmd(UART_TYPE, ENABLE);
  
  isInit = true;
}

bool uartTest(void)
{
  return isInit;
}

bool uartGetDataWithTimout(uint8_t *c)
{
  if (xQueueReceive(uartDataDelivery, c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
  {
    return true;
  }
  return false;
}

void uartSendData(uint8_t* data, uint32_t size)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
#ifdef UART_SPINLOOP_FLOWCTRL
    while(GPIO_ReadInputDataBit(UART_TXEN_PORT, UART_TXEN_PIN) == Bit_SET);
#endif
    while (!(UART_TYPE->SR & USART_FLAG_TXE));
    UART_TYPE->DR = (data[i] & 0x00FF);
  }
}

void uartSendDataIsrBlocking(uint8_t* data, uint32_t size)
{
  outDataIsr = data;
  dataSizeIsr = size;
  dataIndexIsr = 1;
  uartSendData(&data[0], 1);
  USART_ITConfig(UART_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
}

int uartPutchar(int ch)
{
    uartSendData((uint8_t *)&ch, 1);
    
    return (unsigned char)ch;
}


void uartIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint8_t rxDataInterrupt;

  if (USART_GetITStatus(UART_TYPE, USART_IT_TXE))
  {
    if (outDataIsr && (dataIndexIsr < dataSizeIsr))
    {
      USART_SendData(UART_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
      dataIndexIsr++;
    }
    else
    {
      USART_ITConfig(UART_TYPE, USART_IT_TXE, DISABLE);
      xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  USART_ClearITPendingBit(UART_TYPE, USART_IT_TXE);
  if (USART_GetITStatus(UART_TYPE, USART_IT_RXNE))
  {
    rxDataInterrupt = USART_ReceiveData(UART_TYPE) & 0x00FF;
    xQueueSendFromISR(uartDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
}


