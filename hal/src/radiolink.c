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
#include <stdbool.h>
#include <errno.h>

#include "config.h"
#include "nrf24l01.h"
#include "ledseq.h"
#include "data_handling.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"




static bool isInit;

/* Synchronisation */
xSemaphoreHandle dataRdy;


static struct {
  bool enabled;
} state;

static void interruptCallback()
{
  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

  //To unlock RadioTask
  xSemaphoreGiveFromISR(dataRdy, &xHigherPriorityTaskWoken);

  if(xHigherPriorityTaskWoken)
    vPortYieldFromISR();
}


/* Radio task handles the CRTP packet transfers as well as the radio link
 * specific communications (eg. Scann and ID ports, communication error handling
 * and so much other cool things that I don't have time for it ...)
 */
static void radiolinkTask(void * arg)
{
  unsigned char dataLen;
	unsigned char status;
  static unsigned char pk[32];

  //Packets handling loop
  while(1)
  {
		
		
//    ledseqRun(LED_GREEN, seq_linkup);
//		
//    nrfSetEnable(true);
//    xSemaphoreTake(dataRdy, portMAX_DELAY);
//    
//		nrfSetEnable(false);
//    //Fetch all the data (Loop until the RX Fifo is NOT empty)
//    while(!(nrfRead1Reg(REG_FIFO_STATUS) & 0x01) )
//    {
//      dataLen = nrfRxLength(0);

////      if (dataLen<5 || dataLen>32)          //If a packet has a wrong size it is dropped
////        nrfFlushRx();
////      else                     //Else, it is processed
////      {
////        //Fetch the data
////        nrfReadRX((uint8_t *)pk, dataLen);
////      }
//    }

//		nrfReadRX((uint8_t *)pk, 32);
//		
//		status = nrfNop();
//		nrfWrite1Reg(REG_STATUS, status);
//		if(status & 0x40) 
//			nrfFlushRx();
#if 1
		nrfSetMode(TX_MODE);

		nrfSetEnable(false);
		memset((void *)pk, 0x5A, 5);
		nrfWriteTx((uint8_t *)pk, 5);
		nrfSetEnable(true);
		
		xSemaphoreTake(dataRdy, portMAX_DELAY);
		/* clear the interruptions flags */
		status = nrfNop();
    nrfWrite1Reg(REG_STATUS, status);
		
		if(status & 0x20) {
			nrfFlushTx();
			ledseqRun(LED_RED, seq_linkup);
		}

#else
		nrfSetMode(RX_MODE);
		
		xSemaphoreTake(dataRdy, portMAX_DELAY);
		status = nrfNop();
		nrfWrite1Reg(REG_STATUS, status);
		if(status & 0x40) {
			nrfReadRX((uint8_t *)pk, 32);
			nrfFlushRx();
			ledseqRun(LED_RED, seq_linkup);
		}
		
#endif	

    //Re-enable the radio 
//		vTaskDelay(1000);		
  }
}

static void radiolinkInitNRF24L01P(void)
{
  int i;
	
	nrfSetMode(RX_MODE);
	
  vTaskDelay(M2T(2)); //Wait for the chip to be ready
  // Enable the dynamic payload size and the ack payload for the pipe 0
//  nrfWrite1Reg(REG_FEATURE, 0x06);
//  nrfWrite1Reg(REG_DYNPD, 0x01);

  for(i=0;i<3;i++)
    nrfFlushRx();
  for(i=0;i<3;i++)
    nrfFlushTx();
}

/*
 * Public functions
 */

void radiolinkInit()
{
  if(isInit)
    return;

  nrfInit();

  nrfSetInterruptCallback(interruptCallback);

  vTaskSetApplicationTaskTag(0, (void*)TASK_RADIO_ID_NBR);

  /* Initialise the semaphores */
  vSemaphoreCreateBinary(dataRdy);

  radiolinkInitNRF24L01P();

    /* Launch the Radio link task */
  xTaskCreate(radiolinkTask, (const char * )"RadioLink",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  isInit = true;
}

bool radiolinkTest()
{
  return nrfTest();
}

void radiolinkReInit(void)
{
  radiolinkInitNRF24L01P();
}
