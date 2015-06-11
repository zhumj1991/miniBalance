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
#define DEBUG_MODULE "NRF"

#include "nrf24l01.h"

#include <stdbool.h>
#include <string.h>

/* ST includes */
#include "stm32f4xx.h"

//#include "debug.h"
#include "nrf24l01Reg.h"
#include "nvicconf.h"


/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
#define RADIO_GPIO_CS             GPIO_Pin_7
#define RADIO_GPIO_CS_PORT        GPIOF
#define RADIO_GPIO_CS_PERIF       RCC_AHB1Periph_GPIOF

#define RADIO_GPIO_CE             GPIO_Pin_4
#define RADIO_GPIO_CE_PORT        GPIOA
#define RADIO_GPIO_CE_PERIF       RCC_AHB1Periph_GPIOA

#define RADIO_GPIO_IRQ            GPIO_Pin_7
#define RADIO_GPIO_IRQ_PORT       GPIOH
#define RADIO_GPIO_IRQ_PERIF      RCC_AHB1Periph_GPIOH
#define RADIO_GPIO_IRQ_SRC_PORT   EXTI_PortSourceGPIOH
#define RADIO_GPIO_IRQ_SRC        EXTI_PinSource7
#define RADIO_GPIO_IRQ_LINE       EXTI_Line7

#define RADIO_SPI                 SPI3
#define RADIO_SPI_CLK             RCC_APB1Periph_SPI3
#define RADIO_GPIO_SPI_PORT       GPIOB
#define RADIO_GPIO_SPI_CLK        RCC_AHB1Periph_GPIOB
#define RADIO_GPIO_SPI_SCK        GPIO_Pin_3
#define RADIO_GPIO_SPI_MISO       GPIO_Pin_4
#define RADIO_GPIO_SPI_MOSI       GPIO_Pin_5

#define	RADIO_GPIO_AF_SPI							GPIO_AF_SPI3
#define RADIO_GPIO_AF_SPI_SCK_PIN     GPIO_PinSource3
#define RADIO_GPIO_AF_SPI_MISO_PIN    GPIO_PinSource4
#define RADIO_GPIO_AF_SPI_MOSI_PIN    GPIO_PinSource5


#define DUMMY_BYTE    0xA5

/* nRF24L SPI commands */
#define CMD_R_REG              0x00
#define CMD_W_REG              0x20
#define CMD_R_RX_PAYLOAD       0x61
#define CMD_W_TX_PAYLOAD       0xA0
#define CMD_FLUSH_TX           0xE1
#define CMD_FLUSH_RX           0xE2
#define CMD_REUSE_TX_PL        0xE3
#define CMD_ACTIVATE           0x50
#define CMD_RX_PL_WID          0x60
#define CMD_W_ACK_PAYLOAD(P)  (0xA8|(P&0x0F))
#define CMD_W_PAYLOAD_NO_ACK   0xD0
#define CMD_NOP                0xFF


#define ADDR_WIDTH			5		// 5 uint8s TX/RX address width
#define TX_PLOAD_WIDTH	32	// 20 uint8s TX payload
#define RX_PLOAD_WIDTH	32	// 20 uint8s TX payload

const uint8_t tx_addr[ADDR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};
const uint8_t rx_addr[ADDR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};


/* Usefull macro */
#define RADIO_EN_CS()		GPIO_ResetBits(RADIO_GPIO_CS_PORT, RADIO_GPIO_CS)
#define RADIO_DIS_CS()	GPIO_SetBits(RADIO_GPIO_CS_PORT, RADIO_GPIO_CS)
#define RADIO_DIS_CE()	GPIO_ResetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE)
#define RADIO_EN_CE()		GPIO_SetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE)
#define ACTIVATE_DATA   0x73

/* Private variables */
static bool isInit;
static void (*interruptCb)(void) = NULL;

/***********************
 * SPI private methods *
 ***********************/
static char spiSendByte(char byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI peripheral */
  SPI_SendData(RADIO_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_ReceiveData(RADIO_SPI);
}

static char spiReceiveByte()
{
  return spiSendByte(DUMMY_BYTE);
}

/****************************************************************
 * nRF SPI commands, Every commands return the status byte      *
 ****************************************************************/

/* Read len bytes from a nRF24L register. 5 Bytes max */
unsigned char nrfReadReg(unsigned char address, unsigned char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte( CMD_R_REG | (address & 0x1F) );
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    buffer[i]=spiReceiveByte();

  RADIO_DIS_CS();

  return status;
}

/* Write len bytes a nRF24L register. 5 Bytes max */
unsigned char nrfWriteReg(unsigned char address, unsigned char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the write command with the address */
  status = spiSendByte(CMD_W_REG | (address & 0x1F) );
  /* Write LEN bytes */
  for(i=0; i<len; i++)
    spiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

/* Write only one byte (useful for most of the reg.) */
unsigned char nrfWrite1Reg(unsigned char address, unsigned char byte)
{
  return nrfWriteReg(address, &byte, 1);
}

/* Read only one byte (useful for most of the reg.) */
unsigned char nrfRead1Reg(unsigned char address) {
  unsigned char byte;

  nrfReadReg(address, &byte, 1);

  return byte;
}

/* Sent the NOP command. Used to get the status byte */
unsigned char nrfNop()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_NOP);
  RADIO_DIS_CS();

  return status;
}

unsigned char nrfFlushRx()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_FLUSH_RX);
  RADIO_DIS_CS();

  return status;
}

unsigned char nrfFlushTx()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_FLUSH_TX);
  RADIO_DIS_CS();

  return status;
}

// Return the payload length
unsigned char nrfRxLength(unsigned int pipe)
{
  unsigned char length;

  RADIO_EN_CS();
  spiSendByte(CMD_RX_PL_WID);
  length = spiReceiveByte();
  RADIO_DIS_CS();

  return length;
}

unsigned char nrfActivate()
{
  unsigned char status;
  
  RADIO_EN_CS();
  status = spiSendByte(CMD_ACTIVATE);
  spiSendByte(ACTIVATE_DATA);
  RADIO_DIS_CS();

  return status;
}

// Write the ack payload of the pipe 0
unsigned char nrfWriteAck(unsigned int pipe, unsigned char *buffer, int len)
{
  unsigned char status;
  int i;

//  ASSERT(pipe<6);

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte(CMD_W_ACK_PAYLOAD(pipe));
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    spiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

// Read the RX payload
unsigned char nrfReadRX(unsigned char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte(CMD_R_RX_PAYLOAD);
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    buffer[i]=spiReceiveByte();

  RADIO_DIS_CS();

  return status;
}

unsigned char nrfWriteTx(unsigned char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte(CMD_W_TX_PAYLOAD);
  /* Read LEN bytes */
  for(i=0; i<len; i++)
		spiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

/* Interrupt service routine, call the interrupt callback
 */
void nrfIsr()
{
  if (interruptCb)
    interruptCb();

  return;
}

void nrfSetInterruptCallback(void (*cb)(void))
{
  interruptCb = cb;
}

void nrfSetChannel(unsigned int channel)
{
  if (channel<126)
    nrfWrite1Reg(REG_RF_CH, channel);
}

void nrfSetDatarate(int datarate)
{
  switch(datarate)
  {
    case RADIO_RATE_250K:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_250K);
      break;
    case RADIO_RATE_1M:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_1M);
      break;
    case RADIO_RATE_2M:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_2M);
      break;
  }  
}

void nrfSetAddress(uint8_t pipe, unsigned char *txAddr, unsigned char *rxAddr)
{
  uint8_t len = ADDR_WIDTH;

  if (pipe > 1)
    len = 1;

	if(pipe > ADDR_WIDTH)
		return;
	if(txAddr)
		nrfWriteReg(REG_TX_ADDR, txAddr, len);

	if(rxAddr)
		nrfWriteReg(REG_RX_ADDR_P0 + pipe, rxAddr, len);
}

void nrfSetEnable(bool enable)
{
  if (enable)
  {
    RADIO_EN_CE();
  } 
  else
  {
    RADIO_DIS_CE();
  }
}

void nrfSetMode(uint8_t mode)
{
	uint8_t data, status;

	switch(mode) {
		case TX_MODE:
			RADIO_DIS_CE();
		
			nrfSetAddress(0, (uint8_t *)tx_addr, (uint8_t *)rx_addr);
			nrfWrite1Reg(REG_EN_AA, 0x01);			//Auto Ack
			nrfWrite1Reg(REG_EN_RXADDR, 0x01);
			nrfWrite1Reg(REG_SETUP_RETR, 0x1a);	//Auto resend
			nrfSetChannel(40);									//Set the radio channel
		  nrfSetDatarate(RADIO_RATE_2M);			//Set the radio data rate
			nrfWrite1Reg(REG_CONFIG, 0x0e);
		
			RADIO_EN_CE();
		
			break;
		case RX_MODE:
			RADIO_DIS_CE();
		
			nrfSetAddress(0, 0, (uint8_t *)rx_addr);
			nrfWrite1Reg(REG_EN_AA, 0x01);			//Auto Ack
			nrfWrite1Reg(REG_EN_RXADDR, 0x01);
			nrfSetChannel(40);									//Set the radio channel
			nrfWrite1Reg(REG_RX_PW_P0, RX_PLOAD_WIDTH);
		  nrfSetDatarate(RADIO_RATE_2M);			//Set the radio data rate
			nrfWrite1Reg(REG_CONFIG, 0x0f);
		
			RADIO_EN_CE();
		
			break;
	}
}

unsigned char nrfGetStatus()
{
  return nrfNop();
}

bool nrfInterruptActive(void)
{
  return (GPIO_ReadInputDataBit(RADIO_GPIO_IRQ_PORT, RADIO_GPIO_IRQ) == Bit_RESET);
}

/* Interruption initialisation */
static void extiInit()
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_RADIO_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* Initialisation */
void nrfInit(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

  /* Enable the EXTI interrupt router */
  extiInit();

  /* Enable SPI and GPIO clocks */
  RCC_AHB1PeriphClockCmd(RADIO_GPIO_SPI_CLK | RADIO_GPIO_CS_PERIF | RADIO_GPIO_CE_PERIF | RADIO_GPIO_IRQ_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(RADIO_SPI_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure the interruption (EXTI Source) */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_IRQ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RADIO_GPIO_IRQ_PORT, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(RADIO_GPIO_IRQ_SRC_PORT, RADIO_GPIO_IRQ_SRC);
  EXTI_InitStructure.EXTI_Line = RADIO_GPIO_IRQ_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Configure I/O for the Chip Enable */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RADIO_GPIO_CE_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RADIO_GPIO_CS_PORT, &GPIO_InitStructure);

  /* Configure SPI pins: SCK and MOSI MISO */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_SCK |  RADIO_GPIO_SPI_MOSI | RADIO_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(RADIO_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* disable the chip select */
  RADIO_DIS_CS();

  /* disable the chip enable */
  RADIO_DIS_CE();

  //Map SPI to alternate functions
  GPIO_PinAFConfig(RADIO_GPIO_SPI_PORT, RADIO_GPIO_AF_SPI_SCK_PIN,  RADIO_GPIO_AF_SPI);
  GPIO_PinAFConfig(RADIO_GPIO_SPI_PORT, RADIO_GPIO_AF_SPI_MISO_PIN, RADIO_GPIO_AF_SPI);
  GPIO_PinAFConfig(RADIO_GPIO_SPI_PORT, RADIO_GPIO_AF_SPI_MOSI_PIN, RADIO_GPIO_AF_SPI);

  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(RADIO_SPI, &SPI_InitStructure);

  /* Enable the SPI  */
  SPI_Cmd(RADIO_SPI, ENABLE);
  
  isInit = true;
}

bool nrfTest(void)
{
	uint8_t data[5];

	if(!isInit)
		return false;
	
	nrfReadReg(REG_RX_ADDR_P0, data, 5);

  return !(memcmp(data, rx_addr, 5));
}

void extiInterruptHandler(void)
{
  if (EXTI_GetITStatus(RADIO_GPIO_IRQ_LINE) == SET)
  {
    nrfIsr();
    EXTI_ClearITPendingBit(RADIO_GPIO_IRQ_LINE);
  }
}
