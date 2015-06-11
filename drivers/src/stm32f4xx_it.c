/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

#include "uart_syslink.h"
#include "esp8266.h"
//#include "nrf24l01.h"
#include "i2cdev.h"


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

__asm void HardFaultHandling()
{
	BX LR
	
	IMPORT	printHardFault
	DCD		printHardFault
	
	TST LR, #4
	ITE EQ
	MRSEQ R0, MSP
	MRSNE R0, PSP
	B printHardFault
}

void printHardFault(uint32_t* hardfaultArgs)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfaultArgs[0]);
  stacked_r1 = ((unsigned long) hardfaultArgs[1]);
  stacked_r2 = ((unsigned long) hardfaultArgs[2]);
  stacked_r3 = ((unsigned long) hardfaultArgs[3]);

  stacked_r12 = ((unsigned long) hardfaultArgs[4]);
  stacked_lr = ((unsigned long) hardfaultArgs[5]);
  stacked_pc = ((unsigned long) hardfaultArgs[6]);
  stacked_psr = ((unsigned long) hardfaultArgs[7]);


  uartPrintf("[Hard fault handler]\r\n");
  uartPrintf("R0 = %x\r\n", stacked_r0);
  uartPrintf("R1 = %x\r\n", stacked_r1);
  uartPrintf("R2 = %x\r\n", stacked_r2);
  uartPrintf("R3 = %x\r\n", stacked_r3);
  uartPrintf("R12 = %x\r\n", stacked_r12);
  uartPrintf("LR = %x\r\n", stacked_lr);
  uartPrintf("PC = %x\r\n", stacked_pc);
  uartPrintf("PSR = %x\r\n", stacked_psr);
  uartPrintf("BFAR = %x\r\n", (*((volatile unsigned int *)(0xE000ED38))));
  uartPrintf("CFSR = %x\r\n", (*((volatile unsigned int *)(0xE000ED28))));
  uartPrintf("HFSR = %x\r\n", (*((volatile unsigned int *)(0xE000ED2C))));
  uartPrintf("DFSR = %x\r\n", (*((volatile unsigned int *)(0xE000ED30))));
  uartPrintf("AFSR = %x\r\n", (*((volatile unsigned int *)(0xE000ED3C))));

  while (1)
  {
	}
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  //http://www.st.com/mcu/forums-cat-6778-23.html
  //****************************************************
  //To test this application, you can use this snippet anywhere:
  // //Let's crash the MCU!
  // asm (" MOVS r0, #1 \n"
  // " LDM r0,{r1-r2} \n"
  // " BX LR; \n");
//	__asm( "TST LR, #4 \n"
//	"ITE EQ \n"
//	"MRSEQ R0, MSP \n"
//	"MRSNE R0, PSP \n"
//	"B printHardFault");
	HardFaultHandling();

}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

#ifdef NVIC_NOT_USED_BY_FREERTOS
/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
#endif

/**
 * @brief  This function handles SysTick Handler.
 */
extern void tickFreeRTOS(void);

void SysTick_Handler(void)
{
    tickFreeRTOS();
}


void USART1_IRQHandler(void)
{
  uartIsr();
}

void USART2_IRQHandler(void)
{
  esp8266Isr();
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void PPP_IRQHandler(void)
{
}

/**
  * nRF24L01
  */
//void EXTI9_5_IRQHandler(void)
//{
//	extiInterruptHandler();
//}

/**
  * I2C
  */
void DMA1_Stream3_IRQHandler(void)
{
	i2cDmaInterruptHandlerI2c2();
}

void DMA1_Stream5_IRQHandler(void)
{
	i2cDmaInterruptHandlerI2c1();
}

void DMA1_Stream6_IRQHandler(void)
{
	i2cDmaInterruptHandlerI2c1();
}

void DMA1_Stream7_IRQHandler(void)
{
	i2cDmaInterruptHandlerI2c2();
}

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
