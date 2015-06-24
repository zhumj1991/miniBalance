/*
	可以输出到GPIO的TIM通道:

	TIM1_CH1, PA8,	PE9,
	TIM1_CH2, PA9,	PE11
	TIM1_CH3, PA10,	PE13
	TIM1_CH4, PA11,	PE14

	TIM2_CH1,
	TIM2_CH2, PA1,	PB3
	TIM2_CH3, PA2,	PB10
	TIM2_CH4, PA3,	PB11

	TIM3_CH1, PA6,  PB4, PC6
	TIM3_CH2, PA7,	PB5, PC7
	TIM3_CH3, PB0,	PC8
	TIM3_CH4, PB1,	PC9

	TIM4_CH1, PB6,  PD12
	TIM4_CH2, PB7,	PD13
	TIM4_CH3, PB8,	PD14
	TIM4_CH4, PB9,	PD15

	TIM5_CH1, PA0,  PH10
	TIM5_CH2, PA1,	PH11
	TIM5_CH3, PA2,	PH12
	TIM5_CH4, PA3,	PI10

	TIM8_CH1, PC6,  PI5
	TIM8_CH2, PC7,	PI6
	TIM8_CH3, PC8,	PI7
	TIM8_CH4, PC9,	PI2

	TIM9_CH1, PA2,  PE5
	TIM9_CH2, PA3,	PE6

	TIM10_CH1, PB8,  PF6

	TIM11_CH1, PB9,  PF7

	TIM12_CH1, PB14,  PH6
	TIM12_CH2, PB15,  PH9

	TIM13_CH1, PA6,  PF8
	TIM14_CH1, PA7,  PF9

	APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14
	APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11
*/

#include <stdbool.h>

#include "encoder.h"

/* ST includes */
#include "stm32f4xx.h"

#include "nvicconf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// HW defines
/* M1 Encoder */
#define ENCODER_M1_TIM_PERIF				RCC_APB1Periph_TIM5
#define ENCODER_M1_TIM					    TIM5
#define ENCODER_M1_TIM_DBG  				DBGMCU_TIM5_STOP

#define ENCODER_M1_GPIO_PERIF				RCC_AHB1Periph_GPIOA
#define ENCODER_M1_GPIO_PORT				GPIOA
#define ENCODER_M1_GPIO_0						GPIO_Pin_0	// T5_CH1
#define ENCODER_M1_GPIO_1						GPIO_Pin_1	// T5_CH2
#define ENCODER_M1_GPIO_AF					GPIO_AF_TIM5
#define	ENCODER_M1_GPIO_AF_PIN_0		GPIO_PinSource0
#define	ENCODER_M1_GPIO_AF_PIN_1		GPIO_PinSource1

/* M2 Encoder */
#define ENCODER_M2_TIM_PERIF				RCC_APB1Periph_TIM4
#define ENCODER_M2_TIM					    TIM4
#define ENCODER_M2_TIM_DBG  				DBGMCU_TIM4_STOP

#define ENCODER_M2_GPIO_PERIF				RCC_AHB1Periph_GPIOB
#define ENCODER_M2_GPIO_PORT				GPIOB
#define ENCODER_M2_GPIO_0						GPIO_Pin_6	// T4_CH1
#define ENCODER_M2_GPIO_1						GPIO_Pin_7	// T4_CH2
#define ENCODER_M2_GPIO_AF					GPIO_AF_TIM4
#define	ENCODER_M2_GPIO_AF_PIN_0		GPIO_PinSource6
#define	ENCODER_M2_GPIO_AF_PIN_1		GPIO_PinSource7


static bool isInit=false;

static void encoderM1Init()
{
	//Init structures
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_ICInitTypeDef					TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	//Enable gpio and the timer
	RCC_AHB1PeriphClockCmd(ENCODER_M1_GPIO_PERIF, ENABLE);
	RCC_APB1PeriphClockCmd(ENCODER_M1_TIM_PERIF, ENABLE);
	
	// Configure the GPIO for the timer caputure
	GPIO_InitStructure.GPIO_Pin = ENCODER_M1_GPIO_0 | ENCODER_M1_GPIO_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(ENCODER_M1_GPIO_PORT, &GPIO_InitStructure);
	
	//Remap M1-2
	GPIO_PinAFConfig(ENCODER_M1_GPIO_PORT, ENCODER_M1_GPIO_AF_PIN_0, ENCODER_M1_GPIO_AF);
	GPIO_PinAFConfig(ENCODER_M1_GPIO_PORT, ENCODER_M1_GPIO_AF_PIN_1, ENCODER_M1_GPIO_AF);

	//Timer configuration
	TIM_DeInit(ENCODER_M1_TIM);
	TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRESCALE;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(ENCODER_M1_TIM, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(ENCODER_M1_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 3;
  TIM_ICInit(ENCODER_M1_TIM, &TIM_ICInitStructure);
	
	TIM_SetCounter(ENCODER_M1_TIM, 0x7FFF);
	TIM_ClearFlag(ENCODER_M1_TIM, TIM_FLAG_Update);
	
	TIM_Cmd(ENCODER_M1_TIM, ENABLE);
}

static void encoderM2Init()
{
	//Init structures
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_ICInitTypeDef	TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	
	//Enable gpio and the timer
	RCC_AHB1PeriphClockCmd(ENCODER_M2_GPIO_PERIF, ENABLE);
	RCC_APB1PeriphClockCmd(ENCODER_M2_TIM_PERIF, ENABLE);
	
	// Configure the GPIO for the timer caputure
	GPIO_InitStructure.GPIO_Pin = ENCODER_M2_GPIO_0 | ENCODER_M2_GPIO_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(ENCODER_M2_GPIO_PORT, &GPIO_InitStructure);
	
	//Remap M1-2
	GPIO_PinAFConfig(ENCODER_M2_GPIO_PORT, ENCODER_M2_GPIO_AF_PIN_0, ENCODER_M2_GPIO_AF);
	GPIO_PinAFConfig(ENCODER_M2_GPIO_PORT, ENCODER_M2_GPIO_AF_PIN_1, ENCODER_M2_GPIO_AF);

	//Timer configuration
	TIM_DeInit(ENCODER_M2_TIM);
	TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRESCALE;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(ENCODER_M2_TIM, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(ENCODER_M2_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 3;
  TIM_ICInit(ENCODER_M2_TIM, &TIM_ICInitStructure);
	
	TIM_SetCounter(ENCODER_M2_TIM, 0x7FFF);
	TIM_ClearFlag(ENCODER_M2_TIM, TIM_FLAG_Update);
	
	TIM_Cmd(ENCODER_M2_TIM, ENABLE);
}

void encoderInit(void)
{
	if (isInit)
    return;

	encoderM1Init();
	encoderM2Init();
		
	isInit = true;
}

bool encoderTest()
{
	return isInit;
}

void encoderCalculate(int32_t encoder_ml, int32_t encoder_mr)
{
	encoder_ml = TIM_GetCounter(ENCODER_M1_TIM) - 0x7FFF;
	TIM_SetCounter(ENCODER_M1_TIM, 0x7FFF);
	
	encoder_mr = TIM_GetCounter(ENCODER_M2_TIM) - 0x7FFF;
	TIM_SetCounter(ENCODER_M2_TIM, 0x7FFF);
}
