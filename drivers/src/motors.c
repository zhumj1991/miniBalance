/**
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
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <stdbool.h>

#include "motors.h"

// ST lib includes
#include "stm32f4xx_conf.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// HW defines
#define MOTORS_GPIO_TIM_PERIF     RCC_APB1Periph_TIM3
#define MOTORS_GPIO_TIM_M1_2      TIM3
#define MOTORS_GPIO_TIM_M1_2_DBG  DBGMCU_TIM3_STOP

#define MOTORS_GPIO_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_PORT          GPIOB
#define MOTORS_GPIO_M1            GPIO_Pin_0 // T3_CH3
#define MOTORS_GPIO_M1_AF_PIN     GPIO_PinSource0
#define MOTORS_GPIO_M1_AF					GPIO_AF_TIM3
#define MOTORS_GPIO_M2            GPIO_Pin_1 // T3_CH4
#define MOTORS_GPIO_M2_AF_PIN     GPIO_PinSource1
#define MOTORS_GPIO_M2_AF					GPIO_AF_TIM3

#define	MOTORS_DIR_GPIO_PERIF			RCC_AHB1Periph_GPIOC
#define MOTORS_DIR_GPIO_PORT      GPIOC
#define	MOTORS_DIR_GPIO_AIN1			GPIO_Pin_0
#define	MOTORS_DIR_GPIO_AIN2			GPIO_Pin_1
#define	MOTORS_DIR_GPIO_BIN1			GPIO_Pin_2
#define	MOTORS_DIR_GPIO_BIN2			GPIO_Pin_3

#define	MOTORS_STBY_GPIO_PERIF		RCC_AHB1Periph_GPIOC
#define MOTORS_STBY_GPIO_PORT     GPIOC
#define	MOTORS_STBY_GPIO					GPIO_Pin_4

/* define GPIO high/low */
#define	STBY_H										MOTORS_STBY_GPIO_PORT->BSRRL = MOTORS_STBY_GPIO
#define	STBY_L										MOTORS_STBY_GPIO_PORT->BSRRH = MOTORS_STBY_GPIO
#define	AIN1_H										MOTORS_DIR_GPIO_PORT->BSRRL = MOTORS_DIR_GPIO_AIN1
#define	AIN1_L										MOTORS_DIR_GPIO_PORT->BSRRH = MOTORS_DIR_GPIO_AIN1
#define	AIN2_H										MOTORS_DIR_GPIO_PORT->BSRRL = MOTORS_DIR_GPIO_AIN2
#define	AIN2_L										MOTORS_DIR_GPIO_PORT->BSRRH = MOTORS_DIR_GPIO_AIN2
#define	BIN1_H										MOTORS_DIR_GPIO_PORT->BSRRL = MOTORS_DIR_GPIO_BIN1
#define	BIN1_L										MOTORS_DIR_GPIO_PORT->BSRRH = MOTORS_DIR_GPIO_BIN1
#define	BIN2_H										MOTORS_DIR_GPIO_PORT->BSRRL = MOTORS_DIR_GPIO_BIN2
#define	BIN2_L										MOTORS_DIR_GPIO_PORT->BSRRH = MOTORS_DIR_GPIO_BIN2

/* Utils Conversion macro */
#define C_BITS_TO_16(x) ((x) << (16 - MOTORS_PWM_BITS))
#define C_16_TO_BITS(x) ((x) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1))

static bool isInit=false;

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit(void)
{
  if (isInit)
    return;

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Enable gpio and the timer
  RCC_AHB1PeriphClockCmd(MOTORS_GPIO_PERIF | MOTORS_DIR_GPIO_PERIF | MOTORS_STBY_GPIO_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(MOTORS_GPIO_TIM_PERIF, ENABLE);

	// Configure the GPIO for the direction output
	GPIO_InitStructure.GPIO_Pin = MOTORS_DIR_GPIO_AIN1 | \
																 MOTORS_DIR_GPIO_AIN2 | \
																 MOTORS_DIR_GPIO_BIN1 | \
																 MOTORS_DIR_GPIO_BIN2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(MOTORS_DIR_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = MOTORS_STBY_GPIO;
	GPIO_Init(MOTORS_STBY_GPIO_PORT, &GPIO_InitStructure);
	
  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Pin = (MOTORS_GPIO_M1 | MOTORS_GPIO_M2);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MOTORS_GPIO_PORT, &GPIO_InitStructure);

  //Remap M1-2
  GPIO_PinAFConfig(MOTORS_GPIO_PORT, MOTORS_GPIO_M1_AF_PIN, MOTORS_GPIO_M1_AF);
	GPIO_PinAFConfig(MOTORS_GPIO_PORT, MOTORS_GPIO_M2_AF_PIN, MOTORS_GPIO_M2_AF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTORS_GPIO_TIM_M1_2, &TIM_TimeBaseStructure);

  //PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC3Init(MOTORS_GPIO_TIM_M1_2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(MOTORS_GPIO_TIM_M1_2, TIM_OCPreload_Enable);

  TIM_OC4Init(MOTORS_GPIO_TIM_M1_2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTORS_GPIO_TIM_M1_2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(MOTORS_GPIO_TIM_M1_2, ENABLE);
  //Enable the timer
  TIM_Cmd(MOTORS_GPIO_TIM_M1_2, ENABLE);

  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(MOTORS_GPIO_TIM_M1_2, ENABLE);

  // Halt timer during debug halt.
  DBGMCU_Config(MOTORS_GPIO_TIM_M1_2_DBG, ENABLE);
  
  isInit = true;
}

bool motorsTest(void)
{
	motorsSetDir(MOTOR_LEFT ,MOTOR_STOP);
	motorsSetDir(MOTOR_RIGHT ,MOTOR_STOP);

  return isInit;
}


void motorsSetRatio(int id, uint16_t ratio)
{
  switch(id) {
    case MOTOR_LEFT:
      TIM_SetCompare3(MOTORS_GPIO_TIM_M1_2, C_16_TO_BITS(ratio));
      break;
    case MOTOR_RIGHT:
      TIM_SetCompare3(MOTORS_GPIO_TIM_M1_2, C_16_TO_BITS(ratio));
      break;
  }

  return;
}

int motorsGetRatio(int id)
{
  switch(id) {
    case MOTOR_LEFT:
      return C_BITS_TO_16(TIM_GetCapture3(MOTORS_GPIO_TIM_M1_2));
    case MOTOR_RIGHT:
      return C_BITS_TO_16(TIM_GetCapture3(MOTORS_GPIO_TIM_M1_2));
  }

  return -1;
}

void motorsSetDir(int id, int dir)
{
	//TODO: change direction by hardware
	switch(id) {
    case MOTOR_LEFT:
			switch(dir) {
				case MOTOR_BACKWARD:
					STBY_H;  
					AIN1_L; AIN2_H;  
					break;
				case MOTOR_FORWARD:
					STBY_H;  
					AIN1_H; AIN2_L;   
					break;
				case MOTOR_STOP:
					STBY_L;  
					AIN1_L; AIN2_L;  
					break;
				}
		 break;
    case MOTOR_RIGHT:
			switch(dir) {
				case MOTOR_BACKWARD:
					STBY_H; 
					BIN1_H; BIN2_L;
					break;
				case MOTOR_FORWARD:
					STBY_H;    
					BIN1_L; BIN2_H; 
					break;
				case MOTOR_STOP:
					STBY_L;    
					BIN1_L; BIN2_L;
					break;
				}
		break;
	}
}

#ifdef MOTOR_RAMPUP_TEST
// FreeRTOS Task to test the Motors driver with a rampup of each motor alone.
void motorsTestTask(void* params)
{
  int step=0;
  float rampup = 0.01;

  motorsSetupMinMaxPos();
  motorsSetRatio(MOTOR_LEFT, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_RIGHT, 1*(1<<16) * 0.0);
  vTaskDelay(M2T(1000));

  while(1)
  {
    vTaskDelay(M2T(100));

    motorsSetRatio(MOTOR_LEFT, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_RIGHT, 1*(1<<16) * rampup);

    rampup += 0.001;
    if (rampup >= 0.1)
    {
      if(++step > 3) step=0;
      rampup = 0.01;
    }
  }
}
#else
// FreeRTOS Task to test the Motors driver
void motorsTestTask(void* params)
{
  static const int sequence[] = {0.1*(1<<16), 0.15*(1<<16), 0.2*(1<<16), 0.25*(1<<16)};
  int step=0;

  //Wait 3 seconds before starting the motors
  vTaskDelay(M2T(3000));

  while(1) {
    motorsSetRatio(MOTOR_LEFT, sequence[step % 4]);
    motorsSetRatio(MOTOR_RIGHT, sequence[(step+2) % 4]);

    if(++step > 3) step=0;

    vTaskDelay(M2T(1000));
  }
}
#endif

