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
 * Motors.h - Motor driver header file
 *
 * The motors PWM ratio is a value on 16 bits (from 0 to 0xFFFF)
 * the functions of the driver will make the conversions to the actual PWM
 * precision (ie. if the precision is 8bits 0xFFFF and 0xFF00 are equivalents).
 *
 * The precision is set in number of bits by the define MOTORS_PWM_BITS
 * The timer prescaler is set by MOTORS_PWM_PRESCALE
 */
#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/******** Defines ********/

//The following defines gives a PWM of 9 bits at ~140KHz for a sysclock of 72MHz
#define MOTORS_PWM_BITS     9
#define MOTORS_PWM_PERIOD   ((1 << MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE 0

// Motor Max Load rpn
#define	MOTOR_RPN						147

// Motors IDs define
#define MOTOR_LEFT					0
#define MOTOR_RIGHT  				1

#define	MOTOR_FORWARD				0
#define	MOTOR_BACKWARD			1
#define	MOTOR_STOP					2

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.5 * (1 << 16))
#define MOTORS_TEST_ON_TIME       M2T(10)
#define MOTORS_TEST_DELAY_TIME    M2T(50)

/*** Public interface ***/

/**
 * Initialisation. Will set all motors ratio to 0%
 */
void motorsInit(void);

/**
 * Test of the motor modules. The test will spin each motor very short in
 * the sequence M1 to M4.
 */
bool motorsTest(void);

/**
 * Set the PWM ratio of the motor 'id'
 */
void motorsSetRatio(int id, uint16_t ratio);

/**
 * Get the PWM ratio of the motor 'id'. Return -1 if wrong ID.
 */
int motorsGetRatio(int id);

/**
 * Set the PWM ratio of the motor 'dir'
 */
void motorsSetDir(int id, int dir);
/**
 * FreeRTOS Task to test the Motors driver
 */
void motorsTestTask(void* params);

#endif /* __MOTORS_H__ */

