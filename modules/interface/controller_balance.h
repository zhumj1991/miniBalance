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
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stdbool.h>
#include "stm32f4xx.h"
#include "commander_balance.h"


void controllerInit(void);
bool controllerTest(void);

/**
 * Make the controller run an update of the attitude PID. The output is
 * the desired rate which should be fed into a rate controller. The
 * attitude controller can be run in a slower update rate then the rate
 * controller.
 */
void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float *rollOutput, float *pitchOutput, float *yawOutput);

/**
 * Make the controller run an update of the motor velocity PID.
 */
void controllerCorrectMotorPID(
			float motoLeftActual, float motoRightActual,
			float motoLeftDesired, float motoRightDesired,
			float *motorLeftOutput, float *motorRightOutput);
/**
 * Reset controller roll, pitch and yaw PID's.
 */
void controllerResetAllPID(void);
			
void velocityGetPID(TransPacket *dataPack);
void turnGetPID(TransPacket *dataPack);
			
void turnPIDBack(TransPacket *dataPack);
void turnPIDBack(TransPacket *dataPack);
	
#endif /* CONTROLLER_H_ */
