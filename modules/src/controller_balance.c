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
 *
 */
#include <stdbool.h>
#include <string.h>
 
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "controller_balance.h"
#include "pid.h"

__packed
struct pidValue {
	float kp;
	float ki;
	float kd;
};

__packed
union pidPack{
	uint8_t data[12];
	struct pidValue pidValue;
};

/*
#define TRUNCATE_SINT16(out, in) \
  {\
    if (in > INT16_MAX) out = (int16_t)INT16_MAX;\
    else if (in < INT16_MIN) out = (int16_t)INT16_MIN;\
    else out = (int16_t)in;\
  }
*/

//Fancier version
#define TRUNCATE_SINT16(out, in) (out = (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

//Better semantic
#define SATURATE_SINT16(in) ( (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

PidObject pidVelocity;
PidObject pidTurn;

static union pidPack *velocityPIDPack;
static union pidPack *turnPIDPack;

static bool isInit;

void controllerInit()
{
  if(isInit)
    return;
  
  //TODO: get parameters from configuration manager instead
  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD);
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
  
	pidInit(&pidVelocity, 0, 0, 0, 0);
	pidInit(&pidTurn, 0, 0, 0, 0);
	pidSetIntegralLimit(&pidVelocity, UINT16_MAX);
	pidSetIntegralLimit(&pidTurn, UINT16_MAX);
	
  isInit = true;
}

bool controllerTest()
{
  return isInit;
}

void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float *rollOutput, float *pitchOutput, float *yawOutput)
{
	float yawError;
	
  pidSetDesired(&pidRoll, eulerRollDesired);
	*rollOutput = pidUpdate(&pidRoll, eulerRollActual, true);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
	*pitchOutput = pidUpdate(&pidPitch, eulerPitchActual, true);
  // Update PID for yaw axis
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0)
    yawError -= 360.0;
  else if (yawError < -180.0)
    yawError += 360.0;
  pidSetError(&pidYaw, yawError);
	*yawOutput = pidUpdate(&pidYaw, eulerYawActual, false);
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
}

void velocityGetPID(TransPacket *dataPack)
{
	velocityPIDPack = (union pidPack *)&dataPack->dataBuf[0];
	
	pidVelocity.kp = velocityPIDPack->pidValue.kp;
	pidVelocity.ki = velocityPIDPack->pidValue.ki;
	pidVelocity.kd = velocityPIDPack->pidValue.kd;
}

void turnGetPID(TransPacket *dataPack)
{
	turnPIDPack = (union pidPack *)&dataPack->dataBuf[0];
	
	pidTurn.kp = turnPIDPack->pidValue.kp;
	pidTurn.ki = turnPIDPack->pidValue.ki;
	pidTurn.kd = turnPIDPack->pidValue.kd;
}

void velocityPIDBack(TransPacket *dataPack)
{
	velocityPIDPack = (union pidPack *)&dataPack->dataBuf[0];
	
	velocityPIDPack->pidValue.kp = pidVelocity.kp;
	velocityPIDPack->pidValue.ki = pidVelocity.ki;
	velocityPIDPack->pidValue.kd = pidVelocity.kd;
	
	dataPack->len = 12;
}

void turnPIDBack(TransPacket *dataPack)
{
	turnPIDPack = (union pidPack *)&dataPack->dataBuf[0];
	
	turnPIDPack->pidValue.kp = pidTurn.kp;
	turnPIDPack->pidValue.ki = pidTurn.ki;
	turnPIDPack->pidValue.kd = pidTurn.kd;
	
	dataPack->len = 12;
}
