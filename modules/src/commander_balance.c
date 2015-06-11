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
#include <string.h>
#include "stm32f4xx_conf.h"

#include "FreeRTOS.h"
#include "task.h"

#include "commander_balance.h"
#include "data_handling.h"


struct CommanderValues
{
	uint8_t flagForward;
	uint8_t flagBackward;
	uint8_t flagLeft;
	uint8_t flagRight;
	uint8_t throttle;
};

struct CommanderValues targetVal;
static bool isInit;
static uint32_t lastUpdate;
static bool isInactive;

static void commanderWatchdog(void);
static void commanderWatchdogReset(void);

void commanderInit(void)
{
  if(isInit)
    return;

  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;
}

bool commanderTest(void)
{
  return isInit;
}

void commanderGetPacket(TransPacket * pk)
{
  targetVal = *((struct CommanderValues *)pk->dataBuf);
	commanderWatchdogReset();
}

static void commanderWatchdog(void)
{
  uint32_t ticktimeSinceUpdate;

  ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;

  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    memset((void *)&targetVal, 0, sizeof(targetVal));
		
    isInactive = true;
  }
  else
  {
    isInactive = false;
  }
}

static void commanderWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}


void comamndGetControl(uint8_t *flagForward, uint8_t *flagBackward,
				uint8_t *flagLeft, uint8_t *flagRight)
{
	*flagForward = targetVal.flagBackward;
	*flagBackward =targetVal.flagForward;
	*flagLeft =	targetVal.flagLeft;
	*flagRight = targetVal.flagRight;

  commanderWatchdog();
}

