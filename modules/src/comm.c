/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 * comm.c - High level communication module
 */

#include <stdbool.h>

#include "config.h"

//#include "radiolink.h"
//#include "crtp.h"
//#include "console.h"
//#include "crtpservice.h"
//#include "multilog.h"
//#include "pidctrl.h"
//#include "param.h"
//#include "log.h"
//#include "eskylink.h"
#include "uartWiFilink.h"

static bool isInit;

void commInit(void)
{
  if (isInit)
    return;
  
//  usbInit();
//#ifndef USE_ESKYLINK
//  radiolinkInit();
//#else
//  eskylinkInit();
//#endif
	
//  multilogLaunch(4);
//  logInit();
//  pidCtrlInit();
//  consoleInit();
	
	wifiInit();
  
  isInit = true;
}

bool commTest(void)
{
  bool pass = isInit;
  
//  pass &= usbTest();
//  pass &= radiolinkTest();
//  pass &= consoleTest();
//  pass &= paramTest();
  pass &= wifiTest();
	
  return pass;
}

