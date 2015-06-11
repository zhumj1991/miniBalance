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
 * nrf24l01.h: nRF24L01(-p) PRX mode low level driver
 */

#ifndef __ESP8266_H__
#define __ESP8266_H__

#include <stdbool.h>
#include "stm32f4xx.h"

void esp8266Init(void);

bool esp8266Test(void);

bool esp8266GetDataWithTimout(uint8_t *c);
void esp8266SendData(uint8_t* data, uint32_t size);

void esp8266Isr(void);
#endif
