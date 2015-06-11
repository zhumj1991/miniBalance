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

#ifndef __UART_WIFI_H__
#define __UART_WIFI_H__

#include <stdbool.h>

void wifiInit(void);

bool wifiTest(void);

void esp8266Isr(void);
#endif
