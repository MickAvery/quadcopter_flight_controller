/**
 * \file   pinconf.h
 * \author Mav Cuyugan
 * \brief  Macros to map MCU pins to peripherals
 */

#ifndef _PINCONF_H
#define _PINCONF_H

#include <hal.h>

/* UART pins */

#define UART_TX_PORT   GPIOA
#define UART_TX_PADNUM 0

#define UART_RX_PORT   GPIOA
#define UART_RX_PADNUM 1

#define UART_PIN_ALTMODE 8

#endif /* _PINCONF_H */