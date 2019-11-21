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

/* I2C pins */

#define I2C_SCL_PORT   GPIOB
#define I2C_SCL_PADNUM 10

#define I2C_SDA_PORT   GPIOB
#define I2C_SDA_PADNUM 11

#define I2C_PIN_ALTMODE 4

/* ICU pins */

#define ICU_PORT    GPIOB
#define ICU_PADNUM  4
#define ICU_ALTMODE 2

/* PWM output pins */

#define PWM_NW_PORT    GPIOA
#define PWM_NW_PADNUM  5
#define PWM_NW_ALTMODE 1

#define PWM_NE_PORT    GPIOB
#define PWM_NE_PADNUM  3
#define PWM_NE_ALTMODE 1

#define PWM_SW_PORT    GPIOA
#define PWM_SW_PADNUM  2
#define PWM_SW_ALTMODE 1

#define PWM_SE_PORT    GPIOA
#define PWM_SE_PADNUM  3
#define PWM_SE_ALTMODE 1

#endif /* _PINCONF_H */