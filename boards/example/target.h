/** @file   target.h
    @author M. P. Hayes, UCECE
    @date   02 June 2007
    @brief  Target definitions for Wireless Treetap Rev 2 (2015)
*/
#ifndef TARGET_H
#define TARGET_H

#include "mat91lib.h"

/* System clocks  */
#define F_XTAL 16e6
#define MCU_PLL_MUL 12
#define MCU_PLL_DIV 1
#define MCU_USB_DIV 2
/* 192 MHz  */
#define F_PLL (F_XTAL / MCU_PLL_DIV * MCU_PLL_MUL)
/* 96 MHz  */
#define F_CPU (F_PLL / 2)

#define LED_FLASH PA27_PIO

/* Regulator enable pin for nrf module */
#define EN_NRF_PIO PA10_PIO

/* Nordic radio pin definitions */
#define NRF_CE_PIO PA11_PIO
#define NRF_CSK_PIO PA14_PIO
#define NRF_CSN_PIO PA9_PIO
#define NRF_IRQ_PIO PA26_PIO
#define NRF_MOSI_PIO PA13_PIO
#define NRF_MISO_PIO PA12_PIO

#define NRF_SPI_CHANNEL 0

#endif /* TARGET_H  */
