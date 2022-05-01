/*
 * mx25flash_config.h
 *
 *  Created on: Apr 25, 2022
 *      Author: User
 */

#ifndef MX25FLASH_CONFIG_H_
#define MX25FLASH_CONFIG_H_

/***************************************************************************//**
 * @file
 * @brief BRD4150B specific configuration for on-board serial flash.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef MX25CONFIG_H
#define MX25CONFIG_H

#include "em_device.h"
#include "em_gpio.h"

#define MX25_PORT_MOSI         gpioPortC
#define MX25_PIN_MOSI          02
#define MX25_PORT_MISO         gpioPortC
#define MX25_PIN_MISO          00
#define MX25_PORT_SCLK         gpioPortC
#define MX25_PIN_SCLK          01
#define MX25_PORT_CS           gpioPortC
#define MX25_PIN_CS            03

#define MX25_USART             USART0
#define MX25_USART_CLK         cmuClock_USART0


#endif // MX25CONFIG_H

#endif /* MX25FLASH_CONFIG_H_ */
