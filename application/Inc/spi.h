/**
  ******************************************************************************
  * @file           : spi.h
  * @brief          : Header file for spi.h                 
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.0.0
  * @date           2025-07-30
  *
  * Based on FreeJoy firmware by Yury Vostrenkov (2020)
  * https://github.com/FreeJoy-Team/FreeJoy
  *
  * This software includes original or modified portions of FreeJoy, distributed
  * under the terms of the GNU General Public License v3.0 or later:
  * https://www.gnu.org/licenses/gpl-3.0.html
  *
  * Modifications and additions are © 2025 Invictus Cockpit Systems.
  *
  * This software has been carefully modified for a specific purpose.  It is not recommended for use outside of the Invictus HOTAS system.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

void SPI_Start(void);
void SPI_HalfDuplex_Transmit(uint8_t * data, uint16_t length, uint8_t spi_mode);
void SPI_HalfDuplex_Receive(uint8_t * data, uint16_t length, uint8_t spi_mode);
void SPI_FullDuplex_TransmitReceive(uint8_t * tx_data, uint8_t * rx_data, uint16_t length, uint8_t spi_mode);
#endif 	/* __SPI_H__ */

