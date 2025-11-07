/**
  ******************************************************************************
  * @file           : mcp320x.c
  * @brief          : MCP320x ADC driver implementation
			
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
  * Modifications and additions are � 2025 Invictus Cockpit Systems.
  *
  * This software has been carefully modified for a specific purpose.  It is not recommended for use outside of the Invictus HOTAS system.
  *
  ******************************************************************************
  */

#include "mcp320x.h"

static uint8_t tmp_buf[3];

uint16_t MCP320x_GetData(sensor_t * sensor, uint8_t channel)
{
	uint16_t ret;

	// MCP3202 only
	ret = (sensor->data[1 + 3*channel] & 0x0F)<<8 | sensor->data[2 + 3*channel];

	return ret;
}

void MCP320x_StartDMA(sensor_t * sensor, uint8_t channel)
{
	sensor->rx_complete = 0;
	sensor->tx_complete = 1;
	sensor->curr_channel = channel;

	// CS low
	pin_config[sensor->source].port->ODR &= ~pin_config[sensor->source].pin;

	// MCP3202 only
	tmp_buf[0] = 0x01;
	tmp_buf[1] = (channel == 1) ? 0xE0 : 0xA0;
	SPI_FullDuplex_TransmitReceive(&tmp_buf[0],&sensor->data[3*channel], 3, MCP32xx_SPI_MODE);
}

void MCP320x_StopDMA(sensor_t * sensor)
{	
	DMA_Cmd(DMA1_Channel2, DISABLE);
	// CS high
	pin_config[sensor->source].port->ODR |= pin_config[sensor->source].pin;
	sensor->rx_complete = 1;
	sensor->tx_complete = 1;
}


