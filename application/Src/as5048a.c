/**
  ******************************************************************************
  * @file           : as5048a.c
  * @brief          : AS5048A sensors driver implementation
			
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

#include "as5048a.h"

uint8_t gtmp_buf[2];

void reset_err_flag(sensor_t * sensor){
	uint16_t tmp;
	// CS low
	pin_config[sensor->source].port->ODR &= ~pin_config[sensor->source].pin;
	gtmp_buf[0] = 0x40;
	gtmp_buf[1] = 0x01;
	
	SPI_HalfDuplex_Transmit(&gtmp_buf[0], 2, AS5048A_SPI_MODE);
	
	do{
		tmp = DMA_GetCurrDataCounter(DMA1_Channel3);
	}  while(tmp!=0);
	
	// CS high
	pin_config[sensor->source].port->ODR |= pin_config[sensor->source].pin;
	
}
/**
  * @brief AS5048A get measured data
  * @param data: variable for storing data
  * @param sensor: Sensor struct
  * @retval status
  */
int AS5048A_GetData(uint16_t * data, sensor_t * sensor, uint8_t channel)
{
	int ret = 0;
	uint16_t tmp;
	// wait till the DMA channel has finished
	do{
		tmp = DMA_GetCurrDataCounter(DMA1_Channel2);
	}  while(tmp!=0);
	tmp = sensor->data[0];
	tmp = (tmp << 8) | sensor->data[1];
	*data = tmp & 0x3FFF;
	// check error bit
	if((tmp&0x4000)!=0){
		// reset err flag with 0x4001 command
		reset_err_flag(sensor);
		ret = -1;
	}
	// check parity
	tmp ^= tmp >> 8;
	tmp ^= tmp >> 4;
	tmp ^= tmp >> 2;
	tmp ^= tmp >> 1;
	if((tmp & 1)==1) ret = -2;
	return ret;
}


/**
  * @brief AS5048A start processing with DMA
  * @param sensor: Sensor struct
  * @retval None
  */
void AS5048A_StartDMA(sensor_t * sensor)
{	
	
	sensor->rx_complete = 0;
	sensor->tx_complete = 1;

	gtmp_buf[0] = 0x3F;		// Read Meas. command: 0x3FFF
	gtmp_buf[1] = 0xFF;		// 
	
	// CS low
	pin_config[sensor->source].port->ODR &= ~pin_config[sensor->source].pin;
	SPI_FullDuplex_TransmitReceive(gtmp_buf, sensor->data, 2, AS5048A_SPI_MODE);
}

void AS5048A_StopDMA(sensor_t * sensor)
{	
	DMA_Cmd(DMA1_Channel2, DISABLE);
	// CS high
	pin_config[sensor->source].port->ODR |= pin_config[sensor->source].pin;
	sensor->rx_complete = 1;
	sensor->tx_complete = 1;

}



