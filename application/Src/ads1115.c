/**
  ******************************************************************************
  * @file           : ads1115.c
  * @brief          : ADS1115 ADC driver implementation
	* @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.1.0
  * @date           2025-10-25
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

#include "ads1115.h"

/**
  * @brief Get PGA configuration byte based on channel
  * @param channel: ADC channel (0 = Roll, 2 = Pitch)
  * @retval PGA config byte
  */
static uint8_t ADS1115_GetPGAConfig(uint8_t channel)
{
	if (channel == 0)  // Roll axis - PGA 8 for maximum resolution
	{
		return 0xC9;  // PGA 8: ±0.512V
	}
	else if (channel == 2)  // Pitch axis - PGA 4 (safe for 40 lbf)
	{
		return 0xC7;  // PGA 4: ±1.024V
	}

	// Default for unused channels 1 and 3
	return 0xC7;  // PGA 4: ±1.024V
}

/**
  * @brief ADS1115 init function
  * @param sensor: Sensor struct
  * @param p_dev_config: Device configuration pointer (unused for now)
  * @retval None
  */
void ADS1115_Init(sensor_t * sensor, dev_config_t * p_dev_config)
{
	int status;
	uint8_t tmp_buf[2];
	uint8_t pga_config = ADS1115_GetPGAConfig(0);  // Init with channel 0

	tmp_buf[0] = pga_config;
	tmp_buf[1] = 0xE3;  // 860 SPS
	status = I2C_WriteBlocking(sensor->address, 1, tmp_buf, 2);

	tmp_buf[0] = 0x00;
	tmp_buf[1] = 0x00;
	if (status == 0)
	{
		status = I2C_ReadBlocking(sensor->address, 1, tmp_buf, 2, 0);
	}
}

/**
  * @brief ADS1115 get measured data
  * @param sensor: Sensor struct
  * @retval data
  */
int16_t ADS1115_GetData(sensor_t * sensor, uint8_t channel)
{
	return sensor->data[3+2*channel]<<8|sensor->data[4 + 2*channel];
}

/**
  * @brief ADS1115 start processing data in blocking mode
  * @param sensor: Sensor struct
  * @retval status
  */
int ADS1115_ReadBlocking(sensor_t * sensor, uint8_t channel)
{
	int ret;
	uint8_t tmp_buf[2];
	
	ret = I2C_ReadBlocking(sensor->address, 0, tmp_buf, 2, 0);
	
	if (ret == 0 ) 
	{
		memcpy(&sensor->data[3 + 2*channel], tmp_buf, 2);
		sensor->ok_cnt++;
	}
	else sensor->err_cnt++;
	
	return ret;
}

/**
  * @brief ADS1115 set mux in blocking mode
  * @param sensor: Sensor struct
  * @param channel: ADC channel to select
  * @param p_dev_config: Device configuration pointer (unused for now)
  * @retval status
  */
int ADS1115_SetMuxBlocking(sensor_t * sensor, uint8_t channel, dev_config_t * p_dev_config)
{
	int ret;
	uint8_t tmp_buf[3];
	uint8_t pga_config = ADS1115_GetPGAConfig(channel);

	tmp_buf[0] = pga_config | (channel << 4);  // config reg MSB with dynamic PGA
	tmp_buf[1] = 0xE3;                         // config reg LSB (860 SPS)

	ret = I2C_WriteBlocking(sensor->address, 1, tmp_buf, 2);

	sensor->curr_channel = channel;

	return ret;
}

/**
  * @brief ADS1115 start processing data in DMA mode
  * @param sensor: Sensor struct
  * @retval status
  */
int ADS1115_StartDMA(sensor_t * sensor, uint8_t channel)
{
	int ret;
	
	sensor->rx_complete = 0;	
	ret = I2C_ReadNonBlocking(sensor->address, 0, &sensor->data[3 + 2*channel], 2, 1);
	
	if (ret != 0 )  	// communication error occured
	{
		sensor->err_cnt++;
		sensor->tx_complete = 1;
		sensor->rx_complete = 1;
	}
	
	return ret;
}

/**
  * @brief ADS1115 set mux in DMA mode
  * @param sensor: Sensor struct
  * @param channel: ADC channel to select
  * @param p_dev_config: Device configuration pointer (unused for now)
  * @retval status
  */
int ADS1115_SetMuxDMA(sensor_t * sensor, uint8_t channel, dev_config_t * p_dev_config)
{
	int ret;
	uint8_t pga_config = ADS1115_GetPGAConfig(channel);

	sensor->data[0] = 0x01;                        // config reg address
	sensor->data[1] = pga_config | (channel << 4); // config reg MSB with dynamic PGA
	sensor->data[2] = 0xE3;                        // config reg LSB (860 SPS)

	sensor->tx_complete = 0;
	ret = I2C_WriteNonBlocking(sensor->address, &sensor->data[0], 3);


	if (ret != 0 ) 	// communication error occured
	{
//		sensor->err_cnt++;
		sensor->tx_complete = 1;
		sensor->rx_complete = 1;
	}
	else sensor->curr_channel = channel;

	return ret;
}

