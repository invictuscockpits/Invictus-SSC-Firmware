/**
  ******************************************************************************
  * @file           : ads1115.c
  * @brief          : ADS1115 ADC driver implementation
	* @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.1.1
  * @date           2025-11-24
  *
  * This file incorporates code from FreeJoy by Yury Vostrenkov (2020)
  * https://github.com/FreeJoy-Team/FreeJoy
  *
  * Licensed under the GNU General Public License v3.0 or later.
  * https://www.gnu.org/licenses/gpl-3.0.html
  *
  * © 2025 Invictus Cockpit Systems. All modifications reserved.
  * This firmware is designed exclusively for Invictus HOTAS hardware.
  *
  ******************************************************************************
  */

#include "ads1115.h"
#include "device_info.h"

/**
  * @brief Convert PGA gain value to ADS1115 configuration byte
  * @param pga_gain: PGA gain value (1, 2, 4, 8, or 16)
  * @retval PGA config byte for ADS1115 (bits 11-9 set, with OS=1, MODE=1, MUX=100)
  *
  * ADS1115 PGA bits (11-9):
  *   001: ±4.096V (gain 1)
  *   010: ±2.048V (gain 2)
  *   011: ±1.024V (gain 4)
  *   100: ±0.512V (gain 8)
  *   101: ±0.256V (gain 16)
  */
static uint8_t ADS1115_PGAGainToConfig(uint8_t pga_gain)
{
	uint8_t pga_bits;

	switch (pga_gain) {
		case 1:  pga_bits = 0x01; break;  // 001: ±4.096V
		case 2:  pga_bits = 0x02; break;  // 010: ±2.048V
		case 4:  pga_bits = 0x03; break;  // 011: ±1.024V
		case 8:  pga_bits = 0x04; break;  // 100: ±0.512V
		case 16: pga_bits = 0x05; break;  // 101: ±0.256V
		default: pga_bits = 0x03; break;  // Default to PGA 4 (±1.024V)
	}

	// Base config: OS=1, MUX=000, MODE=1 (single-shot)
	// 0b1000_0001 = 0x81, then OR with PGA bits shifted left by 1
	// MUX bits cleared (000) so caller can OR in the correct MUX config
	return 0x81 | (pga_bits << 1);
}

/**
  * @brief Get PGA configuration byte based on channel
  * @param channel: ADC channel (0-3)
  * @retval PGA config byte
  */
static uint8_t ADS1115_GetPGAConfig(uint8_t channel)
{
	if (channel > 3) channel = 0;  // Safety check
	return ADS1115_PGAGainToConfig(g_device_info.adc_pga[channel]);
}

/**
  * @brief Get MUX configuration based on channel and mode
  * @param channel: ADC channel (0-3)
  * @retval MUX config bits (upper 3 bits of config register)
  */
static uint8_t ADS1115_GetMUXConfig(uint8_t channel)
{
	if (channel > 3) channel = 0;  // Safety check

	uint8_t mode = g_device_info.adc_mode[channel];

	if (mode == 0) {
		// Single-ended mode: AINx vs GND
		// MUX: 100 (AIN0), 101 (AIN1), 110 (AIN2), 111 (AIN3)
		return 0x04 + channel;
	} else {
		// Differential mode: natural pairs (0-1, 2-3)
		// MUX: 000 (AIN0-AIN1), 011 (AIN2-AIN3)
		if (channel == 0 || channel == 1) {
			return 0x00;  // AIN0 - AIN1
		} else {
			return 0x03;  // AIN2 - AIN3
		}
	}
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
	uint8_t mux_config = ADS1115_GetMUXConfig(channel);

	tmp_buf[0] = pga_config | (mux_config << 4);  // config reg MSB with dynamic PGA and MUX
	tmp_buf[1] = 0xE3;                             // config reg LSB (860 SPS)

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
	uint8_t mux_config = ADS1115_GetMUXConfig(channel);

	sensor->data[0] = 0x01;                             // config reg address
	sensor->data[1] = pga_config | (mux_config << 4);  // config reg MSB with dynamic PGA and MUX
	sensor->data[2] = 0xE3;                             // config reg LSB (860 SPS)

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

