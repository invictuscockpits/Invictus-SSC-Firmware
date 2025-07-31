/**
  ******************************************************************************
  * @file           : as5600.c
  * @brief          : AS5600 magnetic encoder driver implementation
			
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

#include "as5600.h"

/**
  * @brief AS5600 init fuction
  * @param sensor: Sensor struct
	* @param min: minimum calibrated value
	* @param max: maximum calibrated value
  * @retval None
  */
void AS5600_Init(sensor_t * sensor, uint16_t min, uint16_t max)
{
//	int status;
//	uint8_t tmp_buf[2];

//	tmp_buf[0] = min >> 8;
//	tmp_buf[1] = min & 0x0F;
//	status = I2C_WriteBlocking(sensor->address, 0x01, tmp_buf, 2);
//	
//	Delay_ms(2);
//	
//	tmp_buf[0] = max >> 8;
//	tmp_buf[1] = max & 0x0F;
//	status = I2C_WriteBlocking(sensor->address, 0x03, tmp_buf, 2);
//	
//	Delay_ms(2);
}


/**
  * @brief AS5600 get raw measured data
  * @param sensor: Sensor struct
  * @retval data
  */
int16_t AS5600_GetRawData(sensor_t * sensor)
{
	return sensor->data[0]<<8|sensor->data[1];
}

/**
  * @brief AS5600 get scaled measured data
  * @param sensor: Sensor struct
  * @retval data
  */
int16_t AS5600_GetScaledData(sensor_t * sensor)
{
	return sensor->data[2]<<8|sensor->data[3];
}

/**
  * @brief AS5600 start processing data in blocking mode
  * @param sensor: Sensor struct
  * @retval status
  */
int AS5600_ReadBlocking(sensor_t * sensor)
{
	int ret;	
	uint8_t tmp_buf[4];
	
	ret = I2C_ReadBlocking(sensor->address, 0x0C, tmp_buf, 4, 1);
	
	if (ret == 0 ) 
	{
		memcpy(&sensor->data[0], tmp_buf, 4);
		sensor->ok_cnt++;
	}
	else sensor->err_cnt++;
	
	return ret;
}

/**
  * @brief AS5600 start processing data in DMA mode
  * @param sensor: Sensor struct
  * @retval status
  */
int AS5600_StartDMA(sensor_t * sensor)
{
	int ret;	
	
	sensor->rx_complete = 0;	
	ret = I2C_ReadNonBlocking(sensor->address, 0x0C, &sensor->data[0], 4, 1);
	
	if (ret != 0 )
	{
		sensor->err_cnt++;
		sensor->rx_complete = 1;
	}
	
	return ret;
}


