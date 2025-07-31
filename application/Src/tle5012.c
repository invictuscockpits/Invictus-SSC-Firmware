/**
  ******************************************************************************
  * @file           : tle5011.c
  * @brief          : TLE5011 sensors driver implementation
			
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
	
#include "tle5012.h"
#include <math.h>

static uint8_t cmd;

static uint8_t MathCRC8(uint8_t crc, uint8_t data)
{
	crc ^= data;
	for (uint8_t bit=0 ; bit<8 ; bit++ ) 
	{ 
		if ((crc & 0x80)!=0) 
		{ 
			crc <<= 1; 
			crc ^= 0x1D; 
		} else 
		{ 
			crc <<= 1; 
		}; 
	};
	return(crc);
};

static uint8_t CheckCrc(uint8_t * data, uint8_t crc, uint8_t initial, uint8_t length) 
{
  uint8_t ret = initial;
	uint8_t index = 0;
	
	while (index < length)
	{
		ret = MathCRC8(ret, data[index++]);
	}        
  ret = ~ret;
	
  return (ret == crc);
}

void TLE5012_Read(uint8_t * data, uint8_t addr, uint8_t length)
{
	cmd = 0x80 | (addr & 0x0F)<<3 | (length & 0x07);
	
	SPI_HalfDuplex_Transmit(&cmd, 1, TLE5012_SPI_MODE);
	if (length > 0)
	{
		SPI_HalfDuplex_Receive(data, length+1, TLE5012_SPI_MODE);
	}

}

void TLE5012_Write(uint8_t * data, uint8_t addr, uint8_t length)
{
	cmd = addr<<3 | (addr & 0x0F)<<3 | (length & 0x07);
	SPI_HalfDuplex_Transmit(&cmd, 1, TLE5012_SPI_MODE);
	if (length > 0)
	{
		SPI_HalfDuplex_Transmit(data, length, TLE5012_SPI_MODE);
	}
}

int TLE5012_GetAngle(sensor_t * sensor, float * angle)
{
	int16_t reg_data;
	float out = 0;
	int ret = 0;
	
	if (CheckCrc(&sensor->data[0], sensor->data[5], 0xFF, 4))
	{
		reg_data = (sensor->data[2] & 0x3F)<<8 | sensor->data[3];
		if (sensor->data[2] & 0x40)  reg_data -= 16384;
				
		out = reg_data * 360.0f / 32768.0f;			
		*angle = out;
		ret = 0;
	}
	else
	{
		ret = -1;
	}
	return ret;
}

void TLE5012_StartDMA(sensor_t * sensor)
{	
	sensor->rx_complete = 1;
	sensor->tx_complete = 0;
	// CS low
	pin_config[sensor->source].port->ODR &= ~pin_config[sensor->source].pin;
	sensor->data[0] = 0x80;
	sensor->data[1] = 0x21;
	
	SPI_HalfDuplex_Transmit(&sensor->data[0], 2, TLE5012_SPI_MODE);
}

void TLE5012_StopDMA(sensor_t * sensor)
{	
	DMA_Cmd(DMA1_Channel2, DISABLE);
	
	
	
	// CS high	
	pin_config[sensor->source].port->ODR |= pin_config[sensor->source].pin;
	sensor->rx_complete = 1;
	sensor->tx_complete = 1;
	
	SPI_BiDirectionalLineConfig(SPI1, SPI_Direction_Tx);
	
	Delay_us(5);	// wait SPI clocks to stop
	
	// switch MOSI back to push-pull
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;						
	GPIO_Init (GPIOB,&GPIO_InitStructure);	
}



