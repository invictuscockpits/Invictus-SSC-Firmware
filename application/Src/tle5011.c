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

#include "tle5011.h"
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

void TLE5011_Read(uint8_t * data, uint8_t addr, uint8_t length)
{
	cmd = 0x80 | (addr & 0x0F)<<3 | (length & 0x07);
	
	SPI_HalfDuplex_Transmit(&cmd, 1, TLE5011_SPI_MODE);
	if (length > 0)
	{
		SPI_HalfDuplex_Receive(data, length+1, TLE5011_SPI_MODE);
	}

}

void TLE5011_Write(uint8_t * data, uint8_t addr, uint8_t length)
{
	cmd = addr<<3 | (addr & 0x0F)<<3 | (length & 0x07);
	SPI_HalfDuplex_Transmit(&cmd, 1, TLE5011_SPI_MODE);
	if (length > 0)
	{
		SPI_HalfDuplex_Transmit(data, length, TLE5011_SPI_MODE);
	}
}

int TLE5011_GetAngle(sensor_t * sensor, float * angle)
{
	int16_t x_value, y_value;
	float out = 0;
	int ret = -1;
	
	
	
	if (CheckCrc(&sensor->data[2], sensor->data[6], 0xFB, 4))
	{
		x_value = sensor->data[3]<<8 | sensor->data[2];
		y_value = sensor->data[5]<<8 | sensor->data[4];
		
		if((x_value != 32767) && (x_value != -32768) && (x_value != 0) &&
			 (y_value != 32767) && (y_value != -32768) && (y_value != 0))
		{				
			out = atan2f((float)y_value, (float)x_value)/ M_PI * (float)180.0;			
			*angle = out;
			ret = 0;
		}
	}
	return ret;
}

void TLE5011_StartDMA(sensor_t * sensor)
{	
	sensor->rx_complete = 1;
	sensor->tx_complete = 0;
	
	// switch MOSI to open-drain
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;						
	GPIO_Init (GPIOB,&GPIO_InitStructure);
	
	// CS low
	pin_config[sensor->source].port->ODR &= ~pin_config[sensor->source].pin;
	sensor->data[0] = 0x00;
	sensor->data[1] = 0x8C;
	
	SPI_HalfDuplex_Transmit(&sensor->data[0], 2, TLE5011_SPI_MODE);
}

void TLE5011_StopDMA(sensor_t * sensor)
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



