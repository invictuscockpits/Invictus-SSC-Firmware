/**
  ******************************************************************************
  * @file           : periphery.h
  * @brief          : Header for periphery.c file.
  *                   This file contains the common defines of the periphery.
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
#ifndef __PERIPHERY_H__
#define __PERIPHERY_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "common_types.h"

#include "spi.h"
#include "i2c.h"

#include "string.h"

#define MAX_TIMERS_NUM					10

#define TICKS_IN_MILLISECOND		2

extern volatile uint64_t Ticks;
extern volatile uint32_t TimingDelay;


typedef struct 
{
	GPIO_TypeDef * 	port;
	uint16_t				pin;
	uint8_t 				number;
	
}	pin_config_t;

extern pin_config_t pin_config[USED_PINS_NUM];

void SysTick_Init(void);
void Timers_Init(dev_config_t * p_dev_config);
void PWM_SetFromAxis(dev_config_t * p_dev_config, analog_data_t * axis_data);

uint64_t GetMillis(void);

void Delay_ms(__IO uint32_t nTime);
void Delay_us(__IO uint32_t nTime);

void IO_Init (dev_config_t * p_dev_config);




#endif 	/* __PERIPHERY_H__ */
