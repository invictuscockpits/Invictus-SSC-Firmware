/**
  ******************************************************************************
  * @file           : leds.c
  * @brief          : LEDs driver implementation
		
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
	
#include "leds.h"
#include "buttons.h"
	
uint8_t leds_state[MAX_LEDS_NUM];
	
void LEDs_LogicalProcess (dev_config_t * p_dev_config)
{
	for (uint8_t i=0; i<MAX_LEDS_NUM; i++)
	{
		if (p_dev_config->leds[i].input_num >= 0)
		{
			switch (p_dev_config->leds[i].type)
			{
				default:
				case LED_NORMAL:
					leds_state[i] = logical_buttons_state[p_dev_config->leds[i].input_num].current_state;
				break;
				
				case LED_INVERTED:
					leds_state[i] = !logical_buttons_state[p_dev_config->leds[i].input_num].current_state;
				break;
				
			}
		}
	}
}

void LED_SetSingle(uint8_t * state_buf, dev_config_t * p_dev_config, uint8_t * pos)
{
	for (uint8_t i=0; i<USED_PINS_NUM; i++)
	{
		if (p_dev_config->pins[i] == LED_SINGLE)
		{
			leds_state[*pos] ? (pin_config[i].port->ODR |= pin_config[i].pin) : (pin_config[i].port->ODR &= ~pin_config[i].pin); 
			(*pos)++;
		}
	}
}

void LED_SetMatrix(uint8_t * state_buf, dev_config_t * p_dev_config, uint8_t * pos)
{
	static int8_t last_row = -1;
	static uint8_t last_pos = 0;
	int8_t max_row = -1;
	
	for (uint8_t i=0; i<USED_PINS_NUM; i++)
	{
		// turn off leds
		if (p_dev_config->pins[i] == LED_ROW)
		{
			pin_config[i].port->ODR &= ~pin_config[i].pin;
			max_row = i;
			for (uint8_t j=0; j<USED_PINS_NUM; j++)
			{
				if (p_dev_config->pins[j] == LED_COLUMN)
				{
					pin_config[j].port->ODR |= pin_config[j].pin;
					(*pos)++;
				}
			}
		}
	}
	for (uint8_t i=0; i<USED_PINS_NUM; i++)
	{
		if (p_dev_config->pins[i] == LED_ROW && (i > last_row || i == max_row))
		{
			pin_config[i].port->ODR |= pin_config[i].pin;
			for (uint8_t j=0; j<USED_PINS_NUM; j++)
			{
				if (p_dev_config->pins[j] == LED_COLUMN)
				{
					if (leds_state[last_pos++] > 0) 
					{
						pin_config[j].port->ODR &= ~pin_config[j].pin;
					}
					else
					{
						pin_config[j].port->ODR |= pin_config[j].pin; 
					}				
				}
			}
			if (last_pos >= *pos) last_pos = 0;
			(i == max_row) ? (last_row = -1): (last_row = i);
			break;
		}
		
	}
}

void LEDs_PhysicalProcess (dev_config_t * p_dev_config)
{
	uint8_t pos = 0;
	
	LEDs_LogicalProcess(p_dev_config);
	
	LED_SetMatrix(leds_state, p_dev_config, &pos);
	LED_SetSingle(leds_state, p_dev_config, &pos);
		
}


