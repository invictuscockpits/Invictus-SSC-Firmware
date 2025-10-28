/**
  ******************************************************************************
  * @file           : config.c
  * @brief          : Config management implementation
		
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

#include "config.h"

app_config_t app_config;

uint8_t DevConfigCheck (dev_config_t * p_dev_config)
{
	uint8_t ret = 0;
	
	if (p_dev_config == NULL)	ret = 1;
	
	return ret;
}

void DevConfigSet (dev_config_t * p_dev_config)
{
	uint32_t data_addr = (uint32_t) p_dev_config;
	uint32_t prog_addr;
	
	if (DevConfigCheck(p_dev_config) != 0)
		return;

	prog_addr = CONFIG_ADDR;
	
	FLASH_Unlock();
	FLASH_ErasePage(prog_addr);
	
	for (int i=0; i<sizeof(dev_config_t); i+=4)
	{
		FLASH_ProgramWord(prog_addr+i, *(uint32_t *)(data_addr + i)); 

	}
	FLASH_Lock();
}

void DevConfigGet (dev_config_t * p_dev_config)
{
	uint32_t read_addr;
	uint32_t data_addr = (uint32_t) p_dev_config;
	
	if (p_dev_config == NULL)
		return;
	
	//read_addr = FLASH_BASE + (*(uint16_t *)FLASHSIZE_BASE - 1)*1024;		// last page
	read_addr = CONFIG_ADDR;
	for (int i=0; i<sizeof(dev_config_t); i+=4)
	{
		*(uint32_t *)(data_addr + i) = *(uint32_t *) (read_addr+i);
	}
	
}

void AppConfigInit (dev_config_t * p_dev_config)
{
	int8_t prev_a = -1;
	int8_t prev_b = -1;
	
	app_config.axis = 0;
	app_config.axis_cnt = 0;
	app_config.buttons_cnt = 0;
	app_config.pov = 0;
	app_config.pov_cnt = 0;
	
	for (uint8_t i=0; i<MAX_AXIS_NUM; i++)
	{
		if (p_dev_config->axis_config[i].out_enabled)	
		{
			app_config.axis |= (1<<i);
			app_config.axis_cnt++;
		}
	}

	for (uint8_t i=0; i<MAX_BUTTONS_NUM; i++)
	{
	
		if (p_dev_config->buttons[i].type == POV1_DOWN ||
					p_dev_config->buttons[i].type == POV1_UP ||
					p_dev_config->buttons[i].type == POV1_LEFT ||
					p_dev_config->buttons[i].type == POV1_RIGHT)
		{
			p_dev_config->buttons[i].is_disabled = 1;
			app_config.pov |= 0x01;
		}
		else if (p_dev_config->buttons[i].type == POV2_DOWN ||
					p_dev_config->buttons[i].type == POV2_UP ||
					p_dev_config->buttons[i].type == POV2_LEFT ||
					p_dev_config->buttons[i].type == POV2_RIGHT)
		{
			p_dev_config->buttons[i].is_disabled = 1;
			app_config.pov |= 0x02;
		}
		else if (p_dev_config->buttons[i].type == POV3_DOWN ||
					p_dev_config->buttons[i].type == POV3_UP ||
					p_dev_config->buttons[i].type == POV3_LEFT ||
					p_dev_config->buttons[i].type == POV3_RIGHT)
		{
			p_dev_config->buttons[i].is_disabled = 1;
			app_config.pov |= 0x04;
		}
		else if (p_dev_config->buttons[i].type == POV4_DOWN ||
					p_dev_config->buttons[i].type == POV4_UP ||
					p_dev_config->buttons[i].type == POV4_LEFT ||
					p_dev_config->buttons[i].type == POV4_RIGHT)
		{
			p_dev_config->buttons[i].is_disabled = 1;
			app_config.pov |= 0x08;
		}
		
		app_config.pov_cnt = ((app_config.pov & 0x08)>>3) + ((app_config.pov & 0x04)>>2) + 
												 ((app_config.pov & 0x02)>>1) + (app_config.pov & 0x01);
		
		if (!p_dev_config->buttons[i].is_disabled && p_dev_config->buttons[i].physical_num >=0)
		{
			app_config.buttons_cnt++;
		}
	}
}

void AppConfigGet (app_config_t * p_app_config)
{
	memcpy(p_app_config, &app_config, sizeof(app_config_t));
}

uint8_t IsAppConfigEmpty (app_config_t * p_app_config)
{
	if (p_app_config->axis_cnt == 0 && p_app_config->buttons_cnt == 0 &&
		  p_app_config->pov_cnt == 0) return 1;
	return 0;
}

