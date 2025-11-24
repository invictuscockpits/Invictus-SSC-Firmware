/**
  ******************************************************************************
  * @file           : analog.h
  * @brief          : Header for analog.c file.
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.0.0
  * @date           2025-07-30
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
	
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANALOG_H__
#define __ANALOG_H__

#include "common_types.h"
#include "periphery.h"

#define FILTER_BUF_SIZE									30
#define DEADBAND_BUF_SIZE								16					//Freejoy default is 8.  Increased to 16 for strain gauges
//#define DEADBAND_HOLD_VALUE							2000			//Possible legacy from previous versions of software, but I cannot find it used anywhere

#define ADC_CONV_NUM										8

extern sensor_t sensors[MAX_AXIS_NUM];

typedef struct
{
	uint32_t 	channel;
	uint8_t 	number;
	
} adc_channel_config_t;



void AxesInit (dev_config_t * p_dev_config);
void ADC_Conversion (void);
void AxesProcess (dev_config_t * p_dev_config);
void AxisResetCalibration (dev_config_t * p_dev_config, uint8_t axis_num);
void AnalogGet (analog_data_t * out_data, analog_data_t * scaled_data, analog_data_t * raw_data);

#endif 	/* __ANALOG_H__ */

