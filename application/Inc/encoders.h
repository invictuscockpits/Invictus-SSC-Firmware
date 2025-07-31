/**
  ******************************************************************************
  * @file           : encoders.h
  * @brief          : Header for encoders.c file.
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
#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include "common_types.h"
#include "periphery.h"
#include "buttons.h"

extern encoder_state_t encoders_state[MAX_ENCODERS_NUM];

void EncoderProcess (logical_buttons_state_t * button_state_buf, dev_config_t * p_dev_config);
void EncodersInit (dev_config_t * p_dev_config);

#endif 	/* __BUTTONS_H__ */

