/**
  ******************************************************************************
  * @file           : common_defines.h
  * @brief          : This file contains the common defines for the app.                  
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

/**
  ******************************************************************************
  * @file           : common_defines.h
  * @brief          : This file contains the common defines for the app.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_DEFINES_H__
#define __COMMON_DEFINES_H__

//#define DEBUG

#define FIRMWARE_VERSION					0x2121              // v2.1.2 (Fixed firmware flasher)
#define USED_PINS_NUM						30					// Contstant for HOTAS Control boards
#define MAX_AXIS_NUM						8					// max 8
#define MAX_BUTTONS_NUM						128					// power of 2, max 128
#define MAX_POVS_NUM						4					// max 4
#define MAX_ENCODERS_NUM					16					// max 64
#define MAX_SHIFT_REG_NUM					4					// max 4
#define MAX_LEDS_NUM						24

#define AXIS_MIN_VALUE						(-32767)
#define AXIS_MAX_VALUE						(32767)
#define AXIS_CENTER_VALUE					(AXIS_MIN_VALUE + (AXIS_MAX_VALUE-AXIS_MIN_VALUE)/2)
#define AXIS_FULLSCALE						(AXIS_MAX_VALUE - AXIS_MIN_VALUE + 1)

#define CONFIG_ADDR													(0x0800FC00)
#define FLASH_PAGE_SIZE                     0x400
#define FACTORY_ADDR                        (CONFIG_ADDR - FLASH_PAGE_SIZE)  // protected page
#define FACTORY_MAGIC                       0xF00C
#define FACTORY_VERSION                     0x02 //increase with major changes to protect older versions

#define DEVICE_INFO_OFFSET                  128  // 128 bytes offset (leaving room for force anchors to grow to 128 bytes)
#define DEVICE_INFO_ADDR                    (FACTORY_ADDR + DEVICE_INFO_OFFSET)
#define DEVICE_INFO_MAGIC                   0xDEF0


#define INV_SERIAL_MAX_LEN  16
#define INV_MODEL_MAX_LEN   16
#define DOM_ASCII_LEN       10 


enum
{
    REPORT_ID_JOY = 1,
    REPORT_ID_PARAM,
    REPORT_ID_CONFIG_IN,
    REPORT_ID_CONFIG_OUT,
    REPORT_ID_FIRMWARE,
    REPORT_ID_DEV = 6,   //For force anchors
};

enum {
    OP_GET_FACTORY_ANCHORS  = 1,
    OP_SET_FACTORY_ANCHORS  = 2,
    OP_LOCK_FACTORY_ANCHORS = 3,
    //OP_UNLOCK_FACTORY_ANCHORS = 4,
    // New device info operations
    OP_GET_DEVICE_INFO = 5,
    OP_SET_DEVICE_INFO = 6,
};



#endif 	/* __COMMON_DEFINES_H__ */
