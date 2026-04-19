
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	
	* @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.1.0
  * @date           2025-10-25
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "periphery.h"
#include "config.h"
#include "analog.h"
#include "buttons.h"


#include "usb_hw.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "usb_desc.h"
#include "device_info.h"


/* Private variables ---------------------------------------------------------*/
dev_config_t dev_config;
volatile uint8_t bootloader = 0;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{	
	// Relocate vector table
	WRITE_REG(SCB->VTOR, 0x8002000);
	
	SysTick_Init();
	
	// getting configuration from flash memory
	DevConfigGet(&dev_config);
	// set default config at first startup
	if ((dev_config.firmware_version & 0xFFF0) != (FIRMWARE_VERSION &0xFFF0))
	{
	    DevConfigSet((dev_config_t *) &init_config);
	    DevConfigGet(&dev_config);
	}

	AppConfigInit(&dev_config);

	// Auto-recovery safety net: if the loaded config produces zero enabled
	// axes AND zero enabled buttons AND zero POVs, the HID descriptor will
	// be effectively empty and the device won't show any inputs in joy.cpl.
	// This can happen if a previous configurator write disabled everything
	// or if the flash became inconsistent. Force-reload init_config to
	// recover a usable state.
	{
		app_config_t tmp_app_config;
		AppConfigGet(&tmp_app_config);
		if (tmp_app_config.axis_cnt == 0 &&
		    tmp_app_config.buttons_cnt == 0 &&
		    tmp_app_config.pov_cnt == 0)
		{
			DevConfigSet((dev_config_t *) &init_config);
			DevConfigGet(&dev_config);
			AppConfigInit(&dev_config);
		}
	}
	
	// Initialize device info BEFORE USB so product string reflects the
	// per-device name (e.g. "Invictus VFT Sensor") on first enumeration.
	device_info_init();
	USB_UpdateProductString(g_device_info.device_name);

	USB_HW_Init();
	// wait for USB initialization
	Delay_ms(1000);

	IO_Init(&dev_config);

	ShiftRegistersInit(&dev_config);
	RadioButtons_Init(&dev_config);
	SequentialButtons_Init(&dev_config);

	// init sensors
	AxesInit(&dev_config);
	// start sequential periphery reading
	Timers_Init(&dev_config);

  while (1)
  {		
		ButtonsDebounceProcess(&dev_config);
		ButtonsReadLogical(&dev_config);

		analog_data_t tmp[8];
		AnalogGet(NULL, tmp, NULL);
		PWM_SetFromAxis(&dev_config, tmp);
		
		// Enter flasher command received
		if (bootloader > 0)
		{
			// Disable HID report generation
			NVIC_DisableIRQ(TIM2_IRQn);
			Delay_ms(50);	// time to let HID end last transmission
			// Disable USB
			PowerOff();
			USB_HW_DeInit();
			Delay_ms(500);	
			EnterBootloader();
		}
  }
}

/**
  * @brief  Jumping to memory address corresponding bootloader program
  * @param  None
  * @retval None
  */
void EnterBootloader (void)
{
	/* Enable the power and backup interface clocks by setting the
	 * PWREN and BKPEN bits in the RCC_APB1ENR register
	 */
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);

	/* Enable write access to the backup registers and the
		* RTC.
		*/
	SET_BIT(PWR->CR, PWR_CR_DBP);
	WRITE_REG(BKP->DR4, 0x424C);
	CLEAR_BIT(PWR->CR, PWR_CR_DBP);
	
	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
	
	NVIC_SystemReset();
}


/**
  * @}
  */


