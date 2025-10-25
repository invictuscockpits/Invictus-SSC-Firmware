
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	
	* @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.1.0
  * @date           2025-10-25
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "periphery.h"
#include "config.h"
#include "analog.h"
#include "buttons.h"
#include "leds.h"
#include "encoders.h"


#include "usb_hw.h"
#include "usb_lib.h"
#include "usb_pwr.h"


/* Private variables ---------------------------------------------------------*/
dev_config_t dev_config;
volatile uint8_t bootloader = 0;

/* Private function prototypes -----------------------------------------------*/
void device_info_init(void);
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
	
	USB_HW_Init();
	// wait for USB initialization
	Delay_ms(1000);	
	
	IO_Init(&dev_config);
	 
	EncodersInit(&dev_config);	
	ShiftRegistersInit(&dev_config);
	RadioButtons_Init(&dev_config);
	SequentialButtons_Init(&dev_config);

	// init sensors
	AxesInit(&dev_config);
	// start sequential periphery reading
	Timers_Init(&dev_config);		
	
	// Initialize device info from flash
	device_info_init();
	
  while (1)
  {		
		ButtonsDebounceProcess(&dev_config);
		ButtonsReadLogical(&dev_config);
		
		LEDs_PhysicalProcess(&dev_config);
		
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


