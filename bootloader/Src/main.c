/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main body program of Invictus Bootloader
 *
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

/* Includes ------------------------------------------------------------------*/

#include "periphery.h"
#include "usb_hw.h"

uint16_t magic_word;
uint16_t boot1;
uint16_t checkUserCode;

/* Private types */
typedef void (*funct_ptr)(void);
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static bool CheckUserCode(uint32_t user_address);

static uint16_t GetMagicWord(void);

static void EnterProgram(void);

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void) 
{
    IO_Init();
    Delay(100);

    magic_word = GetMagicWord();
    boot1 = READ_BIT(GPIOB->IDR, GPIO_IDR_IDR2);
    checkUserCode = CheckUserCode(FIRMWARE_COPY_ADDR);

    if ((magic_word == 0x424C) || boot1 || checkUserCode == 0) 
		{
        USB_HW_Init();
    } 
		else 
		{
        EnterProgram();
        // Never reached
        while (1);
    }

    while (1) 
		{
        if (!flash_started) 
				{
            LED1_ON;
            Delay(500000);
            LED1_OFF;
            Delay(10000000);
        }
        if (flash_finished) 
				{
            Delay(100000);
            USB_Shutdown();
            Delay(1000000);
            EnterProgram();
        }
    }
}

static bool CheckUserCode(uint32_t user_address) 
{
    uint32_t sp = *(volatile uint32_t *) user_address;

    /* Check if the stack pointer in the vector table points
       somewhere in SRAM */
    return ((sp & 0x2FFE0000) == SRAM_BASE) ? 1 : 0;
}

static uint16_t GetMagicWord(void) 
{
    /* Enable the power and backup interface clocks by setting the
     * PWREN and BKPEN bits in the RCC_APB1ENR register
     */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
    uint16_t value = READ_REG(BKP->DR4);
    if (value) 
		{

        /* Enable write access to the backup registers and the
         * RTC.
         */
        SET_BIT(PWR->CR, PWR_CR_DBP);
        WRITE_REG(BKP->DR4, 0x0000);
        CLEAR_BIT(PWR->CR, PWR_CR_DBP);
    }
    CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);

    return value;
}

/**
  * @brief  Jumping to memory address corresponding main program
  * @param  None
  * @retval None
  */
static void EnterProgram(void) 
{
    funct_ptr Program = (funct_ptr) *(volatile uint32_t *) (FIRMWARE_COPY_ADDR + 0x04);

    /* Setup the vector table to the final user-defined one in Flash
     * memory
     */
    WRITE_REG(SCB->VTOR, FIRMWARE_COPY_ADDR);
	
    /*
     * Setup the stack pointer to the user-defined one
     */
		//__set_MSP((*(volatile uint32_t *) FIRMWARE_COPY_ADDR));
		__ASM volatile ("MSR msp, %0" : : "r" ((*(volatile uint32_t *) FIRMWARE_COPY_ADDR)) : );
	
    // Use asm so stack is not used for branch
    __ASM volatile("bx %0\n\t"
    :
    : "r" (Program)
    :
    );
}
