/**
  ******************************************************************************
  * @file           : periphery.c
  * @brief          : Periphery driver implementation
	
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

#include "periphery.h"
#include "stm32f10x_tim.h"

/* define compiler specific symbols */
#if defined ( __CC_ARM   )
#define __ASM            __asm                                      /*!< asm keyword for ARM Compiler          */
#define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler       */

#elif defined ( __ICCARM__ )
#define __ASM           __asm                                       /*!< asm keyword for IAR Compiler          */
#define __INLINE        inline                                      /*!< inline keyword for IAR Compiler. Only avaiable in High optimization mode! */

#elif defined   (  __GNUC__  )
#define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
#define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */

#elif defined   (  __TASKING__  )
#define __ASM            __asm                                      /*!< asm keyword for TASKING Compiler      */
#define __INLINE         inline                                     /*!< inline keyword for TASKING Compiler   */

#endif


volatile uint64_t Ticks;
volatile uint32_t TimingDelay;

pin_config_t pin_config[USED_PINS_NUM] =
        {
                {GPIOA, GPIO_Pin_0,  0},                    // 0
                {GPIOA, GPIO_Pin_1,  1},                    // 1
                {GPIOA, GPIO_Pin_2,  2},                    // 2
                {GPIOA, GPIO_Pin_3,  3},                    // 3
                {GPIOA, GPIO_Pin_4,  4},                    // 4
                {GPIOA, GPIO_Pin_5,  5},                    // 5
                {GPIOA, GPIO_Pin_6,  6},                    // 6
                {GPIOA, GPIO_Pin_7,  7},                    // 7
                {GPIOA, GPIO_Pin_8,  8},                    // 8
                {GPIOA, GPIO_Pin_9,  9},                    // 9
                {GPIOA, GPIO_Pin_10, 10},                // 10
                {GPIOA, GPIO_Pin_15, 15},                // 11
                {GPIOB, GPIO_Pin_0,  0},                    // 12
                {GPIOB, GPIO_Pin_1,  1},                    // 13
                {GPIOB, GPIO_Pin_3,  3},                    // 14
                {GPIOB, GPIO_Pin_4,  4},                    // 15
                {GPIOB, GPIO_Pin_5,  5},                    // 16
                {GPIOB, GPIO_Pin_6,  6},                    // 17
                {GPIOB, GPIO_Pin_7,  7},                    // 18
                {GPIOB, GPIO_Pin_8,  8},                    // 19
                {GPIOB, GPIO_Pin_9,  9},                    // 20
                {GPIOB, GPIO_Pin_10, 10},                // 21
                {GPIOB, GPIO_Pin_11, 11},                // 22
                {GPIOB, GPIO_Pin_12, 12},                // 23
                {GPIOB, GPIO_Pin_13, 13},                // 24
                {GPIOB, GPIO_Pin_14, 14},                // 25
                {GPIOB, GPIO_Pin_15, 15},                // 26
                {GPIOC, GPIO_Pin_13, 13},                // 27
                {GPIOC, GPIO_Pin_14, 14},                // 28
                {GPIOC, GPIO_Pin_15, 15},                // 29
        };


/**
  * @brief SysTick Configuration
  * @retval None
  */
void SysTick_Init(void) {
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.SYSCLK_Frequency / 1000);
}

/**
  * @brief Timers Configuration
  * @retval None
  */
void Timers_Init(dev_config_t * p_dev_config)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	RCC_ClocksTypeDef RCC_Clocks;
	
	RCC_GetClocksFreq(&RCC_Clocks);	
	
	// Reset tick counter
	Ticks = 0;
	
	// Axis and HID timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);	
	TIM_TimeBaseInitStructure.TIM_Prescaler = RCC_Clocks.PCLK1_Frequency/100000 - 1;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;			// 2000Hz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	
	NVIC_SetPriority(TIM2_IRQn, 3);
	NVIC_EnableIRQ(TIM2_IRQn);

	TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief Update PWM values
	* @param p_dev_config: Pointer to device config
	* @param axis_data: Pointer to axis values
  * @retval None
  */
void PWM_SetFromAxis(dev_config_t * p_dev_config, analog_data_t * axis_data)
{
	// LED PWM support removed
}


/**
  * @brief Get up-time milliseconds
  * @retval milliseconds
  */
uint64_t GetMillis(void) 
{
    return Ticks/(TICKS_IN_MILLISECOND);
}


/**
  * @brief Delay implementation
  * @retval None
  */
void Delay_ms(uint32_t nTime) 
{
    TimingDelay = nTime;
    while (TimingDelay != 0);
}

/**
  * @brief Delay implementation
  * @retval None
  */
void Delay_us(uint32_t nTime) 
{
    int32_t us = nTime * 5;

    while (us > 0) {
        us--;
    }
}

/**
  * @brief Generator Initialization Function
  * @param None
  * @retval None
  */
void Generator_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructureure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 18 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 9;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    /*GPIOB Configuration: TIM4 channel1*/
    GPIO_InitStructureure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructureure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructureure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructureure);

    /* TIM4 enable counter */
    TIM_Cmd(TIM4, ENABLE);
}

/* IO init function */
void IO_Init (dev_config_t * p_dev_config)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Remapping
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	
  // GPIO Ports Clock Enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	while ((p_dev_config->firmware_version & 0xFFF0) != (FIRMWARE_VERSION & 0xFFF0))
	{
		// blink LED if firmware version doesnt match
		GPIOB->ODR ^= GPIO_Pin_12;
		GPIOC->ODR ^=	GPIO_Pin_13;
		Delay_ms(300);
	}
	
	// Reset GPIO (preserve USB pins PA11/PA12)
	GPIOA->CRL=0x44444444;
	GPIOA->CRH=0x44BB4444;  // 0xB preserves PA11/PA12 as AF output for USB
	GPIOA->ODR=0x0;
	GPIOB->CRL=0x44444444;
	GPIOB->CRH=0x44444444;
	GPIOB->ODR=0x0;
	GPIOC->CRL=0x44444444;
	GPIOC->CRH=0x44444444;
	GPIOC->ODR=0x0;
	

	// setting up GPIO according confgiguration
	for (int i=0; i<USED_PINS_NUM; i++)
	{
		// buttons
		if (p_dev_config->pins[i] == BUTTON_GND)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
		}
		else if (p_dev_config->pins[i] == BUTTON_VCC)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
		}
		else if (p_dev_config->pins[i] == BUTTON_COLUMN)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
		}
		else if (p_dev_config->pins[i] == BUTTON_ROW)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
		}		
		else if (p_dev_config->pins[i] == AXIS_ANALOG)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
		}
		else if (p_dev_config->pins[i] == SPI_SCK && i == 14)		// PB3
		{
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
			GPIO_Init (GPIOB,&GPIO_InitStructure);			
		}
		else if (p_dev_config->pins[i] == SPI_MISO && i == 15)			// PB4
		{		
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init (GPIOB,&GPIO_InitStructure);
		}
		else if (p_dev_config->pins[i] == SPI_MOSI && i == 16)			// PB5
		{		
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;						// PP or OD?
			GPIO_Init (GPIOB,&GPIO_InitStructure);

			SPI_Start();
		}
		else if (p_dev_config->pins[i] == I2C_SCL && i == 21)			// PB10
		{
			I2C_Start();
			
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
			GPIO_Init (GPIOB,&GPIO_InitStructure);
		}
		else if (p_dev_config->pins[i] == I2C_SDA && i == 22)			// PB11
		{		
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;						
			GPIO_Init (GPIOB,&GPIO_InitStructure);
		}
		else if (p_dev_config->pins[i] == MCP3202_CS)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
			GPIO_WriteBit(pin_config[i].port, pin_config[i].pin, Bit_SET);
		}
		else if (p_dev_config->pins[i] == SHIFT_REG_CLK)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
			GPIO_WriteBit(pin_config[i].port, pin_config[i].pin, Bit_RESET);
		}
		else if (p_dev_config->pins[i] == SHIFT_REG_LATCH)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
			GPIO_WriteBit(pin_config[i].port, pin_config[i].pin, Bit_SET);
		}
		else if (p_dev_config->pins[i] == SHIFT_REG_DATA)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
		}
		else if (p_dev_config->pins[i] == NOT_USED)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_InitStructure.GPIO_Pin = pin_config[i].pin;
			GPIO_Init(pin_config[i].port, &GPIO_InitStructure);
		}
		
	}



#ifdef DEBUG
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
}


