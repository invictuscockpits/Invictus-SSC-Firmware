/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @project        Invictus HOTAS Firmware
 * @author         Invictus Cockpit Systems
 * @version        1.2.1
 * @date           2025-10-27
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
 
  * @attention
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include "usb_istr.h"
#include "usb_lib.h"
#include "periphery.h"
#include "analog.h"
#include "buttons.h"
#include "mcp320x.h"
#include "ads1115.h"
#include "config.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define ADC_PERIOD_TICKS										4					// 1 tick = 500us
#define SENSORS_PERIOD_TICKS								4
#define BUTTONS_PERIOD_TICKS								1

/* Private variables ---------------------------------------------------------*/

static joy_report_t 			joy_report;
static params_report_t 	params_report;

volatile int32_t millis = 0;
volatile int32_t joy_millis = 0;
volatile int32_t adc_ticks = 0;
volatile int32_t sensors_ticks = 1;
volatile int32_t buttons_ticks = 0;
volatile int32_t configurator_millis = 0;
volatile int status = 0;
extern dev_config_t dev_config;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	if (TimingDelay != 0x00)										
  {
    TimingDelay--;
  }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/


void TIM2_IRQHandler(void)
{
	uint8_t						report_buf[64];
	uint8_t						pos = 0;
	app_config_t			tmp_app_config;
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		Ticks++;
		millis = GetMillis();
		
		
		
		// check if it is time to send joystick data
		if (millis - joy_millis >= dev_config.exchange_period_ms )
		{
			joy_millis = millis;

			AppConfigGet(&tmp_app_config);
				
			// getting fresh data to joystick report buffer
			ButtonsGet(joy_report.button_data, 
								 params_report.log_button_data, 
								 params_report.phy_button_data, 
								 &params_report.shift_button_data);
			AnalogGet(joy_report.axis_data, NULL, params_report.raw_axis_data);	
			POVsGet(joy_report.pov_data);
			
			// fill joystick report buffer
			report_buf[pos++] = REPORT_ID_JOY;			
			if (tmp_app_config.buttons_cnt > 0)
			{
				memcpy(&report_buf[pos], joy_report.button_data, MAX_BUTTONS_NUM/8);
				pos += (tmp_app_config.buttons_cnt - 1)/8 + 1;
			}
			for (uint8_t i=0; i<MAX_AXIS_NUM; i++)
			{
					if (tmp_app_config.axis & (1<<i))
					{
						report_buf[pos++] = (uint8_t) (joy_report.axis_data[i] & 0xFF);
						report_buf[pos++] = (uint8_t) (joy_report.axis_data[i] >> 8);							
					}
			}
			for (uint8_t i=0; i<MAX_POVS_NUM; i++)
			{
					if (tmp_app_config.pov & (1<<i))
					{
						report_buf[pos++] = joy_report.pov_data[i];
					}
			}
			// send joystick report
			USB_CUSTOM_HID_SendReport(1, report_buf, pos);
		
			// fill params report buffer
			if (configurator_millis > millis)
			{
				static uint8_t report = 0;
				report_buf[0] = REPORT_ID_PARAM;
				params_report.firmware_version = FIRMWARE_VERSION;
				memcpy(params_report.axis_data, joy_report.axis_data, sizeof(params_report.axis_data));
				
				if (report == 0)
				{
					report_buf[1] = 0;
					memcpy(&report_buf[2], (uint8_t *)&(params_report), 62);
				}
				else
				{
					report_buf[1] = 1;
					memcpy(&report_buf[2], (uint8_t *)&(params_report) + 62, sizeof(params_report_t) - 62);
				}
				
				// send params report
				if (USB_CUSTOM_HID_SendReport(2, report_buf, 64) == 0)
				{
					report = !report;
				}
			}
		}

		// digital inputs polling
		if (Ticks - buttons_ticks >= BUTTONS_PERIOD_TICKS)
		{
			buttons_ticks = Ticks;
			ButtonsReadPhysical(&dev_config, raw_buttons_data);
		}
		
		// Internal ADC conversion
		if (Ticks - adc_ticks >= ADC_PERIOD_TICKS)
		{		
			adc_ticks = Ticks;	

			AxesProcess(&dev_config);					// process axis only once for one data reading
			
			// Disable periphery before ADC conversion
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,DISABLE);
			// I2C2 NOT disabled to prevent ADS1115 DMA disruption
		  // GPIO NOT disabled to prevent ADS1115 DMA disruption

			// ADC measurement
			ADC_Conversion();

			// Enable periphery after ADC conversion
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

		}
		// External sensors data receiption
		if (Ticks - sensors_ticks >= SENSORS_PERIOD_TICKS && Ticks > adc_ticks +1)		// prevent ADC and sensors reading during same ms
		{																																						
			sensors_ticks = Ticks;

			// start SPI sensors 
			for (uint8_t i=0; i<MAX_AXIS_NUM; i++)
			{
				if (sensors[i].source >= 0 && sensors[i].tx_complete && sensors[i].rx_complete)
				{
					if (sensors[i].type == MCP3202)
					{
						MCP320x_StartDMA(&sensors[i], 0);
						break;
					}
				}
			}
			// start I2C sensors 
 			for (uint8_t i=0; i<MAX_AXIS_NUM; i++)
			{
				if (sensors[i].source == (pin_t)SOURCE_I2C && sensors[i].rx_complete && sensors[i].tx_complete)
				{		
					if (sensors[i].type == ADS1115)
					{
						status = ADS1115_StartDMA(&sensors[i], sensors[i].curr_channel);	
						if (status != 0) continue;
						else break;
					}
				}
			}
		}
		
	}
}

// SPI Rx Complete
void DMA1_Channel2_IRQHandler(void)
{
	uint8_t i=0;
	
	if (DMA_GetITStatus(DMA1_IT_TC2))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC2);
		DMA_Cmd(DMA1_Channel2, DISABLE);
		
		// wait SPI transfer to end
		while(SPI1->SR & SPI_SR_BSY);
		
		// searching for active sensor
		for (i=0; i<MAX_AXIS_NUM; i++)
		{
			if (sensors[i].source >= 0 && !sensors[i].rx_complete) break;
		}
		// Close connection to the sensor
		if (i < MAX_AXIS_NUM)
		{
			if (sensors[i].type == MCP3202)
			{
				MCP320x_StopDMA(&sensors[i]);
				// get data from next channel
				if (sensors[i].curr_channel < 1)	
				{
					MCP320x_StartDMA(&sensors[i], sensors[i].curr_channel + 1);
					return;
				}
				i++;
			}
		}
		
		
		// Process next sensor
		for ( ;i<MAX_AXIS_NUM;i++)
		{
			if (sensors[i].source >= 0 && sensors[i].rx_complete && sensors[i].tx_complete)
			{
				if (sensors[i].type == MCP3202)
				{
					MCP320x_StartDMA(&sensors[i], 0);
					return;
				}
			}
		}
	}
}

// SPI Tx Complete
void DMA1_Channel3_IRQHandler(void)
{
	uint8_t i=0;
	
	if (DMA_GetITStatus(DMA1_IT_TC3))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC3);
		DMA_Cmd(DMA1_Channel3, DISABLE);
		
		// wait SPI transfer to end
		while(!SPI1->SR & SPI_SR_TXE);
		while(SPI1->SR & SPI_SR_BSY);
		
		// searching for active sensor
		for (i=0; i<MAX_AXIS_NUM; i++)
		{
			if (sensors[i].source >= 0 && !sensors[i].tx_complete)
			{
				sensors[i].tx_complete = 1;
				sensors[i].rx_complete = 0;
				break;
			}
		}
	}
}

// I2C Tx Complete (used for ADS1115 mux setting operation) 
void DMA1_Channel4_IRQHandler(void)
{
	uint8_t i=0;
	uint32_t ticks = I2C_TIMEOUT;
	
	if (DMA_GetFlagStatus(DMA1_FLAG_TC4))
	{
		// Clear transmission complete flag 
		DMA_ClearFlag(DMA1_FLAG_TC4);
		
		I2C_DMACmd(I2C2,DISABLE);	
		DMA_Cmd(DMA1_Channel4,DISABLE);
		
		// EV8_2: Wait until BTF is set before programming the STOP
    while (((I2C2->SR1 & 0x00004) != 0x000004) && --ticks) {;}
		if(ticks == 0)	
		{
			sensors[i].tx_complete = 1;
			sensors[i].rx_complete = 1;
			return;
		}
		ticks = I2C_TIMEOUT;
		
    // Program the STOP
    I2C_GenerateSTOP(I2C2, ENABLE);
		
    /* Make sure that the STOP bit is cleared by Hardware */
		while ((I2C2->CR1&0x200) == 0x200 && --ticks);
		if (ticks == 0)	
		{
			sensors[i].tx_complete = 1;
			sensors[i].rx_complete = 1;
			return;
		}
		
		for (i = 0; i < MAX_AXIS_NUM; i++)
		{
			// searching for active sensor
			if (sensors[i].source == (pin_t)SOURCE_I2C && !sensors[i].tx_complete)
			{			
				sensors[i++].tx_complete = 1;			// TODO: check sensor disconnection				
				break;	
			}
		}
		
		// start processing for next I2C sensor 
		for (; i<MAX_AXIS_NUM; i++)
		{
				if (sensors[i].source == (pin_t)SOURCE_I2C && sensors[i].rx_complete && sensors[i].tx_complete)
				{		
					if (sensors[i].type == ADS1115)
					{
						status = ADS1115_StartDMA(&sensors[i], sensors[i].curr_channel);
						if (status != 0) continue;
						else break;
					}
				}
			}
	}
}

// I2C Rx Complete
void DMA1_Channel5_IRQHandler(void)
{
	uint8_t i=0;
	uint32_t ticks = I2C_TIMEOUT;
	
	if (DMA_GetFlagStatus(DMA1_FLAG_TC5))
	{
		// Clear transmission complete flag 
		DMA_ClearFlag(DMA1_FLAG_TC5);
		
		I2C_DMACmd(I2C2,DISABLE);	
		DMA_Cmd(DMA1_Channel5,DISABLE);
		
		I2C_GenerateSTOP(I2C2, ENABLE);
		
		
		while ((I2C2->CR1&0x200) == 0x200 && --ticks);
		if (ticks == 0)	
		{
			sensors[i].tx_complete = 1;
			sensors[i].rx_complete = 1;
			return;
		}
		
		for (i = 0; i < MAX_AXIS_NUM; i++)
		{
			// searching for active sensor
			if (sensors[i].source == (pin_t)SOURCE_I2C && !sensors[i].rx_complete)
			{
				sensors[i].ok_cnt++;
				sensors[i].rx_complete = 1;		

				if (sensors[i].type == ADS1115)
				{
					// set mux to next channel
					uint8_t channel = (sensors[i].curr_channel < 3) ? (sensors[i].curr_channel + 1) : 0;
					status = ADS1115_SetMuxDMA(&sensors[i], channel, &dev_config);
				}
			}
		}
	}
}

// I2C error
void I2C2_ER_IRQHandler(void)
{
	__IO uint32_t SR1Register =0;

	/* Read the I2C2 status register */
	SR1Register = I2C2->SR1;
	/* If AF = 1 */
	if ((SR1Register & 0x0400) == 0x0400)
	{
		I2C2->SR1 &= 0xFBFF;
		SR1Register = 0;
	}
	/* If ARLO = 1 */
	if ((SR1Register & 0x0200) == 0x0200)
	{
		I2C2->SR1 &= 0xFBFF;
		SR1Register = 0;
	}
	/* If BERR = 1 */
	if ((SR1Register & 0x0100) == 0x0100)
	{
		I2C2->SR1 &= 0xFEFF;
		SR1Register = 0;
	}

	/* If OVR = 1 */
	if ((SR1Register & 0x0800) == 0x0800)
	{
		I2C2->SR1 &= 0xF7FF;
		SR1Register = 0;
	}
		
	// Reset I2C
	I2C2->CR1 |= I2C_CR1_SWRST;
	I2C2->CR1 &= ~I2C_CR1_SWRST;
	I2C_Start();
}



/**
* @brief This function handles USB low priority or CAN RX0 interrupts.
*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{	
	USB_Istr();
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/