/**
 ******************************************************************************
 * @file           : stm32f10x_conf.h
 * @brief          : Library configuration file
 *
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
//#include "stm32f10x_adc.h"
//#include "stm32f10x_bkp.h"
//#include "stm32f10x_can.h"
//#include "stm32f10x_cec.h"
//#include "stm32f10x_crc.h"
//#include "stm32f10x_dac.h"
//#include "stm32f10x_dbgmcu.h"
//#include "stm32f10x_dma.h"
//#include "stm32f10x_exti.h"
//#include "stm32f10x_flash.h"
//#include "stm32f10x_fsmc.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_i2c.h"
//#include "stm32f10x_iwdg.h"
//#include "stm32f10x_pwr.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_rtc.h"
//#include "stm32f10x_sdio.h"
//#include "stm32f10x_spi.h"
//#include "stm32f10x_tim.h"
//#include "stm32f10x_usart.h"
//#include "stm32f10x_wwdg.h"
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
