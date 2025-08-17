/**
 ******************************************************************************
 * @file           : usb_endp.c
 * @brief          : Endpoint Routines 
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
 * Modifications and additions are © 2025 Invictus Cockpit Systems.
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

#include "periphery.h"
#include "usb_hw.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"

#include "crc16.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/



#define REPORT_ID_FIRMWARE				0x04


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

__IO uint8_t EP1_PrevXferComplete = 1;
static volatile uint16_t crc_in = 0;

volatile bool flash_started = 0;
volatile bool flash_finished = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void)
{
    static uint16_t firmware_len = 0;

    uint8_t hid_buf[64];
    uint8_t repotId;

    /* Read received data (2 bytes) */
    USB_SIL_Read(EP1_OUT, hid_buf);
    repotId = hid_buf[0];

    LED1_ON;
    switch (repotId)
    {
        case REPORT_ID_FIRMWARE:
        {
            uint16_t crc_comp = 0;
            uint16_t firmware_in_cnt = 0;
            uint16_t cnt = (uint16_t)(hid_buf[1] << 8) | hid_buf[2];

            if (cnt == 0)   // first packet with info data
            {
                firmware_len = (uint16_t)(hid_buf[5] << 8) | hid_buf[4];
                crc_in       = (uint16_t)(hid_buf[7] << 8) | hid_buf[6];

                /* Compute max pages and max length so we DO NOT erase reserved tail pages.
                   Assumes the last 2 pages are reserved (e.g., CONFIG/FACTORY). */
                const uint16_t reserved_pages = 2u;
                const uint16_t max_pages = (uint16_t)((MAX_PAGE - reserved_pages) - FIRMWARE_START_PAGE);
                const uint32_t max_fw_len = (uint32_t)max_pages * (uint32_t)FLASH_PAGE_SIZE;

                if (firmware_len <= max_fw_len)   // size check vs. app region only
                {
                    flash_started = 1;

                    /* Selective erase: only the application region */
                    FLASH_Unlock();
                    for (uint16_t i = 0; i < max_pages; i++)
                    {
                        if (FLASH_ErasePage(FIRMWARE_COPY_ADDR + (uint32_t)i * FLASH_PAGE_SIZE) != FLASH_COMPLETE)
                        {
                            firmware_in_cnt = 0xF003;    // flash erase error
                            break;
                        }
                    }
                    FLASH_Lock();

                    if (firmware_in_cnt != 0xF003)
                        firmware_in_cnt = cnt + 1;      // request next chunk
                }
                else
                {
                    firmware_in_cnt = 0xF001;          // firmware size error
                }
            }
            else if (flash_started && (firmware_len > 0) && (cnt * 60u < firmware_len))   // body packet(s)
            {
                /* Write full 60-byte chunk */
                FLASH_Unlock();
                for (uint8_t i = 0; i < 60; i += 2)
                {
                    uint16_t hw = (uint16_t)(hid_buf[i + 5] << 8) | hid_buf[i + 4];
                    FLASH_ProgramHalfWord(FIRMWARE_COPY_ADDR + (uint32_t)(cnt - 1) * 60u + i, hw);
                }
                FLASH_Lock();

                firmware_in_cnt = cnt + 1;             // request next chunk
            }
            else if (flash_started && firmware_len > 0)    // last packet
            {
                /* Only write the remaining bytes (<=60), clamp to even */
                uint32_t wrote = (uint32_t)(cnt - 1) * 60u;
                uint16_t tail  = 0;
                if (firmware_len > wrote) {
                    uint32_t rem = (uint32_t)firmware_len - wrote;
                    tail = (uint16_t)(rem > 60u ? 60u : rem);
                }
                // ensure even number of bytes for half-word programming
                if (tail & 1u) tail--;

                FLASH_Unlock();
                for (uint16_t i = 0; i < tail; i += 2)
                {
                    uint16_t hw = (uint16_t)(hid_buf[i + 5] << 8) | hid_buf[i + 4];
                    FLASH_ProgramHalfWord(FIRMWARE_COPY_ADDR + wrote + i, hw);
                }
                FLASH_Lock();

                /* CRC16 over exactly firmware_len bytes at destination */
                crc_comp = Crc16((uint8_t*)FIRMWARE_COPY_ADDR, firmware_len);
                if ((crc_in == crc_comp) && (crc_comp != 0))
                {
                    flash_started  = 0;
                    flash_finished = 1;
                    firmware_in_cnt = 0xF000;          // OK
                }
                else
                {
                    firmware_in_cnt = 0xF002;          // CRC error
                }
            }

            if (firmware_in_cnt > 0)
            {
                uint8_t tmp_buf[3];
                tmp_buf[0] = REPORT_ID_FIRMWARE;
                tmp_buf[1] = (uint8_t)(firmware_in_cnt >> 8);
                tmp_buf[2] = (uint8_t)(firmware_in_cnt & 0xFF);
                USB_CUSTOM_HID_SendReport(1, tmp_buf, 3);
            }
        }
        break;

        default:
            break;
    }

    LED1_OFF;
    SetEPRxStatus(ENDP1, EP_RX_VALID);
}

/*******************************************************************************
* Function Name  : EP1_IN_Callback.
* Description    : EP1 IN Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  EP1_PrevXferComplete = 1;
}

int8_t USB_CUSTOM_HID_SendReport(uint8_t EP_num, uint8_t * data, uint8_t length)
{
	if ((EP1_PrevXferComplete) && (bDeviceState == CONFIGURED))
	{
			USB_SIL_Write(EP1_IN, data, length);
			SetEPTxValid(ENDP1);
			EP1_PrevXferComplete = 0;
			return 0;
	}
	return -1;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

