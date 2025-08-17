/**
  ******************************************************************************
  * @file           : i2c.c
  * @brief          : I2C driver implementation
			
	* @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.1.0
  * @date           2025-08-08
  *
  * Based on FreeJoy firmware by Yury Vostrenkov (2020)
  * https://github.com/FreeJoy-Team/FreeJoy
  *
  * This software includes original or modified portions of FreeJoy, distributed
  * under the terms of the GNU General Public License v3.0 or later:
  * https://www.gnu.org/licenses/gpl-3.0.html
  *
  * Modifications and additions are  2025 Invictus Cockpit Systems.
  *
  * This software has been carefully modified for a specific purpose.  It is not recommended for use outside of the Invictus HOTAS system.
  *
	* @note           : Default = I2C2 (PB10/PB11, DMA1 CH4/5) – matches your original.
  *                   If PB6/PB7 are configured as I2C_SCL/I2C_SDA, switches to I2C1.
  ******************************************************************************
  */



#include "i2c.h"

// ===== Internal state for selected bus/peripheral/channels/flags =====
static uint8_t s_use_i2c1 = 0; // 0 => I2C2 (default); 1 => I2C1

static I2C_TypeDef          *s_I2C          = I2C2;
static DMA_Channel_TypeDef  *s_DMA_TX       = DMA1_Channel4;
static DMA_Channel_TypeDef  *s_DMA_RX       = DMA1_Channel5;
static IRQn_Type             s_I2C_ER_IRQn  = I2C2_ER_IRQn;
static uint32_t              s_RCC_APB1_I2C = RCC_APB1Periph_I2C2;

/* We keep your “clear RX flags before TX” behavior; these map per selected RX channel */
static uint32_t s_DMA_FLAG_GL_RX = DMA1_FLAG_GL5;
static uint32_t s_DMA_FLAG_TC_RX = DMA1_FLAG_TC5;
static uint32_t s_DMA_FLAG_TE_RX = DMA1_FLAG_TE5;
static uint32_t s_DMA_FLAG_HT_RX = DMA1_FLAG_HT5;

static void I2C_ApplyBusSelection(void)
{
    if (s_use_i2c1) {
        s_I2C          = I2C1;
        s_DMA_TX       = DMA1_Channel6;
        s_DMA_RX       = DMA1_Channel7;
        s_I2C_ER_IRQn  = I2C1_ER_IRQn;
        s_RCC_APB1_I2C = RCC_APB1Periph_I2C1;

        s_DMA_FLAG_GL_RX = DMA1_FLAG_GL7;
        s_DMA_FLAG_TC_RX = DMA1_FLAG_TC7;
        s_DMA_FLAG_TE_RX = DMA1_FLAG_TE7;
        s_DMA_FLAG_HT_RX = DMA1_FLAG_HT7;
    } else {
        s_I2C          = I2C2;
        s_DMA_TX       = DMA1_Channel4;
        s_DMA_RX       = DMA1_Channel5;
        s_I2C_ER_IRQn  = I2C2_ER_IRQn;
        s_RCC_APB1_I2C = RCC_APB1Periph_I2C2;

        s_DMA_FLAG_GL_RX = DMA1_FLAG_GL5;
        s_DMA_FLAG_TC_RX = DMA1_FLAG_TC5;
        s_DMA_FLAG_TE_RX = DMA1_FLAG_TE5;
        s_DMA_FLAG_HT_RX = DMA1_FLAG_HT5;
    }
}

/**
 * @brief Explicitly force bus (0=I2C2, 1=I2C1) before I2C_Start().
 */
void I2C_ForceBus(uint8_t use_i2c1)
{
    s_use_i2c1 = (use_i2c1 ? 1u : 0u);
    I2C_ApplyBusSelection();
}

/**
 * @brief Auto-select from pins[]: if PB6==I2C_SCL and PB7==I2C_SDA -> I2C1 else I2C2.
 * @note  This does not touch GPIO; set pin modes in your own board init as before.
 */
void I2C_SelectBusFromPins(const pin_t *pins, size_t count)
{
    // guard
    if (!pins || count == 0) {
        s_use_i2c1 = 0;
        I2C_ApplyBusSelection();
        return;
    }

    // We assume your pins[] layout follows PA0..,PB..,PC.. (as in config/GUI).
    // We only need to check B6 and B7 indices.
    // Find the indices of PB6 and PB7 in your pins[] array:
    // From your config and GUI code, the order is:
    //  [A0..A7, A8, A9, A10, A15, B0, B1, B3..B15, C13..C15]  => PB6 index is where "B6" lives in that array.
    // If your array is exactly USED_PINS_NUM with that order, PB6 is the element named "B6", ditto PB7.
    // To avoid hardcoding a fragile index, we scan for the first occurrence of I2C_SCL and I2C_SDA on Port B pins 6/7
    // based on your known Gen4 mapping, but keep it simple: check specific array slots if count>=30.

    int pb6_idx = -1, pb7_idx = -1;

    // Heuristic: in your posted INI, the sequence lists A0..A7,A8,A9,A10,A15,B0,B1,B3..B15,C13,C14,C15 (30 entries).
    // PB6 and PB7 are the 18th and 19th "B*" entries overall (with gaps), which resolve to global indices 17 and 18
    // in your sample—BUT to be safe across small schema shifts, look by board-defined names if available.
    // Since we only have numeric enums here, we fall back to the typical 30-slot layout.
    if (count >= 30) {
        // Using the same index math as your PinComboBox list:
        // Index of B6 in that list is 17 (0-based) and B7 is 18. Adjust if your schema differs.
        pb6_idx = 17;
        pb7_idx = 18;
    }

    uint8_t use_i2c1 = 0;
    if (pb6_idx >= 0 && pb7_idx >= 0 && pb6_idx < (int)count && pb7_idx < (int)count) {
        if (pins[pb6_idx] == I2C_SCL && pins[pb7_idx] == I2C_SDA) {
            use_i2c1 = 1;
        }
    }

    I2C_ForceBus(use_i2c1);
}

/**
  * @brief Initialize selected I2C peripheral (default I2C2).
  * @note  DMA clocks are on AHB; I2C on APB1. GPIO is configured elsewhere (unchanged).
  */
void I2C_Start(void)

{
	
		//Force I2C1 to use PB6/PB7 
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;

    // Apply selection if not applied yet
    I2C_ApplyBusSelection();

    RCC_APB1PeriphClockCmd(s_RCC_APB1_I2C, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = 0x07;
    I2C_Init(s_I2C, &I2C_InitStructure);
    I2C_Cmd(s_I2C, ENABLE);

    I2C_ITConfig(s_I2C, I2C_IT_ERR, ENABLE);
    NVIC_EnableIRQ(s_I2C_ER_IRQn);

    DMA_InitTypeDef DMA_InitStructure;

    // RX base config
    DMA_DeInit(s_DMA_RX);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&s_I2C->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)0;
    DMA_InitStructure.DMA_BufferSize         = 0;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_Init(s_DMA_RX, &DMA_InitStructure);

    // TX base config
    DMA_DeInit(s_DMA_TX);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&s_I2C->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)0;
    DMA_InitStructure.DMA_BufferSize         = 0;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_Init(s_DMA_TX, &DMA_InitStructure);

    I2C_DMACmd(s_I2C, DISABLE);
    I2C_AcknowledgeConfig(s_I2C, ENABLE);
}

/**
  * @brief  Write (register + payload), blocking
  */
int I2C_WriteBlocking(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    uint32_t ticks = I2C_TIMEOUT;
    I2C_DMACmd(s_I2C, DISABLE);
    I2C_AcknowledgeConfig(s_I2C, ENABLE);

    I2C_GenerateSTART(s_I2C, ENABLE);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_MODE_SELECT) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_Send7bitAddress(s_I2C, dev_addr << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_SendData(s_I2C, reg_addr);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    while (length--) {
        I2C_SendData(s_I2C, *data++);
        while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --ticks);
        if (!ticks) return -1; ticks = I2C_TIMEOUT;
    }

    I2C_GenerateSTOP(s_I2C, ENABLE);
    return 0;
}

/**
  * @brief  Read (write reg, then read), blocking
  */
int I2C_ReadBlocking(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length, uint8_t nack)
{
    uint32_t ticks = I2C_TIMEOUT;
    I2C_DMACmd(s_I2C, DISABLE);
    I2C_AcknowledgeConfig(s_I2C, ENABLE);

    while (I2C_GetFlagStatus(s_I2C, I2C_FLAG_BUSY) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_GenerateSTART(s_I2C, ENABLE);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_MODE_SELECT) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_Send7bitAddress(s_I2C, dev_addr << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_SendData(s_I2C, reg_addr);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_GenerateSTART(s_I2C, ENABLE);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_MODE_SELECT) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_Send7bitAddress(s_I2C, dev_addr << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    while (length--) {
        if ((length == 0) && nack) {
            I2C_AcknowledgeConfig(s_I2C, DISABLE);
        }
        while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) && --ticks);
        if (!ticks) return -1; ticks = I2C_TIMEOUT;
        *data++ = I2C_ReceiveData(s_I2C);
    }

    I2C_GenerateSTOP(s_I2C, ENABLE);
    I2C_AcknowledgeConfig(s_I2C, ENABLE);
    return 0;
}

/**
  * @brief  Write (no reg), DMA, non-blocking
  */
int I2C_WriteNonBlocking(uint8_t dev_addr, uint8_t *data, uint16_t length)
{
    uint32_t ticks = I2C_TIMEOUT;
    DMA_InitTypeDef DMA_InitStructure;

    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)data;
    DMA_InitStructure.DMA_BufferSize         = length;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&s_I2C->DR;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_Init(s_DMA_TX, &DMA_InitStructure);

    // keep your original RX-flag clear step
    DMA_ClearFlag(s_DMA_FLAG_GL_RX | s_DMA_FLAG_TC_RX | s_DMA_FLAG_TE_RX | s_DMA_FLAG_HT_RX);
    DMA_ITConfig(s_DMA_TX, DMA_IT_TC, ENABLE);
    I2C_DMACmd(s_I2C, ENABLE);

    I2C_GenerateSTART(s_I2C, ENABLE);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_MODE_SELECT) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_Send7bitAddress(s_I2C, dev_addr << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    DMA_Cmd(s_DMA_TX, ENABLE);
    I2C_DMACmd(s_I2C, ENABLE);
    I2C_DMALastTransferCmd(s_I2C, ENABLE);
    return 0;
}

/**
  * @brief  Read (reg + payload), DMA, non-blocking
  */
int I2C_ReadNonBlocking(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length, uint8_t nack)
{
    uint32_t ticks = I2C_TIMEOUT;
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(s_DMA_RX);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&s_I2C->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)data;
    DMA_InitStructure.DMA_BufferSize         = length;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_Init(s_DMA_RX, &DMA_InitStructure);

    DMA_ClearFlag(s_DMA_FLAG_GL_RX | s_DMA_FLAG_TC_RX | s_DMA_FLAG_TE_RX | s_DMA_FLAG_HT_RX);
    DMA_ITConfig(s_DMA_RX, DMA_IT_TC, ENABLE);
    I2C_DMACmd(s_I2C, ENABLE);

    while (I2C_GetFlagStatus(s_I2C, I2C_FLAG_BUSY) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_GenerateSTART(s_I2C, ENABLE);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_MODE_SELECT) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_Send7bitAddress(s_I2C, dev_addr << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_SendData(s_I2C, reg_addr);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_GenerateSTART(s_I2C, ENABLE);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_MODE_SELECT) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    I2C_Send7bitAddress(s_I2C, dev_addr << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(s_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --ticks);
    if (!ticks) return -1; ticks = I2C_TIMEOUT;

    DMA_Cmd(s_DMA_RX, ENABLE);
    I2C_DMACmd(s_I2C, ENABLE);
    return 0;
}
