/**
  ******************************************************************************
  * @file           : i2c.c
  * @brief          : I2C driver implementation - fixed for I2C1 on PB6/PB7
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.2.0
  * @date           2025-09-09
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
  * This software has been carefully modified for a specific purpose.  
	* It is not recommended for use outside of the Invictus HOTAS system.
  *
  * @note           : Simplified I2C driver fixed for I2C1 on PB6/PB7 (Gen 4 board).
  *                   Removed bus-switching logic for hardware-specific implementation.
  *                   Supports ADS1115 ADC communication via TXS0102 level shifter.
  ******************************************************************************
  */

#include "i2c.h"

// Hardware configuration: I2C1 on PB6/PB7 (fixed for Gen 4 board)
static I2C_TypeDef          *s_I2C          = I2C1;
static DMA_Channel_TypeDef  *s_DMA_TX       = DMA1_Channel6;  // I2C1_TX
static DMA_Channel_TypeDef  *s_DMA_RX       = DMA1_Channel7;  // I2C1_RX
static IRQn_Type             s_I2C_ER_IRQn  = I2C1_ER_IRQn;
static uint32_t              s_RCC_APB1_I2C = RCC_APB1Periph_I2C1;

// DMA flags for I2C1 RX (DMA1 Channel 7)
static uint32_t s_DMA_FLAG_GL_RX = DMA1_FLAG_GL7;
static uint32_t s_DMA_FLAG_TC_RX = DMA1_FLAG_TC7;
static uint32_t s_DMA_FLAG_TE_RX = DMA1_FLAG_TE7;
static uint32_t s_DMA_FLAG_HT_RX = DMA1_FLAG_HT7;


/**
  * @brief Initialize I2C1 peripheral on PB6/PB7 for Gen 4 board
  */
void I2C_Start(void)
{
    // Configure I2C1 to use PB6/PB7 (not remapped)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;

    // Enable clocks
    RCC_APB1PeriphClockCmd(s_RCC_APB1_I2C, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // Initialize I2C1
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

    // Initialize DMA channels for I2C1
    DMA_InitTypeDef DMA_InitStructure;

    // RX configuration
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

    // TX configuration
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
