/**
  ******************************************************************************
  * @file           : analog.c
  * @brief          : Analog axis driver implementation
		
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

#include "analog.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "mcp320x.h"
#include "ads1115.h"
#include "buttons.h"
#include "device_info.h"

#define CENTER_LED_RANGE 125 //ADC Count deviation allowace for Centered Axis LEDs to be lit

sensor_t sensors[MAX_AXIS_NUM];	
uint16_t adc_data[MAX_AXIS_NUM];
uint16_t tmp_adc_data[MAX_AXIS_NUM];
uint8_t adc_conv_num[MAX_AXIS_NUM];

analog_data_t scaled_axis_data[MAX_AXIS_NUM];
analog_data_t raw_axis_data[MAX_AXIS_NUM];
analog_data_t out_axis_data[MAX_AXIS_NUM];


analog_data_t FILTER_LEVEL_1_COEF[FILTER_BUF_SIZE] = {40, 30, 15, 10, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // coefficients (sum <= 100) to prevent overflow
analog_data_t FILTER_LEVEL_2_COEF[FILTER_BUF_SIZE] = {30, 20, 10, 10, 10, 6, 6, 4, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
analog_data_t FILTER_LEVEL_3_COEF[FILTER_BUF_SIZE] = {25, 20, 10, 10, 8, 6, 6, 4, 2, 2, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0};
analog_data_t FILTER_LEVEL_4_COEF[FILTER_BUF_SIZE] = {20, 15, 10, 8, 8, 6, 6, 6, 4, 2, 2, 2, 2, 2, 2, 2, 2, 1, 0, 0};
analog_data_t FILTER_LEVEL_5_COEF[FILTER_BUF_SIZE] = {20, 15, 12, 10, 8, 7, 6, 5, 4, 3, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0}; 
analog_data_t FILTER_LEVEL_6_COEF[FILTER_BUF_SIZE] = {18, 14, 12, 10, 8, 7, 6, 5, 5, 4, 3, 3, 2, 1, 1, 1, 0, 0, 0, 0}; 
analog_data_t FILTER_LEVEL_7_COEF[FILTER_BUF_SIZE] = {16, 14, 12, 10, 8, 7, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0}; 

analog_data_t filter_buffer[MAX_AXIS_NUM][FILTER_BUF_SIZE];
analog_data_t deadband_buffer[MAX_AXIS_NUM][DEADBAND_BUF_SIZE];
	
logical_buttons_state_t axis_buttons[MAX_AXIS_NUM][3];
int32_t	axis_trim_value[MAX_AXIS_NUM];

uint8_t adc_cnt = 0;
uint8_t sensors_cnt = 0;
	
adc_channel_config_t channel_config[MAX_AXIS_NUM] =
{
	{ADC_Channel_0, 0}, {ADC_Channel_1, 1}, 
	{ADC_Channel_2, 2}, {ADC_Channel_3, 3},
	{ADC_Channel_4, 4}, {ADC_Channel_5, 5}, 
	{ADC_Channel_6, 6}, {ADC_Channel_7, 7}, 
};

unsigned int iabs (int x)
{
	if (x >= 0) return x;
	else return -x;
}

/**
  * @brief  Transform value from input range to value in output range
	*	@param	x: Value to transform
	*	@param	in_min:	Minimum value of input range
	*	@param	in_max:	Maximum value of input range
	*	@param	out_min: Minimum value of output range
	*	@param	out_max: Maximum value of output range
  * @retval Transformed value
  */
static int32_t map2(	int32_t x, 
											int32_t in_min, 
											int32_t in_max, 
											int32_t out_min,
											int32_t out_max)
{
	if (x < in_min)	return out_min;
	if (x > in_max)	return out_max;
		
	x = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	
	return x;
}

/**
  * @brief  Transform value of input range -180000 to 180000 to range -32767 to 32767
	*	@param	x: Value to transform
  * @retval Transformed value
  */
static int32_t map_tle (int32_t x)
{
	x = x * 100 / 549;
	
	if (x > AXIS_MAX_VALUE) x = AXIS_MAX_VALUE;
	else if (x < AXIS_MIN_VALUE) x = AXIS_MIN_VALUE;
	
	return x;
}

/**
  * @brief  Transform value from input range to value in output range 
	*	@param	x: Value to transform
	*	@param	in_min:	Minimum value of input range
	*	@param	in_center: Center value of input range
	*	@param	in_max:	Maximum value of input range
	*	@param	out_min: Minimum value of output range
	*	@param	out_center:	Center value of input range
	*	@param	out_max: Maximum value of output range
	*	@param	deadband_size: Width of center dead zone
  * @retval Transformed value
  */
static int32_t map3(	int32_t x, 
											int32_t in_min, 
											int32_t in_center, 
											int32_t in_max, 
											int32_t out_min,
											int32_t out_center,
											int32_t out_max,
											uint8_t deadband_size)
{
	int32_t tmp;
	int32_t ret;
	int32_t dead_zone_right;
	int32_t dead_zone_left;
	
	tmp = x;
	dead_zone_right = ((in_max - in_center)*(deadband_size))>>8; //Adjusted bit shift from >>10 to >>8, which results in 5x larger deadband on raw value.  GUI setting of 15 now
	dead_zone_left = ((in_center - in_min)*deadband_size)>>8;			// covers a 300 ADC count window.  Probably overkill, but wanted the headroom for the older VFTs
	
	if (tmp < in_min)	return out_min;
	if (tmp > in_max)	return out_max; 
	if ((tmp > in_center && (tmp - in_center) < dead_zone_right) || 
			(tmp < in_center &&	(in_center - tmp) < dead_zone_left) || 
			tmp == in_center)
	{
		return out_center;
	}		
	
	if (tmp < in_center)
	{
		ret = ((tmp - in_min) * (out_center - out_min) / (in_center - dead_zone_left - in_min) + out_min);
  }
	else
	{
		ret = ((tmp - in_center - dead_zone_right) * (out_max - out_center) / (in_max - in_center - dead_zone_right) + out_center);
	}
	return ret;
}

/**
  * @brief  Lowing input data resolution
	*	@param	value: Value to process
	*	@param	resolution:	Desired resolution of value in bits
  * @retval Resulting value
  */
analog_data_t SetResolutioin (analog_data_t value, uint8_t resolution)
{
	int32_t tmp = 0;
	int32_t ret = 0;
	uint32_t fullscale = AXIS_MAX_VALUE - AXIS_MIN_VALUE;
	float step;
	
	if (resolution >= 16)
	{
		return value;
	}
	else if (resolution > 0)
	{
		tmp = fullscale >> (16 - resolution);
		step = (float)fullscale/tmp;
		
		tmp = value;
		tmp = (tmp - AXIS_MIN_VALUE)  >> (16 - resolution);
		ret = step * tmp + AXIS_MIN_VALUE;
	}
	
	return ret;
}

/**
  * @brief  FIR filter for input data
	*	@param	value: Value to process
	*	@param	filter_buf:	Pointer to filter data buffer
	*	@param	filter_lvl:	Desired filter level
	*   This parameter can be 0-3 (where 0 is no filtration, 3 is high filtration level)
  * @retval Resulting value
  */
analog_data_t Filter (analog_data_t value, analog_data_t * filter_buf, filter_t filter_lvl)
{
	int32_t tmp32;
	
	switch (filter_lvl)
	{
		default:
		case FILTER_NO:
			return value;
		
		case FILTER_LEVEL_1:
			tmp32 = value * FILTER_LEVEL_1_COEF[0];
			for (uint8_t i=FILTER_BUF_SIZE-1; i>0; i--)
			{
				filter_buf[i] = filter_buf[i-1];
				
				tmp32 += filter_buf[i] * FILTER_LEVEL_1_COEF[i];
			}
		break;
		
		case FILTER_LEVEL_2:
			tmp32 = value * FILTER_LEVEL_2_COEF[0];
			for (uint8_t i=FILTER_BUF_SIZE-1; i>0; i--)
			{
				filter_buf[i] = filter_buf[i-1];
				
				tmp32 += filter_buf[i] * FILTER_LEVEL_2_COEF[i];
			}
		break;
		
		case FILTER_LEVEL_3:
			tmp32 = value * FILTER_LEVEL_3_COEF[0];
			for (uint8_t i=FILTER_BUF_SIZE-1; i>0; i--)
			{
				filter_buf[i] = filter_buf[i-1];
				
				tmp32 += filter_buf[i] * FILTER_LEVEL_3_COEF[i];
			}
		break;
			
		case FILTER_LEVEL_4:
			
			tmp32 = value * FILTER_LEVEL_4_COEF[0];
			for (uint8_t i=FILTER_BUF_SIZE-1; i>0; i--)
			{
				filter_buf[i] = filter_buf[i-1];
				
				tmp32 += filter_buf[i] * FILTER_LEVEL_4_COEF[i];
			}			
		break;
			
		case FILTER_LEVEL_5:
			
			tmp32 = value * FILTER_LEVEL_5_COEF[0];
			for (uint8_t i=FILTER_BUF_SIZE-1; i>0; i--)
			{
				filter_buf[i] = filter_buf[i-1];
				
				tmp32 += filter_buf[i] * FILTER_LEVEL_5_COEF[i];
			}			
		break;

		case FILTER_LEVEL_6:
			
			tmp32 = value * FILTER_LEVEL_6_COEF[0];
			for (uint8_t i=FILTER_BUF_SIZE-1; i>0; i--)
			{
				filter_buf[i] = filter_buf[i-1];
				
				tmp32 += filter_buf[i] * FILTER_LEVEL_6_COEF[i];
			}			
		break;
			
		case FILTER_LEVEL_7:
			
			tmp32 = value * FILTER_LEVEL_7_COEF[0];
			for (uint8_t i=FILTER_BUF_SIZE-1; i>0; i--)
			{
				filter_buf[i] = filter_buf[i-1];
				
				tmp32 += filter_buf[i] * FILTER_LEVEL_7_COEF[i];
			}			
		break;
	}
	
	filter_buf[0] = (uint16_t)(tmp32/100);
	
	
	return filter_buf[0];
}

/**
  * @brief  Dynamic deadband implementation
	*	@param	value: Value to process
	*	@param	deadband_buf:	Pointer to deadband data buffer
	* @param	deadband_size: Deadband width
  * @retval Is holding?
  * @notes 	This function checks if the last 16 [DEADBAND
	*					_BUFF_SIZE] input values have been stable 
	*					within a small range (determined 
	*					by deadband size). If so, it considers the input 
	*					to be "holding still." If not, it's moving or jittering.
	*					DEADBAND_BUF_SIZE set in Common_defines.h
  */
uint8_t IsDynamicDeadbandHolding (analog_data_t value, analog_data_t * deadband_buf, uint8_t deadband_size)
{
	uint8_t is_holding = 1;
	int32_t	disp = 0;
	int32_t mean = 0;
	int32_t treshold;
	
	treshold = 3 * deadband_size;
	
	// calculating mean value
	for (uint8_t i=0; i<DEADBAND_BUF_SIZE; i++)
	{
		mean += deadband_buf[i];
	}
	mean = mean/DEADBAND_BUF_SIZE;
	
	// calculating dispercy
	for (uint8_t i=0; i<DEADBAND_BUF_SIZE; i++)
	{
		disp += (deadband_buf[i] - mean)*(deadband_buf[i] - mean);
	}
	disp = disp/DEADBAND_BUF_SIZE;
	
	if (disp > treshold*treshold)	
	{
		is_holding = 0;
	}
	// shift buffer data and add a new value
	for (uint8_t i=DEADBAND_BUF_SIZE-1; i>0; i--)
	{
		deadband_buf[i] = deadband_buf[i-1];
	}
	deadband_buf[0] = value;
	
	return is_holding;
}

/**
  * @brief  Scaling input data accodring to set axis curve shape
	*	@param	p_axis_cfg: Pointer to axis configuration structure
	*	@param	value:	Value to process
	*	@param	point_cnt:	Number of points in axis curve
  * @retval Resulting value
  */
analog_data_t ShapeFunc (axis_config_t * p_axis_cfg,  analog_data_t value, uint8_t point_cnt)
{
	int32_t out_min, out_max, step;
	int32_t in_min, in_max;
	uint8_t min_index;
	analog_data_t ret;
	
	// check if is not linear
	if (p_axis_cfg->curve_shape[0] != -100 ||  
			p_axis_cfg->curve_shape[1] != -80 ||
			p_axis_cfg->curve_shape[2] != -60 ||
			p_axis_cfg->curve_shape[3] != -40 ||
			p_axis_cfg->curve_shape[4] != - 20 ||
			p_axis_cfg->curve_shape[5] != 0 || 
			p_axis_cfg->curve_shape[6] != 20 ||
			p_axis_cfg->curve_shape[7] != 40 ||
			p_axis_cfg->curve_shape[8] != 60 ||
			p_axis_cfg->curve_shape[9] != 80 ||
			p_axis_cfg->curve_shape[10] != 100)
	{	
		
		
		int32_t tmp = value - AXIS_MIN_VALUE;
		int32_t fullscale = AXIS_MAX_VALUE - AXIS_MIN_VALUE;
		
		step = (float)fullscale/((float)point_cnt-1.0f);
		min_index = tmp/step;
		
		if (min_index == point_cnt-1) min_index = point_cnt-2;
		
		in_min = AXIS_MIN_VALUE + min_index*step;
		in_max = AXIS_MIN_VALUE + (min_index+1)*step;
		
		out_min = ((int32_t)p_axis_cfg->curve_shape[min_index] * (int32_t)fullscale/200 + (int32_t)AXIS_CENTER_VALUE);
		out_max = ((int32_t)p_axis_cfg->curve_shape[min_index+1] * (int32_t)fullscale/200 + (int32_t)AXIS_CENTER_VALUE);
		
		ret = map2(value, in_min, in_max, out_min, out_max);
	}
	else
	{
		ret = value;
	}
	
	return(ret);
}

/**
  * @brief  Axes initialization after startup
	*	@param	p_dev_config: Pointer to device configuration structure
  * @retval None
  */
void AxesInit (dev_config_t * p_dev_config)
{
  ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	 /* DMA and ADC controller clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	for (int i = 0; i<MAX_AXIS_NUM; i++)
	{
		sensors[i].source = -1;
		sensors[i].rx_complete = 1;
		sensors[i].tx_complete = 1;
	}
	
	// Init sources
	for (int i=0; i<USED_PINS_NUM; i++)
	{
		
		   
		if (p_dev_config->pins[i] == AXIS_ANALOG)
		{
			for (uint8_t k=0; k<MAX_AXIS_NUM; k++)
			{
				if (p_dev_config->axis_config[k].source_main == i && sensors_cnt < MAX_AXIS_NUM)
				{
					sensors[sensors_cnt].type = ANALOG;			
					sensors[sensors_cnt].source = i;
					sensors_cnt++;
					adc_cnt++;
					break;
				}
			}
		}
		else if (p_dev_config->pins[i] == MCP3202_CS)
		{
			for (uint8_t k=0; k<MAX_AXIS_NUM; k++)
			{
				if (p_dev_config->axis_config[k].source_main == i && sensors_cnt < MAX_AXIS_NUM)
				{
					sensors[sensors_cnt].type = MCP3202;	
					sensors[sensors_cnt].source = i;
					sensors_cnt++;
					break;
				}
			}
		}
	}

	if (p_dev_config->pins[21] == (I2C_SCL) && p_dev_config->pins[22] == (I2C_SDA)) //pins[21]=PB10 (I2C2_SCL), pins[22]=PB11 (I2C2_SDA)
	{
		// look for ADS1115 sensors with different addresses
		for (uint8_t addr = ADS1115_I2C_ADDR_MIN; addr <= ADS1115_I2C_ADDR_MAX; addr ++)
		{
			for (uint8_t k=0; k<MAX_AXIS_NUM; k++)
			{
				if (p_dev_config->axis_config[k].source_main == (pin_t) SOURCE_I2C)
				{
					if ((p_dev_config->axis_config[k].i2c_address) == addr)
					{
						sensors[sensors_cnt].address = p_dev_config->axis_config[k].i2c_address;
						sensors[sensors_cnt].type = ADS1115;
						sensors[sensors_cnt].source = (pin_t) SOURCE_I2C;

						ADS1115_Init(&sensors[sensors_cnt], p_dev_config);
						sensors_cnt++;
						break;
					}
				}
			}
		}
	}
	
	// Init ADC
	if (adc_cnt > 0)
	{
		// Ranking ADC sensors
		uint8_t rank = 0;	
		for (uint8_t adc=0; adc<MAX_AXIS_NUM; adc++)
		{
			uint8_t is_present = 0;
			for (int i=0; i<sensors_cnt; i++)
			{ 
				if (sensors[i].type == ANALOG && sensors[i].source == adc)
				{
					sensors[i].curr_channel = rank;
					is_present = 1;
				}			
			}
			if (is_present) rank++;
		}
		
		/* ADC1 configuration ------------------------------------------------------*/
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = ENABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = adc_cnt;
		ADC_Init(ADC1, &ADC_InitStructure);

		/* Enable ADC1 DMA */
		ADC_DMACmd(ADC1, ENABLE);
	
		// Configure ADC channels
		uint8_t tmp_rank = 1;
		for (int i=0; i<MAX_AXIS_NUM; i++)
		{ 
			if (p_dev_config->pins[i] == AXIS_ANALOG)		
			{
				for (uint8_t k=0; k<MAX_AXIS_NUM; k++)
				{
					if (p_dev_config->axis_config[k].source_main == i)
					{
						/* ADC1 regular channel configuration */ 
						ADC_RegularChannelConfig(ADC1, channel_config[i].channel, tmp_rank++, ADC_SampleTime_239Cycles5);
						break;
					}
				}
			}
		}
		
		/* DMA1 channel1 configuration ----------------------------------------------*/
		DMA_DeInit(DMA1_Channel1);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &adc_data[0];
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = adc_cnt;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
			/* Enable ADC1 */
		ADC_Cmd(ADC1, ENABLE);
			
			/* Enable ADC1 reset calibration register */   
		ADC_ResetCalibration(ADC1);
		/* Check the end of ADC1 reset calibration register */
		while(ADC_GetResetCalibrationStatus(ADC1));

		/* Start ADC1 calibration */
		ADC_StartCalibration(ADC1);
		/* Check the end of ADC1 calibration */
		while(ADC_GetCalibrationStatus(ADC1));
		

// Centering indicator LED setup: PA8 and PB0
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

GPIO_InitTypeDef gpio;
gpio.GPIO_Mode = GPIO_Mode_Out_PP;
gpio.GPIO_Speed = GPIO_Speed_2MHz;

// Initialize PA8 (X-Axis Indicator)
gpio.GPIO_Pin = GPIO_Pin_8;
GPIO_Init(GPIOA, &gpio);

// Initialize PB0 (Y Axis Indicator)
gpio.GPIO_Pin = GPIO_Pin_0;
GPIO_Init(GPIOB, &gpio);

// Turn both LEDs off initially
GPIO_ResetBits(GPIOA, GPIO_Pin_8);
GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	}
}



/**
  * @brief  ADC conversion processing routine
  * @retval None
  */
void ADC_Conversion (void)
{
	
	if (adc_cnt > 0)
	{
		// clear buffer from old values
		for (uint8_t j=0; j<MAX_AXIS_NUM; j++)
		{
			adc_data[j] = 0;								
		}
		// perform multiple conversions	
		for (uint8_t i=0; i<ADC_CONV_NUM; i++)	
		{
			DMA1_Channel1->CMAR = (uint32_t) &tmp_adc_data[0];
			DMA_SetCurrDataCounter(DMA1_Channel1, adc_cnt);	
			DMA_Cmd(DMA1_Channel1, ENABLE);
			ADC_Cmd(ADC1, ENABLE);
			/* Start ADC1 Software Conversion */ 
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);

			while (!DMA_GetFlagStatus(DMA1_FLAG_TC1));
			DMA_ClearFlag(DMA1_FLAG_TC1);
				
			ADC_Cmd(ADC1, DISABLE);
			DMA_Cmd(DMA1_Channel1, DISABLE);
						
			for (uint8_t j=0; j<MAX_AXIS_NUM; j++)
			{
				adc_data[j] += tmp_adc_data[j];								
			}
		}
		// get mean value
		for (uint8_t j=0; j<MAX_AXIS_NUM; j++)
		{
			adc_data[j] /= ADC_CONV_NUM;								
		}
	}
}

/**
  * @brief  Integer square root using Newton's method
  * @param  n: Input value
  * @retval Square root of n
  */
static uint32_t isqrt(uint32_t n)
{
	if (n == 0) return 0;

	uint32_t x = n;
	uint32_t y = (x + 1) >> 1;

	while (y < x) {
		x = y;
		y = (x + n/x) >> 1;
	}

	return x;
}

/**
  * @brief  Apply circular deadband to a pair of axes
  * @param  x_value: Filtered X axis value (e.g., Roll)
  * @param  y_value: Filtered Y axis value (e.g., Pitch)
  * @param  x_center: X axis center calibration
  * @param  y_center: Y axis center calibration
  * @param  deadband_radius: Circular deadband radius (0-127)
  * @param  x_out: Pointer to output X value (relative to center)
  * @param  y_out: Pointer to output Y value (relative to center)
  * @retval 1 if in deadband (return center), 0 if outside deadband
  */
static uint8_t ApplyCircularDeadband(
	int32_t x_value,
	int32_t y_value,
	int32_t x_center,
	int32_t y_center,
	uint8_t deadband_radius,
	int32_t *x_out,
	int32_t *y_out)
{
	// Calculate offset from center
	int32_t dx = x_value - x_center;
	int32_t dy = y_value - y_center;

	// Calculate squared distance (avoid sqrt for performance check)
	int32_t dist_squared = dx*dx + dy*dy;

	// Calculate deadband radius squared (scale by calibration range)
	// Assume typical range of 2000 counts per axis
	int32_t radius_scaled = (deadband_radius * 2000) >> 8;  // Same scaling as map3
	int32_t radius_squared = radius_scaled * radius_scaled;

	// If inside circular deadband, return center
	if (dist_squared <= radius_squared) {
		*x_out = 0;  // AXIS_CENTER_VALUE will be added later
		*y_out = 0;
		return 1;
	}

	// Outside deadband: scale linearly from deadband edge to full range
	// Calculate radial distance
	uint32_t dist = isqrt((uint32_t)dist_squared);
	uint32_t radius = (uint32_t)radius_scaled;

	if (dist > 0) {
		// Scale factor = (dist - radius) / dist
		// New position = offset * scale_factor
		int32_t scale_num = (int32_t)(dist - radius);
		*x_out = (dx * scale_num) / (int32_t)dist;
		*y_out = (dy * scale_num) / (int32_t)dist;
	} else {
		*x_out = 0;
		*y_out = 0;
	}

	return 0;
}

/**
  * @brief  Axes data processing routine
	*	@param	p_dev_config: Pointer to device configuration structure
  * @retval None
  */
void AxesProcess (dev_config_t * p_dev_config)
{

	int32_t tmp[MAX_AXIS_NUM];
	float tmpf;
	uint8_t circular_processed[MAX_AXIS_NUM] = {0};  // Track axes processed with circular deadband

	for (uint8_t i=0; i<MAX_AXIS_NUM; i++)
	{

		int8_t source = p_dev_config->axis_config[i].source_main;
		uint8_t channel = p_dev_config->axis_config[i].channel;
		uint8_t address = p_dev_config->axis_config[i].i2c_address;

		if (source >= 0)		// source SPI sensors or internal ADC
		{
			if (p_dev_config->pins[source] == AXIS_ANALOG)					// source analog
			{
				uint8_t k=0;
				// search for needed sensor
				for (k=0; k<MAX_AXIS_NUM; k++)
				{
					if (sensors[k].source == source) break;
				}

				if (p_dev_config->axis_config[i].offset_angle > 0)
				{
						tmp[i] = adc_data[sensors[k].curr_channel] - p_dev_config->axis_config[i].offset_angle * 170;
						if (tmp[i] < 0) tmp[i] += 4095;
						else if (tmp[i] > 4095) tmp[i] -= 4095;
				}
				else
				{
					tmp[i] = adc_data[sensors[k].curr_channel];
				}
				raw_axis_data[i] = map2(tmp[i], 0, 4095, AXIS_MIN_VALUE, AXIS_MAX_VALUE);
			}
			else if (p_dev_config->pins[source] == MCP3202_CS)				// source MCP3202
			{
				uint8_t k=0;
				// search for needed sensor
				for (k=0; k<MAX_AXIS_NUM; k++)
				{
					if (sensors[k].source == source) break;
				}
				// get data
				if (p_dev_config->axis_config[i].offset_angle > 0)	// offset enabled
				{
					tmp[i] = MCP320x_GetData(&sensors[k], channel) - p_dev_config->axis_config[i].offset_angle * 170;
					if (tmp[i] < 0) tmp[i] += 4095;
					else if (tmp[i] > 4095) tmp[i] -= 4095;
				}
				else		// offset disabled
				{
					tmp[i] = MCP320x_GetData(&sensors[k], channel);
				}
				raw_axis_data[i] = map2(tmp[i], 0, 4095, AXIS_MIN_VALUE, AXIS_MAX_VALUE);
			}

		}
		else if (source == (axis_source_t)SOURCE_I2C)				// source I2C sensor
		{
			uint8_t k=0;
			// search for needed sensor
			for (k=0; k<MAX_AXIS_NUM; k++)
			{
				if (sensors[k].address == address) break;
			}
			// get data
			if (sensors[k].type == ADS1115)
			{
				if (p_dev_config->axis_config[i].offset_angle > 0)	// offset enabled
				{
					tmp[i] = ADS1115_GetData(&sensors[k], channel) - p_dev_config->axis_config[i].offset_angle * 2730;
					if (tmp[i] < 0) tmp[i] += 32767;
					else if (tmp[i] > 32767) tmp[i] -= 32767;		// always false, ADS1115_GetData return int16_t
				}
				else		// offset disabled
				{
					tmp[i] = ADS1115_GetData(&sensors[k], channel);
				}
				// Differential mode uses full signed range, single-ended uses positive range only
			if (g_device_info.adc_mode[channel] == 1)  // Differential mode
			{
				raw_axis_data[i] = map2(tmp[i], -32768, 32767, AXIS_MIN_VALUE, AXIS_MAX_VALUE);
			}
			else  // Single-ended mode
			{
				raw_axis_data[i] = map2(tmp[i], 0, 32767, AXIS_MIN_VALUE, AXIS_MAX_VALUE);
			}
			}
		}



		// Filtering
		tmp[i] = Filter(raw_axis_data[i], filter_buffer[i], p_dev_config->axis_config[i].filter);

		// Deadband processing and scaling
		if (p_dev_config->axis_config[i].is_circular_deadband && !circular_processed[i])
		{
			// CIRCULAR DEADBAND MODE
			uint8_t pair_idx = p_dev_config->axis_config[i].circular_pair_axis;

			// Only process if paired axis is valid and not yet processed
			if (pair_idx < MAX_AXIS_NUM && pair_idx != i && !circular_processed[pair_idx])
			{
				int32_t x_scaled, y_scaled;

				// First scale both axes to output range WITHOUT deadband (deadband_size=0)
				int32_t x_value = map3(tmp[i],
					p_dev_config->axis_config[i].calib_min,
					p_dev_config->axis_config[i].calib_center,
					p_dev_config->axis_config[i].calib_max,
					AXIS_MIN_VALUE,
					AXIS_CENTER_VALUE,
					AXIS_MAX_VALUE,
					0);  // No deadband in initial scaling

				int32_t y_value = map3(tmp[pair_idx],
					p_dev_config->axis_config[pair_idx].calib_min,
					p_dev_config->axis_config[pair_idx].calib_center,
					p_dev_config->axis_config[pair_idx].calib_max,
					AXIS_MIN_VALUE,
					AXIS_CENTER_VALUE,
					AXIS_MAX_VALUE,
					0);  // No deadband in initial scaling

				// Apply circular deadband to axis pair
				ApplyCircularDeadband(
					x_value,
					y_value,
					AXIS_CENTER_VALUE,  // Center in output space
					AXIS_CENTER_VALUE,
					p_dev_config->axis_config[i].deadband_size,
					&x_scaled,
					&y_scaled
				);

				// Add center offset to get final values
				tmp[i] = AXIS_CENTER_VALUE + x_scaled;
				tmp[pair_idx] = AXIS_CENTER_VALUE + y_scaled;

				// Mark both axes as processed
				circular_processed[i] = 1;
				circular_processed[pair_idx] = 1;
			}
			else
			{
				// Paired axis invalid or already processed, fall back to rectangular deadband
				tmp[i] = map3( tmp[i],
					p_dev_config->axis_config[i].calib_min,
					p_dev_config->axis_config[i].calib_center,
					p_dev_config->axis_config[i].calib_max,
					AXIS_MIN_VALUE,
					AXIS_CENTER_VALUE,
					AXIS_MAX_VALUE,
					p_dev_config->axis_config[i].deadband_size);
			}
		}
		else if (!circular_processed[i])
		{
			// RECTANGULAR DEADBAND MODE (existing behavior)
			if (!p_dev_config->axis_config[i].is_dynamic_deadband)
			{
				// Scale output data
				tmp[i] = map3( tmp[i],
										 p_dev_config->axis_config[i].calib_min,
										 p_dev_config->axis_config[i].calib_center,
										 p_dev_config->axis_config[i].calib_max,
										 AXIS_MIN_VALUE,
										 AXIS_CENTER_VALUE,
										 AXIS_MAX_VALUE,
										 p_dev_config->axis_config[i].deadband_size);
			}
			else
			{
				// Scale output data
				tmp[i] = map3( tmp[i],
										 p_dev_config->axis_config[i].calib_min,
										 p_dev_config->axis_config[i].calib_center,
										 p_dev_config->axis_config[i].calib_max,
										 AXIS_MIN_VALUE,
										 AXIS_CENTER_VALUE,
										 AXIS_MAX_VALUE,
										 p_dev_config->axis_config[i].deadband_size);  // <-- restoring Deadband in addition to dynamic deadband

				if (iabs(tmp[i] - scaled_axis_data[i]) < 3*3*p_dev_config->axis_config[i].deadband_size &&			// 3*3*deadband_size = 3 sigma
						IsDynamicDeadbandHolding(tmp[i], deadband_buffer[i], p_dev_config->axis_config[i].deadband_size))
				{
					tmp[i] = scaled_axis_data[i];			// keep value if deadband confidition is true
				}
			}
		}
		
		// Shaping
		tmp[i] = ShapeFunc(&p_dev_config->axis_config[i], tmp[i], 11);
		// Lowing resolution if needed
		tmp[i] = SetResolutioin(tmp[i], p_dev_config->axis_config[i].resolution + 1);
		
		// Invertion
		if (p_dev_config->axis_config[i].inverted > 0)
		{
			tmp[i] = 0 - tmp[i];
		}

		// Prescaling
		if (p_dev_config->axis_config[i].prescaler != 100)
		{
			if ( 	// no prescaler button defined
					 ((p_dev_config->axis_config[i].button1 < 0 || p_dev_config->axis_config[i].button1_type != AXIS_BUTTON_PRESCALER_EN) &&			
						(p_dev_config->axis_config[i].button2 < 0 || p_dev_config->axis_config[i].button2_type != AXIS_BUTTON_PRESCALER_EN) &&
						(p_dev_config->axis_config[i].button3 < 0 || p_dev_config->axis_config[i].button3_type != AXIS_BUTTON_PRESCALER_EN)) ||
						// or defined and pressed
						((p_dev_config->axis_config[i].button1 >=0 && axis_buttons[i][0].current_state && 
							p_dev_config->axis_config[i].button1_type == AXIS_BUTTON_PRESCALER_EN) ||
						 (p_dev_config->axis_config[i].button2 >=0 && axis_buttons[i][1].current_state && 
							p_dev_config->axis_config[i].button2_type == AXIS_BUTTON_PRESCALER_EN) ||
						 (p_dev_config->axis_config[i].button3 >=0 && axis_buttons[i][2].current_state && 
							p_dev_config->axis_config[i].button3_type == AXIS_BUTTON_PRESCALER_EN))
				 )
			{
				if (p_dev_config->axis_config[i].is_centered)	tmp[i] = tmp[i]*p_dev_config->axis_config[i].prescaler / 100;
				else	tmp[i] = (tmp[i] - AXIS_MIN_VALUE)*p_dev_config->axis_config[i].prescaler / 100 + AXIS_MIN_VALUE;
			}
		}

		// Buttons section
    {
			int32_t millis = GetMillis();
			
			uint8_t inc_button_num = 0;
			uint8_t rst_button_num = 0;
			uint8_t dec_button_num = 0;
			uint8_t cent_button_num = 0;
			
			// detect buttons
			if (p_dev_config->axis_config[i].button1 >= 0 && 
					p_dev_config->axis_config[i].button1_type == AXIS_BUTTON_UP)	inc_button_num |= 1<<0;
			if (p_dev_config->axis_config[i].button3 >= 0 && 
							 p_dev_config->axis_config[i].button3_type == AXIS_BUTTON_UP)	inc_button_num |= 1<<2;
			
			if (p_dev_config->axis_config[i].button1 >= 0 && 
					p_dev_config->axis_config[i].button1_type == AXIS_BUTTON_RESET)	rst_button_num |= 1<<0;
			if (p_dev_config->axis_config[i].button2 >= 0 && 
							 p_dev_config->axis_config[i].button2_type == AXIS_BUTTON_RESET)	rst_button_num |= 1<<1;
			if (p_dev_config->axis_config[i].button3 >= 0 && 
							 p_dev_config->axis_config[i].button3_type == AXIS_BUTTON_RESET)	rst_button_num |= 1<<2;
			
			if (p_dev_config->axis_config[i].button1 >= 0 && 
					p_dev_config->axis_config[i].button1_type == AXIS_BUTTON_DOWN)	dec_button_num |= 1<<0;
			if (p_dev_config->axis_config[i].button3 >= 0 && 
							 p_dev_config->axis_config[i].button3_type == AXIS_BUTTON_DOWN)	dec_button_num |= 1<<2;
			
			if (p_dev_config->axis_config[i].button1 >= 0 && 
					p_dev_config->axis_config[i].button1_type == AXIS_BUTTON_CENTER)	cent_button_num |= 1<<0;
			if (p_dev_config->axis_config[i].button2 >= 0 && 
							 p_dev_config->axis_config[i].button2_type == AXIS_BUTTON_CENTER)	cent_button_num |= 1<<1;
			if (p_dev_config->axis_config[i].button3 >= 0 && 
							 p_dev_config->axis_config[i].button3_type == AXIS_BUTTON_CENTER)	cent_button_num |= 1<<2;
			
			axis_buttons[i][0].prev_physical_state = axis_buttons[i][0].current_state;
			axis_buttons[i][1].prev_physical_state = axis_buttons[i][1].current_state;
			axis_buttons[i][2].prev_physical_state = axis_buttons[i][2].current_state;

			// get new button's states
			if (p_dev_config->axis_config[i].button1 >= 0)
			{
				axis_buttons[i][0].current_state = logical_buttons_state[p_dev_config->axis_config[i].button1].current_state;
			}
			if (p_dev_config->axis_config[i].button2 >= 0)
			{
				axis_buttons[i][1].current_state = logical_buttons_state[p_dev_config->axis_config[i].button2].current_state;
			}
			if (p_dev_config->axis_config[i].button3 >= 0)
			{
				axis_buttons[i][2].current_state = logical_buttons_state[p_dev_config->axis_config[i].button3].current_state;
			}	
			
			// Trimming by buttons
			if (inc_button_num > 0 || rst_button_num > 0 || dec_button_num > 0)
			{
				// increment
				if ((inc_button_num & 0x01) && axis_buttons[i][0].current_state > axis_buttons[i][0].prev_physical_state)
				{
					axis_buttons[i][0].time_last = millis + 500;
					axis_trim_value[i] +=  (AXIS_FULLSCALE)/p_dev_config->axis_config[i].divider;
				}
				else if ((inc_button_num & 0x01) && axis_buttons[i][0].prev_physical_state && millis - axis_buttons[i][0].time_last > 50)
				{
					axis_buttons[i][0].time_last = millis;
					axis_trim_value[i] += (AXIS_FULLSCALE)/p_dev_config->axis_config[i].divider;
				}
				else if ((inc_button_num & 0x04) && axis_buttons[i][2].current_state > axis_buttons[i][2].prev_physical_state)
				{
					axis_buttons[i][2].time_last = millis + 500;
					axis_trim_value[i] +=  (AXIS_FULLSCALE)/p_dev_config->axis_config[i].divider;
				}
				else if ((inc_button_num & 0x04) && axis_buttons[i][2].prev_physical_state && millis - axis_buttons[i][2].time_last > 50)
				{
					axis_buttons[i][2].time_last = millis;
					axis_trim_value[i] += (AXIS_FULLSCALE)/p_dev_config->axis_config[i].divider;
				}
						
				// decrement
				if ((dec_button_num & 0x01) && axis_buttons[i][0].current_state > axis_buttons[i][0].prev_physical_state)
				{
					axis_buttons[i][0].time_last = millis + 500;
					axis_trim_value[i] -=  (AXIS_FULLSCALE)/p_dev_config->axis_config[i].divider;
				}
				else if ((dec_button_num & 0x01) && axis_buttons[i][0].prev_physical_state && millis - axis_buttons[i][0].time_last > 50)
				{
					axis_buttons[i][0].time_last = millis;
					axis_trim_value[i] -= (AXIS_FULLSCALE)/p_dev_config->axis_config[i].divider;
				}
				else if ((dec_button_num & 0x04) && axis_buttons[i][2].current_state > axis_buttons[i][2].prev_physical_state)
				{
					axis_buttons[i][2].time_last = millis + 500;
					axis_trim_value[i] -=  (AXIS_FULLSCALE)/p_dev_config->axis_config[i].divider;
				}
				else if ((dec_button_num & 0x04) && axis_buttons[i][2].prev_physical_state && millis - axis_buttons[i][2].time_last > 50)
				{
					axis_buttons[i][2].time_last = millis;
					axis_trim_value[i] -= (AXIS_FULLSCALE)/p_dev_config->axis_config[i].divider;
				}
				
				// reset
				if ((rst_button_num & 0x01) && axis_buttons[i][0].current_state)
				{
					axis_trim_value[i] = 0;
				}
				else if ((rst_button_num & 0x02) && axis_buttons[i][1].current_state)
				{
					axis_trim_value[i] = 0;
				}
				else if ((rst_button_num & 0x04) && axis_buttons[i][2].current_state)
				{
					axis_trim_value[i] = 0;
				}	
			}

			// Centering
		if (cent_button_num > 0)
		{
			if ((cent_button_num & 0x01) && axis_buttons[i][0].current_state)
			{
					axis_trim_value[i] = -tmp[i];
			}
			else if ((cent_button_num & 0x02) && axis_buttons[i][1].current_state)
			{
				axis_trim_value[i] = -tmp[i];
			}
			else if ((cent_button_num & 0x04) && axis_buttons[i][2].current_state)
			{
				axis_trim_value[i] = -tmp[i];
			}
		}
			
			if (tmp[i] + axis_trim_value[i] > AXIS_MAX_VALUE) 
			{
				axis_trim_value[i] = AXIS_MAX_VALUE - tmp[i];
				tmp[i] = AXIS_MAX_VALUE;
			}
			else if (tmp[i] + axis_trim_value[i] < AXIS_MIN_VALUE) 
			{
				axis_trim_value[i] = tmp[i] + AXIS_MIN_VALUE;
				tmp[i] = AXIS_MIN_VALUE;
			}
			else
			{
				tmp[i] += axis_trim_value[i];
			}
    }		
	} 
			//Centered Axis Indicators
			// PA8 for X-axis (axis 0)
			if (IsDynamicDeadbandHolding(scaled_axis_data[0], deadband_buffer[0], p_dev_config->axis_config[0].deadband_size) &&
			iabs(scaled_axis_data[0] - AXIS_CENTER_VALUE) < CENTER_LED_RANGE) // 
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_8);
			}
			else
			{
				GPIO_ResetBits(GPIOA, GPIO_Pin_8);
			}

			// PB0 for Y-axis (axis 1)
			if (IsDynamicDeadbandHolding(scaled_axis_data[1], deadband_buffer[1], p_dev_config->axis_config[1].deadband_size) &&
					iabs(scaled_axis_data[1] - AXIS_CENTER_VALUE) < CENTER_LED_RANGE)
			{
					GPIO_SetBits(GPIOB, GPIO_Pin_0);
			}
			else
			{
				GPIO_ResetBits(GPIOB, GPIO_Pin_0);
			}
	// Multi-axis process
	for (uint8_t i=0; i<MAX_AXIS_NUM; i++)
	{
		
		// check axis function activation 
		if(((axis_buttons[i][0].current_state && p_dev_config->axis_config[i].button1_type == AXIS_BUTTON_FUNC_EN) ||
			 (axis_buttons[i][1].current_state && p_dev_config->axis_config[i].button2_type == AXIS_BUTTON_FUNC_EN) ||
		   (axis_buttons[i][2].current_state && p_dev_config->axis_config[i].button3_type == AXIS_BUTTON_FUNC_EN) ||
		   (p_dev_config->axis_config[i].button1_type != AXIS_BUTTON_FUNC_EN && 
				p_dev_config->axis_config[i].button2_type != AXIS_BUTTON_FUNC_EN && 
				p_dev_config->axis_config[i].button3_type != AXIS_BUTTON_FUNC_EN)) && 
				p_dev_config->axis_config[i].function != NO_FUNCTION)
		{
			{
				switch (p_dev_config->axis_config[i].function)
				{
					case FUNCTION_PLUS:
						if (p_dev_config->axis_config[i].is_centered)
						{
							tmp[i] = tmp[i] + tmp[p_dev_config->axis_config[i].source_secondary];
						}
						else
						{
							tmp[i] = tmp[i] + tmp[p_dev_config->axis_config[i].source_secondary] - AXIS_MIN_VALUE;
						}
						break;

					case FUNCTION_MINUS:
						if (p_dev_config->axis_config[i].is_centered)
						{
							tmp[i] = tmp[i]/2 - tmp[p_dev_config->axis_config[i].source_secondary]/2;
						}
						else
						{
							tmp[i] = tmp[i] - tmp[p_dev_config->axis_config[i].source_secondary] + AXIS_MIN_VALUE;
						}
						
						break;

					case FUNCTION_EQUAL:
						tmp[i] = tmp[p_dev_config->axis_config[i].source_secondary];
						break;
					
					default:
						break;
				}
			}
			if (tmp[i] > AXIS_MAX_VALUE) tmp[i] = AXIS_MAX_VALUE;
			else if (tmp[i] < AXIS_MIN_VALUE) tmp[i] = AXIS_MIN_VALUE;
		}
		
    // setting technical axis data
    scaled_axis_data[i] = tmp[i];
		
		// Clamp near-zero to 0 to eliminate small residuals like -8
		if (scaled_axis_data[i] > -10 && scaled_axis_data[i] < 10) {
				scaled_axis_data[i] = 0;
		}

    // setting output axis data
    if (p_dev_config->axis_config[i].out_enabled)  out_axis_data[i] = scaled_axis_data[i];
    else  out_axis_data[i] = 0;
		
	}
	
}

/**
  * @brief  Resetting axis calibration values to the default
	*	@param	p_dev_config: Pointer to device configuration structure
	*	@param	axis_num: Number of axis 
  * @retval None
  */
void AxisResetCalibration (dev_config_t * p_dev_config, uint8_t axis_num)
{
	p_dev_config->axis_config[axis_num].calib_max = AXIS_MIN_VALUE;
	p_dev_config->axis_config[axis_num].calib_center = AXIS_CENTER_VALUE;
	p_dev_config->axis_config[axis_num].calib_min = AXIS_MAX_VALUE;
}

/**
  * @brief  Getting axis data in report format
	*	@param	out_data: Pointer to target buffer of axis output data (may be disabled in configuration)
	*	@param	scaled_data: Pointer to target buffer of axis scaled output data
	*	@param	raw_data: Pointer to target buffer of axis raw output data 
  * @retval None
  */
void AnalogGet (analog_data_t * out_data, analog_data_t * scaled_data, analog_data_t * raw_data)
{
	if (scaled_data != NULL)
	{
		memcpy(scaled_data, scaled_axis_data, sizeof(scaled_axis_data));
	}
	if (raw_data != NULL)
	{
		memcpy(raw_data, raw_axis_data, sizeof(raw_axis_data));
	}
	if (raw_data != NULL)
	{
		memcpy(out_data, out_axis_data, sizeof(raw_axis_data));
	}
}




