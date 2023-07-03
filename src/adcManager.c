/*
Copyright (c) 2019 STARFISH PRODUCT ENGINEERING INC.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
//#include "adc.h"
#include "fastcodeUtil.h"
//#include "gpio.h"
#include "ports.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "adcManager.h"
#include "sinkSource.h"



//********************************************************
//Defines
//********************************************************
#define ADC_NUM ADC1 //this is the adc that is used for all conversions
#define ADC_CHANNELS 16
//#define ADC_SAMPLE_PERIOD_SECONDS SECONDS_PER_FASTLOOP
#define ADC_SAMPLE_TIME ADC_SampleTime_15Cycles //ADC_SampleTime_15Cycles

#define DMA_STREAM DMA2_Stream4


//********************************************************
//Type Definitions
//********************************************************
typedef struct {
//	ADC_TypeDef* adc;
//	uint8_t channel;
	float value;
	bool enabled;
	float timeConstantCoeff;
	float scale;
	float offset;

	SinkSource sink;
} AdcChannel;

//********************************************************
//Variable Definitions
//********************************************************


static DMA_InitTypeDef ditd;
static ADC_InitTypeDef aitd;
static ADC_CommonInitTypeDef acitd;
static AdcChannel m_adcs[ADC_CHANNELS];

static uint32_t m_mask = 0x0000;
//static float m_values[ADC_CHANNELS];
//static bool m_enabled[ADC_CHANNELS];
//static float m_timeConstantCoeff[ADC_CHANNELS];
//static float m_scales[ADC_CHANNELS];
//static float m_offsets[ADC_CHANNELS];
static uint16_t m_dmaArray[ADC_CHANNELS];
//static SinkSource m_sinks[ADC_CHANNELS];
static uint32_t m_indexNum = 0;
static bool m_adcStructsSetup = false;
static uint32_t m_adcSetupTimer = 0;

//********************************************************



//********************************************************
//Function Prototypes
//********************************************************

void adcProcessOneChannel(uint32_t i);


//********************************************************


uint32_t getAdcChannelMask(void){
	return m_mask;
}


void setAdcChannelMask(uint32_t mask){
	m_mask = mask;

	uint32_t m = mask;
	uint32_t t = 0;
	for(uint32_t i = 0; i < ADC_CHANNELS; ++i){
		AdcChannel* a = &m_adcs[i];
		if(m & 0b1){
			a->enabled = true;
			++t;

		} else {
			a->enabled = false;
		}
		m >>= 1;

	}
	m_indexNum = t;
}
uint32_t getAdcCount(void){
	return ADC_CHANNELS;
}
//uint32_t getAdcChannelIndex(uint32_t index){
//	uint32_t result = 0;
//	if(index >= 0 && index < ADC_CHANNELS){
//		result = m_enabled[index];
//	}
//	return result;
//
//}
float getAdcValue(uint32_t index){
	float result = NAN;
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		if(a->enabled){
			result = a->value;

		}
	}
	return result;
}
//float getAdcValueFromMask(uint32_t mask){
//	uint32_t index = 0;
//	uint32_t m = mask;
//	for(uint32_t i = 0; i < ADC_CHANNELS; ++i){
//		if(m & 0b1){
//			index = i;
//			break;
//		}
//	}
//	return m_values[index];
//}
/**
 * sets up an adc channel
 * @param index the channel to setup
 * @param timeConstantSeconds the time constant for the low pass filter
 * @param scale the gain to apply to this channel
 * @param offset the value to subtract from the channel before applying the gain
 */
void setAdcParams(uint32_t index, float timeConstantSeconds, float scale, float offset){
	float k = getSecondsPerFastLoop()/timeConstantSeconds;//this is an approximation
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];

		a->timeConstantCoeff = k;
		a->scale = scale;
		a->offset = offset;

	}
}
void adcManagerInit(void){
	for(uint32_t i = 0; i < ADC_CHANNELS; ++i){
		AdcChannel* adc = &m_adcs[i];
		adc->enabled = false;
		adc->timeConstantCoeff = getSecondsPerFastLoop();//this essentially disables the TC
		adc->scale = 1.0f;
		adc->offset = 0.0f;
		adc->sink = NULL_SINK_SOURCE;

	}
}
void adcManagerSlowCode(void){


	if(slowTimer(&m_adcSetupTimer, 100*MILLISECONDS)){
		//setup a bunch of structs

		ditd.DMA_Channel = DMA_Channel_0;
		ditd.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC_NUM->DR));
		ditd.DMA_Memory0BaseAddr = (uint32_t)m_dmaArray;
		ditd.DMA_DIR = DMA_DIR_PeripheralToMemory;
		ditd.DMA_BufferSize = ADC_CHANNELS;
		ditd.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		ditd.DMA_MemoryInc = DMA_MemoryInc_Enable;
		ditd.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		ditd.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		ditd.DMA_Mode = DMA_Mode_Normal;
		ditd.DMA_Priority = DMA_Priority_VeryHigh;
		ditd.DMA_FIFOMode = DMA_FIFOMode_Disable;
		ditd.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		ditd.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		ditd.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;


		acitd.ADC_Mode = ADC_Mode_Independent;
		acitd.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
		acitd.ADC_Prescaler = ADC_Prescaler_Div2;
		acitd.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;


		aitd.ADC_Resolution = ADC_Resolution_12b;
		aitd.ADC_ContinuousConvMode = DISABLE;
		aitd.ADC_NbrOfConversion = 16;
		aitd.ADC_ScanConvMode = ENABLE;
		aitd.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		aitd.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
		aitd.ADC_DataAlign = ADC_DataAlign_Right;

		m_adcStructsSetup = true;
	}

}
bool isAdcEnabled(uint32_t index){
	return (index >= 0 && index < ADC_CHANNELS && m_adcs[index].enabled);
}


void adcProcessOneChannel(uint32_t i){
	AdcChannel* a = &m_adcs[i];
	if(a->enabled){
		float adc = (float)(m_dmaArray[i]);
		m_dmaArray[i] = 0xffff;
		adc *= 244.2002442e-6f;//this is 1/4095 make sure this is float or the calc is super expensive!
		adc -= a->offset;
		adc *= a->scale;
		if(finitef(a->value)){
			a->value += a->timeConstantCoeff*(adc - a->value);
		} else {
			a->value = adc;
		}

		setSinkSource(a->sink, adc);
	}
}
void adcManagerFastCode(void){
	//***** first check if last conversion was complete
	//***** if so harvest new data

	if(!m_adcStructsSetup){
		return;
	}

	for(uint32_t i = 0; i < ADC_CHANNELS; ++i){
		adcProcessOneChannel(i);
	}




	if(DMA_GetCmdStatus(DMA_STREAM) == DISABLE){
		//a scan is complete


	}


	if(DMA_GetCmdStatus(DMA_STREAM) == DISABLE){




		//***** set up next conversion in adc
		//disable converter and stuff prior to configuring


		ADC_NUM->CR2 &= ~(ADC_CR2_DMA | ADC_CR2_ADON);
		DMA_DeInit(DMA_STREAM);
		DMA_Cmd(DMA_STREAM, DISABLE);//first disable stream


		ADC_CommonInit(&acitd);






//		ADC_NUM->SMPR1 &= 0b11111 << 27;//make sure top bits are left the way they are
		ADC_NUM->SMPR1 =  0
				| (ADC_SAMPLE_TIME <<  0) //setup sample time for channel 10
				| (ADC_SAMPLE_TIME <<  3) //setup sample time for channel 11
		        | (ADC_SAMPLE_TIME <<  6) //setup sample time for channel 12
		        | (ADC_SAMPLE_TIME <<  9) //setup sample time for channel 13
		        | (ADC_SAMPLE_TIME << 12) //setup sample time for channel 14
		        | (ADC_SAMPLE_TIME << 15) //setup sample time for channel 15
//		        | (ADC_SAMPLE_TIME << 18) //setup sample time for channel 16
//		        | (ADC_SAMPLE_TIME << 21) //setup sample time for channel 17
//		        | (ADC_SAMPLE_TIME << 24) //setup sample time for channel 18
		;
//		ADC_NUM->SMPR2 &= 0b11 << 30;//make sure top bits are left the way they are
		ADC_NUM->SMPR2 =  0
				| (ADC_SAMPLE_TIME <<  0) //setup sample time for channel 0
				| (ADC_SAMPLE_TIME <<  3) //setup sample time for channel 1
				| (ADC_SAMPLE_TIME <<  6) //setup sample time for channel 2
				| (ADC_SAMPLE_TIME <<  9) //setup sample time for channel 3
				| (ADC_SAMPLE_TIME << 12) //setup sample time for channel 4
				| (ADC_SAMPLE_TIME << 15) //setup sample time for channel 5
				| (ADC_SAMPLE_TIME << 18) //setup sample time for channel 6
				| (ADC_SAMPLE_TIME << 21) //setup sample time for channel 7
				| (ADC_SAMPLE_TIME << 24) //setup sample time for channel 8
				| (ADC_SAMPLE_TIME << 27) //setup sample time for channel 9
		;

//		ADC_NUM->SQR1 &= 0b11111111 << 24;//make sure the top pins stay set. Clear all others
		ADC_NUM->SQR1 = 0
				| (ADC_Channel_12 << 0) //put this channel in sequence reg
				| (ADC_Channel_13 << 5) //put this channel in sequence reg
				| (ADC_Channel_14 << 10) //put this channel in sequence reg
				| (ADC_Channel_15 << 15) //put this channel in sequence reg
				| (15 << 20) //specify the number of channels in sequence
		;
//		ADC_NUM->SQR2 &= 0b11 << 30;//make sure the top pins stay set. Clear all others
		ADC_NUM->SQR2 = 0
				| (ADC_Channel_6 << 0) //put this channel in sequence reg
				| (ADC_Channel_7 << 5) //put this channel in sequence reg
				| (ADC_Channel_8 << 10) //put this channel in sequence reg
				| (ADC_Channel_9 << 15) //put this channel in sequence reg
				| (ADC_Channel_10 << 20) //put this channel in sequence reg
				| (ADC_Channel_11 << 25) //put this channel in sequence reg
		;
//		ADC_NUM->SQR3 &= 0b11 << 30;//make sure the top pins stay set. Clear all others
		ADC_NUM->SQR3 = 0
				| (ADC_Channel_0 << 0) //put this channel in sequence reg
				| (ADC_Channel_1 << 5) //put this channel in sequence reg
				| (ADC_Channel_2 << 10) //put this channel in sequence reg
				| (ADC_Channel_3 << 15) //put this channel in sequence reg
				| (ADC_Channel_4 << 20) //put this channel in sequence reg
				| (ADC_Channel_5 << 25) //put this channel in sequence reg
		;




		//***** set up DMA

		//now init DMA channel

//		DMA_SetCurrDataCounter(DMA_STREAM, 16);
		DMA_Init(DMA_STREAM, &ditd);





		//config ADC
		ADC_Init(ADC_NUM, &aitd);
		//enable DMA in ADC
		ADC_NUM->CR2 |= ADC_CR2_DMA;
//		ADC_DMACmd(ADC_NUM, ENABLE);
//		ADC_DMARequestAfterLastTransferCmd(ADC_NUM, DISABLE);
		DMA_Cmd(DMA_STREAM, ENABLE);
//		ADC_Cmd(ADC_NUM, ENABLE);
		ADC_NUM->CR2 |= ADC_CR2_ADON;

		ADC_NUM->CR2 |= ADC_CR2_SWSTART;



		//***** trigger conversion
//		ADC_SoftwareStartConv(ADC_NUM);



		//6.8us



	}

}

void setAdcEnabled(uint32_t index, bool enabled){
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		a->enabled = enabled;
		if(enabled){
			m_mask |= 1 << index;
		} else {
			m_mask &= ~(1 << index);
		}
	}
}

/**
 * sets up a sink for the specified channel's adc data
 * @param index the channel to setup
 * @param sink the sink to funnel the data to
 *
 */
void setAdcSink(uint32_t index, SinkSource sink){
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		a->sink = sink;
	}
}
/**
 * sets up the gain for the specified channel
 * @param index the channel to setup
 * @param sink the sink to funnel the data to
 *
 */
void setAdcGain(uint32_t index, float value){
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		a->scale = value;
	}
}
/**
 * sets up the dc offset for the specified channel
 * @param index the channel to setup
 * @param sink the sink to funnel the data to
 *
 */
void setAdcOffset(uint32_t index, float value){
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		a->offset = value;
	}
}
/**
 * sets up the time constant for the specified channel
 * @param index the channel to setup
 * @param sink the sink to funnel the data to
 *
 */
void setAdcTimeConstant(uint32_t index, float value){
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		a->timeConstantCoeff = value;
	}
}

SinkSource getAdcSink(uint32_t index){

	SinkSource result = NULL_SINK_SOURCE;
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		result = a->sink;
	}
	return result;
}
float getAdcGain(uint32_t index){
	float result = NAN;
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		result = a->scale;
	}
	return result;
}
float getAdcOffset(uint32_t index){
	float result = NAN;
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		result = a->offset;
	}
	return result;
}
void zeroAdc(uint32_t index, float currentValue){
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		a->offset -= currentValue/a->scale;
	}
}
float getAdcTimeConstant(uint32_t index){
	float result = NAN;
	if(index >= 0 && index < ADC_CHANNELS){
		AdcChannel* a = &m_adcs[index];
		result = a->timeConstantCoeff;
	}
	return result;
}
