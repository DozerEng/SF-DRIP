/*
Copyright (c) 2017-2018 STARFISH PRODUCT ENGINEERING INC.

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

//*************************************************
//includes
//*************************************************

#include "dac.h"
#include "ports.h"
#include "stm32f4xx_dac.h"
#include <math.h>



//*************************************************
//defines
//*************************************************

#define DAC1_PIN GPIO_A4_PIN
#define DAC2_PIN GPIO_A5_PIN

//*************************************************
//Types
//*************************************************

//*************************************************
//Variables
//*************************************************

static uint16_t value1;
static uint16_t value2;
static bool enable1;
static bool enable2;
//*************************************************
//function prototypes
//*************************************************

//*************************************************
//code
//*************************************************



/**
 * initialize the driver
 */
void dacInit(void){
	//DAC_Cmd(DAC_Channel_1, ENABLE);
	//DAC_Cmd(DAC_Channel_2, ENABLE);

	DAC_InitTypeDef ditd;
	DAC_StructInit(&ditd);

	ditd.DAC_Trigger = DAC_Trigger_None;//DAC_Trigger_Software;//
	ditd.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

	DAC_Init(DAC_Channel_1, &ditd);
	DAC_Init(DAC_Channel_2, &ditd);

}
/**
 * this should run in fast code
 */
void dacFastCode(void){
	//DAC_SetDualChannelData(DAC_Align_12b_R, value2, value1);
	//DAC_DualSoftwareTriggerCmd(ENABLE);
	if(isPinDir(DAC1_PIN, PIN_AN) && enable1){
		DAC->CR |= DAC_CR_EN1;
//		DAC_Cmd(DAC_Channel_1, ENABLE);
		DAC_SetChannel1Data(DAC_Align_12b_R, value1);
		DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
	} else {
		DAC->CR &= ~DAC_CR_EN1;
//		DAC_Cmd(DAC_Channel_1, DISABLE);
	}
	if(isPinDir(DAC2_PIN, PIN_AN) && enable2){
		DAC->CR |= DAC_CR_EN2;
//		DAC_Cmd(DAC_Channel_2, ENABLE);
		DAC_SetChannel2Data(DAC_Align_12b_R, value2);
		DAC_SoftwareTriggerCmd(DAC_Channel_2, ENABLE);
	} else {
		DAC->CR &= ~DAC_CR_EN2;
//		DAC_Cmd(DAC_Channel_2, DISABLE);
	}

}

void setDac1Value(uint16_t v){
	if(finitef(v)){
		value1 = v;
//		enable1 = true;
	} else {
		value1 = 0;
//		enable1 = false;
	}

}

void setDac2Value(uint16_t v){
	if(finitef(v)){
		value2 = v;
//		enable2 = true;
	} else {
		value2 = 0;
//		enable2 = false;
	}
}


void setDacAsFloat(uint32_t channel, float value){
	float dv = value * 4095.0f;
	if(dv > 4095){
		dv = 4095;
	} else if(dv < 0){
		dv = 0;
	}
	uint16_t v = (uint16_t)dv;

	if(channel == 0){
		setDac1Value(v);
	} else {
		setDac2Value(v);
	}
}

bool isDacEnable(uint32_t channel){
	bool result;
	if(channel == 0){
		result = enable1;
	} else {
		result = enable2;
	}
	return result;
}
void setDacEnable(uint32_t channel, bool e){
	if(channel == 0){
		enable1 = e;
	} else {
		enable2 = e;
	}
}
