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
//notes
//*************************************************
//disabled a ton of code here because this needs to get redone for new SPI routine


//*************************************************
//includes
//*************************************************

#include <fastcode.h>
#include <stdbool.h>
#include "stm32f4xx_spi.h"
#include "spi.h"
#include "ports.h"
#include "filter.h"
#include <max11629.h>

//***********************
//Function Prototypes

//**************************









//static SpiTransaction st[40];
static float values[8];



//static uint32_t firstTimeCount = 0;

//static uint32_t timerReg = 0;

static uint32_t timeoutTimerReg = 0;
static uint8_t maxSpiIndex = 0;
//static bool init = true;

/**
 * * this should be called regularly in slow code
 */
void max11629SlowCode(){

//	//timeout current state if not idle for too long
//	if(slowTimer(&timeoutTimerReg, 5000L)){
//
//		//ClockSpec cs = CPOL_1_CPHA_1;
//		//ClockSpec cs = CPOL_1_CPHA_0;
//		ClockSpec cs = CPOL_0_CPHA_0;
//		BaudDiv bd = DIV_64;
//		BitNum bn = BITS_8;
//
//		uint8_t i = 0;
//
//
//
//
//		//queueSpiTransaction(spiConfig(&st1, SPI2, ADC_CS_PIN, 0b00010000, CPOL_1_CPHA_1, false));//Reset register: reset all regs
//		//read in all data
//		queueSpiTransaction(spiConfig(&st[i++], SPI2, ADC_CS_PIN, 0b00100000, bn, cs, bd, true));//Averaging register: off
//		queueSpiTransaction(spiConfig(&st[i++], SPI2, ADC_CS_PIN, 0b01110100, bn, cs, bd, true));//Setup register: clock mode 11, internal ref
//
//		for(uint8_t c = 0<<3; c < 8<<3; c += 1<<3){
//
//			//configure adc and trigger the next conversion
//
//			queueSpiTransaction(spiConfig(&st[i++], SPI2, ADC_CS_PIN, 0b10000110 | c, bn, cs, bd, false));//Conversion register: set to channel 0
//
//			queueSpiTransaction(spiConfig(&st[i++],  SPI2, ADC_CS_PIN, 0b00000000, bn, cs, bd, false));//Conversion register: set to scan from 0 to 7
//			queueSpiTransaction(spiConfig(&st[i++],  SPI2, ADC_CS_PIN, 0b00000000, bn, cs, bd, true));//Conversion register: set to scan from 0 to 7
//
//
//		}
//		maxSpiIndex = i - 1;
//
//
//
//
//
//
//
//
//
//	}
//	if(isSpiComplete(&st[maxSpiIndex])){
//		maxSpiIndex = 0;
//		uint8_t ii = 3;
//		for(uint8_t i = 0; i < 8; ++i){
//
//			float f =   ((float)(st[ii].rxData))*62.5152625e-3f;//this is 256/4095
//			f += ((float)st[ii + 1].rxData)*244.2e-6;// and 1/4095
//
//			float k = 0.005;
//			values[i] += (f - values[i])*k;
//			//values[i] = f;
//			if(values[i] != values[i]){
//				values[i] = 0.0f;
//			}
//			ii += 3;
//
//
//		}
//
//	}
//

}
/**
 * this returns the an adc value
 */
float getAdcVal(uint8_t i){
	return values[i];
}



