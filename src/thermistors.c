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

#include <math.h>
#include <thermistors.h>
#include "adcManager.h"

//***************************************
//Defines

#define NUM_THERMISTOR_CHANNELS 8
#define SIZE 256
#define ADC_BITS 12
#define STEP (((float)(1<<12))/SIZE)
#define ONEOVERSTEP (1.0f/STEP)





#define KELVIN (273.16f)
//#define ONEOVER25C (1.0f/(KELVIN + 25.0f))

//***************************************
//Variables

static float lookup[SIZE];

static float m_paramC0[NUM_THERMISTOR_CHANNELS];
static float m_paramC1[NUM_THERMISTOR_CHANNELS];
static float m_paramC2[NUM_THERMISTOR_CHANNELS];
static float m_paramC3[NUM_THERMISTOR_CHANNELS];

uint8_t m_adcChannel[NUM_THERMISTOR_CHANNELS];


//***************************************
//Function prototypes

static float calcLog(float adc);
static float lookupLog(float adc);
static uint32_t getLutIndex(float adc);

//*****************************************
//Code


uint32_t getLutIndex(float adc){
	uint32_t result = (uint32_t)(adc*SIZE);
	if(result >= SIZE){
		result = SIZE - 1;
	}
	return result;
}

float calcLog(float adc){
	float rRatio = adc/(1.0f - adc);
	float result;
	if(rRatio <= 0.0f){
		result = INFINITY;
	} else {
		result = logf(rRatio);
	}

	 return result;
}

float lookupLog(float adc){
	uint32_t i = getLutIndex(adc);
	uint32_t j = (i + 1 >= SIZE) ? i : i + 1;
	float fi = adc*SIZE;
	float ffi = (float)i;





	float cr = (fi - ffi);
	float fr = 1.0f - cr;
	float lf = lookup[i];
	float lc = lookup[j];
	float result = lf*fr + lc*cr;

	return result;

}


/**
 * must be called before using this module.
 * This computes the lookup tables
 */
void initThermistors(void){
//	uint32_t i;
	for(float adc = 0.0f; adc < 1.0f; adc += 1.0f/SIZE){


		lookup[getLutIndex(adc)] = calcLog(adc);
	}

	for(uint8_t i = 0; i < NUM_THERMISTOR_CHANNELS; ++i){
		m_paramC0[i] = NAN;
		m_adcChannel[i] = 0;
	}
}

/**
 * This sets up ADC channels to work with thermistors. It is assumed that the thermistor is wired to ground with a pull-up resistor. The pull-up shall be the same value as the R0 value of the thermistor. R0 is usually the thermistor value at 25 C, although the coefficients can be calculated for any known value
 * Each thermistor is configured with  the coefficients of a modified Steinhart-Hart equation.
 * 1/T = c0 + c1*ln(Rt/R0) + c2*ln(Rt/R0)^2 + c3*ln(Rt/R0)^3
 * @param index - the temperature channel that's being configured. Can be from 0 to 7
 * @param adcChannel - the adc channel that will be used as an input
 * @param c0 - the 1st term parameter. If setting up with beta then c0 = 1/T0 - typically (1.0f/298.15f)
 * @param c1 - the 2nd term parameter. If setting up with beta then c1 = 1/beta - typically (1.0f/3380.0f)
 * @param c2 - the 3rd term parameter. If setting up with beta then c2 = 0.0f
 * @param c3 - the 4th term parameter. If setting up with beta then c3 = 0.0f
 */
void setupThermistor(uint8_t index, uint8_t adcChannel, float c0, float c1, float c2, float c3){
	if(index >= NUM_THERMISTOR_CHANNELS){
		return;
	}
	m_adcChannel[index] = adcChannel;
	m_paramC0[index] = c0;
	m_paramC1[index] = c1;
	m_paramC2[index] = c2;
	m_paramC3[index] = c3;
}



/**
 * @return the temperature of the specified channel in Celsius
 */
float getThermistorTemperature(uint8_t index){
	if(index >= NUM_THERMISTOR_CHANNELS){
		return NAN;
	}
	float adc = getAdcValue(m_adcChannel[index]);

	float log = lookupLog(adc);

	float c0 = m_paramC0[index];
	float c1 = m_paramC1[index];
	float c2 = m_paramC2[index];
	float c3 = m_paramC3[index];

	float result = 0;
	result = c3;
	result *= log;
	result += c2;
	result *= log;
	result += c1;
	result *= log;
	result += c0;
	result = 1.0 / result;
	result -= KELVIN;

	return  result;
}
uint32_t getThermistorCount(void){
	return NUM_THERMISTOR_CHANNELS;
}
