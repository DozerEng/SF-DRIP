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

//********************************************************
//Includes
//********************************************************



#include "vRailMonitor.h"
//#include "adc.h"
#include "adcManager.h"
#include "fastcodeUtil.h"


//********************************************************
//Defines
//********************************************************


//********************************************************
//Type Definitions
//********************************************************

#define VRAIL_SAMPLE_TIME_US 5000
#define VRAIL_SAMPLE_TIME_S (1e-6*(float)VRAIL_SAMPLE_TIME_US)
#define VRAIL_SCALE ((10.0f + 63.4f)/10.0f*3.3f)
//********************************************************
//Variable Definitions
//********************************************************

static float m_vRail = 0.0f;
static uint32_t m_timer = 0;


//********************************************************
//Function Prototypes
//********************************************************



//********************************************************
//Code
//********************************************************



void vRailMonitorInit(void){

}
void vRailMonitorSlowCode(void){
	if(slowTimer(&m_timer, VRAIL_SAMPLE_TIME_US)){
		setAdcParams(13, 0.5, 1, 0);
		m_vRail = getAdcValue(13);
//		queueAdcConversion(ADC1, 13, &m_vRail, 0, VRAIL_SAMPLE_TIME_S*1);
	}
}
float getVRail(void){
	return m_vRail*VRAIL_SCALE;
}
