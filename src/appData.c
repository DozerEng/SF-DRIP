/*
Copyright (c) 2020 STARFISH PRODUCT ENGINEERING INC.

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
//this module implements a blackboard pattern for use within the application
//and also from controller to peripheral
//there are two blackboards. One will be written by the controller
//the other will be written by the peripheral.
//this is not enforced within this module as the getters and setters for each blackboard are accessible
//anywhere.


//*************************************************
//includes
//*************************************************
#include "appData.h"
#include <math.h>
#include <stddef.h>

//*************************************************
//defines
//*************************************************


//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************

static float *controllerBlackboard = NULL;//pointer to app data array start
static uint32_t controllerBlackboardLength = 0; //number of elements of app data
static float *peripheralBlackboard = NULL;//pointer to app data array start
static uint32_t peripheralBlackboardLength = 0; //number of elements of app data

//*************************************************
//function prototypes
//*************************************************


//*************************************************
//code
//*************************************************

void initAppData(float* txData, uint32_t txLength, float* rxData, uint32_t rxLength){
	peripheralBlackboard = txData;
	peripheralBlackboardLength = txLength;
	controllerBlackboard = rxData;
	controllerBlackboardLength = rxLength;
}
void setRxAppData(uint32_t index, float value){
	if(index < controllerBlackboardLength){
		controllerBlackboard[index] = value;
	}
}
void setTxAppData(uint32_t index, float value){
	if(index < peripheralBlackboardLength){
		peripheralBlackboard[index] = value;
	}
}
float getRxAppData(uint32_t index){
	float result = NAN;

	if(index < controllerBlackboardLength){
		result = *(controllerBlackboard + index);
	}
	return result;
}
float getTxAppData(uint32_t index){
	float result = NAN;

	if(index < peripheralBlackboardLength){
		result = *(peripheralBlackboard + index);
	}
	return result;
}
uint32_t getTxAppDataLength(void){
	return peripheralBlackboardLength;
}
uint32_t getRxAppDataLength(void){
	return controllerBlackboardLength;
}

/**
 * casts the 32 bit value as a uint32 instead of a float32.
 * Note that this is not the same as (uint32_t)getRxAppData(index)
 */
uint32_t getRxAppDataAsInt(uint32_t index){
	uint32_t result = -1;

		if(index < controllerBlackboardLength){
			result = *(((uint32_t*)controllerBlackboard) + index);
		}
		return result;
}
/**
 * casts the 32 bit value as a uint32 instead of a float32.
 * Note that this is not the same as (uint32_t)getTxAppData(index)
 */
uint32_t getTxAppDataAsInt(uint32_t index){
	uint32_t result = -1;

	if(index < peripheralBlackboardLength){
		result = *(((uint32_t*)peripheralBlackboard) + index);
	}
	return result;
}

