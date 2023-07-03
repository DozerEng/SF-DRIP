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


#include <stdbool.h>
#include <stdint.h>
#include "error.h"
#include "fastcodeUtil.h"
#include "queue.h"




//*************************************************
//Defines
//*************************************************



//*************************************************
//Types
//*************************************************

#define ERROR_QUEUE_LENGTH 100

//*************************************************
//Variables
//*************************************************
static uint32_t fastQueue[ERROR_QUEUE_LENGTH];
static uint32_t fastBackIndex = 0;
static uint32_t fastFrontIndex = 0;
static uint32_t slowQueue[ERROR_QUEUE_LENGTH];
static uint32_t slowBackIndex = 0;
static uint32_t slowFrontIndex = 0;
static uint32_t m_seed = 0;


//*************************************************
//function prototypes
//*************************************************


void fastLog(uint32_t eWord);
void slowLog(uint32_t eWord);
uint32_t makeErrorWord(ErrorType type, uint8_t subType);
/**
 * must be called once to ensure error IDs work
 */
void seed();
/**
 * @return the next seed in the series
 */
uint32_t nextSeed(void);




//*************************************************
//Code
//*************************************************





/**
 * can be called from anywhere to log an error
 */
void logError(ErrorType type, uint8_t subType){
	uint32_t ew = makeErrorWord(type, subType);
	if(isInFastCode()){
		fastLog(ew);
	} else {
		slowLog(ew);
	}
}

/**
 * retrieves currently indexed error from the queue
 * @return the current error. Zero if there is none.
 */
uint32_t getCurrentError(){
	uint32_t* result = 0;
	if(isQueueNotEmpty(&fastFrontIndex, &fastBackIndex, ERROR_QUEUE_LENGTH)){
		result = &fastQueue[fastFrontIndex];
	} else if(isQueueNotEmpty(&slowFrontIndex, &slowBackIndex, ERROR_QUEUE_LENGTH)){
		result = &slowQueue[slowFrontIndex];
	}

	if(result != 0 && (*result & 0xffff) == 0){
		seed();
		*result |= nextSeed();
	}
	return result == 0 ? 0 : *result;

}



/**
 * removes the current error from the queue and switches to the next in line
 */
void clearCurrentError(){
	if(isQueueNotEmpty(&fastFrontIndex, &fastBackIndex, ERROR_QUEUE_LENGTH)){
		doneWithQueueFront(&fastFrontIndex, &fastBackIndex, ERROR_QUEUE_LENGTH);
	} else if(isQueueNotEmpty(&slowFrontIndex, &slowBackIndex, ERROR_QUEUE_LENGTH)){
		doneWithQueueFront(&slowFrontIndex, &slowBackIndex, ERROR_QUEUE_LENGTH);
	} else {
		//currentErrorWord = 0;
	}
}

void fastLog(uint32_t eWord){
	if(isQueueNotFull(&fastFrontIndex, &fastBackIndex, ERROR_QUEUE_LENGTH)){
		fastQueue[fastBackIndex] = eWord;
		justAddedToQueueBack(&fastFrontIndex, &fastBackIndex, ERROR_QUEUE_LENGTH);
	}
}
void slowLog(uint32_t eWord){
	if(isQueueNotFull(&slowFrontIndex, &slowBackIndex, ERROR_QUEUE_LENGTH)){
			slowQueue[slowBackIndex] = eWord;
			justAddedToQueueBack(&slowFrontIndex, &slowBackIndex, ERROR_QUEUE_LENGTH);
		}
}

uint32_t makeErrorWord(ErrorType type, uint8_t subType){
	uint32_t result = ((uint32_t)subType)<<16;
	result |= ((uint32_t)type & 0xff)<<24;
	result |= nextSeed() & 0xffff;

	return result;
}

/**
 * must be called once to ensure error IDs work
 */
void seed(){
	if(m_seed == 0){
		m_seed = getTimeInMicroSeconds() & 0xffff;
	}
}
uint32_t nextSeed(void){
	uint32_t result = m_seed;
	if(m_seed != 0){
		++m_seed;
		m_seed &= 0xffff;
	}
	return result;
}
