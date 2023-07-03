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
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <stm32f4xx.h>
#include <stm32f4xx_tim.h>

#include <fastcodeUtil.h>
#include <ports.h>
#include <clockSetup.h>



//*************************************************
//defines
//*************************************************

#define TIM7_RELOAD_VAL 65535 //this is 5ms
#define TIM7_PRESCALE_VAL 9 //this makes it resolve to 100ns. prescale is divide by 9
#define TIM7_TO_SECONDS (((float)TIM7_PRESCALE_VAL)*(1.0f/90e6f))
#define STATS_TIME_CONSTANT_COEFF (1.0e-4f)

//*************************************************
//Types
//*************************************************


//*************************************************
//SFDQ Variables
//*************************************************




// Calculate a prescaler for fast code loop to get a millisecond count

//const uint32_t MICROSECONDS_PER_HOUR = 3600000000L;
// Define the speed at which the main interrupt is called in Hz
#define FAST_CODE_RATE__HZ   (TIMER_TICKS_PER_MS*1000)
static uint32_t m_time = 0;
static uint32_t m_loopCounts;

//static uint32_t msCount = 0;
static uint32_t m_minSysTick = 10000;

static uint32_t m_fastCodeHertz = 0;
static float m_secondsPerFastloop = 0;
static uint32_t m_microsecondsPerFastloop = 0;

static uint32_t m_fromHereTime = 0;
static bool m_fromHereStarted = false;
static bool inFastCode = false;
static volatile float m_maxTimeToHere;
static uint32_t lastCnt = 0;

static SinkSource m_fastWorstTimeSink = NULL_SINK_SOURCE;
static SinkSource m_fastAverageTimeSink = NULL_SINK_SOURCE;
static SinkSource m_slowWorstTimeSink = NULL_SINK_SOURCE;
static SinkSource m_slowAverageTimeSink = NULL_SINK_SOURCE;

//*************************************************
//function prototypes
//*************************************************

static void setHertz(float hertz);



//*************************************************
//code
//*************************************************



void checkSysTick(bool checkForOverrun){
	//test min value of the systick timer at the end of fastcode. The timer counts down so this represents the longest branch
	uint32_t t = (uint32_t)SysTick->VAL;
	if(t < m_minSysTick){
		m_minSysTick = t;
	}
}
/**
 * Performs execution time stats on slow code loop time.
 * This should be called once per slowcode loop.
 */
void slowCodeTimeCheck(void){

	static uint16_t lastCnt = 0;
	static float timeCheck = 0;
	static float worstCaseTimeCheck = 0;

	uint16_t cnt = TIM7->CNT;

	uint16_t delta = cnt - lastCnt;
	lastCnt = cnt;


	float t = ((float)delta)*TIM7_TO_SECONDS;
	timeCheck += (t  - timeCheck) * STATS_TIME_CONSTANT_COEFF;

	worstCaseTimeCheck *= 1.0f - STATS_TIME_CONSTANT_COEFF;
	if(worstCaseTimeCheck < t){
		worstCaseTimeCheck = t;
	}

	setSinkSource(m_slowWorstTimeSink, worstCaseTimeCheck);
	setSinkSource(m_slowAverageTimeSink, timeCheck);

}

/**
 * must be placed at the start of fastcode
 * this helps logError know whether it's being called from slow or fast code
 */
void dontPutAnyFastCodeBeforeThisFunction(void){


	static float worstCaseTimeCheck = 0;

	uint16_t cnt = TIM7->CNT;
	m_fromHereStarted = false;
	m_time += m_microsecondsPerFastloop;//increment the time counter


	uint16_t delta = cnt - lastCnt;
	lastCnt = cnt;

	float t = ((float)delta)*TIM7_TO_SECONDS;
//	timeCheck += (t  - timeCheck) * STATS_TIME_CONSTANT_COEFF;

	worstCaseTimeCheck *= 1.0f - STATS_TIME_CONSTANT_COEFF;
	if(worstCaseTimeCheck < t){
		worstCaseTimeCheck = t;
	}

	setSinkSource(m_fastWorstTimeSink, worstCaseTimeCheck);
//	setSinkSource(m_fastAverageTimeSink, timeCheck);

	inFastCode = true;

}
/**
 * must be placed at the end of fastcode
 * this helps logError know whether it's being called from slow or fast code
 */
void dontPutAnyFastCodeAfterThisFunction(void){
	uint16_t cnt = TIM7->CNT;
	static float timeCheck = 0;
	uint16_t delta = cnt - lastCnt;


	float t = ((float)delta)*TIM7_TO_SECONDS;
	timeCheck += (t  - timeCheck) * STATS_TIME_CONSTANT_COEFF;

	setSinkSource(m_fastAverageTimeSink, timeCheck);

	inFastCode = false;

}

void testFastcodeTimeFromHere(void){
	m_fromHereTime = TIM7->CNT;
	m_fromHereStarted = true;
}

void testFastcodeTimeToHere(void){
	if(!inFastCode){
		return;
	}
	if(!m_fromHereStarted){
		return;
	}
	uint32_t time = TIM7->CNT;
	uint32_t delta = time - m_fromHereTime;
	if(delta >= TIM7_RELOAD_VAL){
		delta += TIM7_RELOAD_VAL + 1;
	}
	float fDelta = ((float)delta)*TIM7_TO_SECONDS;
	if(fDelta > m_maxTimeToHere){
		m_maxTimeToHere = fDelta;

	}

	m_maxTimeToHere *= 0.99999;//allow it to decay a bit
	setSinkSource(m_slowAverageTimeSink, m_maxTimeToHere);
}





// Note: these functions currently only control the ISR attached to the timer interrupt
// If other interrupts are ever added to the program, they should also be controlled from here
//  or find a global interrupt control switch to set/clear here instead
//void enableTimerInterrupt()  {
//	SysTick->CTRL |=  SysTick_CTRL_TICKINT_Msk;
//}
//void disableTimerInterrupt() {
//	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
//}

// Timer/Counter register set-values
// see section 4.4.8 of ST document PM0214
#define SYSTICK_SCB_SHP_INDEX      11
#define SYSTICK_SCB_SHP_SET_VALUE  0xF0 // highest priority
#define SYSTICK_CTRL_ENABLE_MASK   (SysTick_CTRL_CLKSOURCE_Msk /* use processor clock (AHB) */ \
                                  | SysTick_CTRL_ENABLE_Msk)   /* counter enabled */
//                                | SysTick_CTRL_TICKINT_Msk)  /* enables/disables the interrupt associated with SysTick - do this separately, when ready */

void initialiseFastCode(float hertz){
	inFastCode = false;
	//   This is the period count for the timer generating the ISR.
//   LOOP_COUNTS = ((system_frequency + (ISR_frequency/2)) / ISR_frequency) -1
//   where:
//   - system_frequency is the rate at which the internal CPU instructions are clocked
//   - ISR-frequency is the desired frequency for triggering the ISR
//   - + (ISR_frequency/2) provides up/down rounding for the division (as opposed to floor rounding) - only required for a C expression (not your calculator)
//   - -1 because timers count for a period which is one-higher than the value set in their count registers
	m_loopCounts = (uint32_t)roundf(getSystemClockHz() / hertz);
	setHertz(hertz);
    // The following commented-out line has been replaced with direct register programming (subsequent code)
	SysTick_Config(m_loopCounts);
	SysTick->CTRL |=  SysTick_CTRL_TICKINT_Msk;
//	if (m_loopCounts <= SysTick_LOAD_RELOAD_Msk)                      // Reload value must not exceed 0xFFFFFF
//	{
//		SCB->SHP[SYSTICK_SCB_SHP_INDEX] = SYSTICK_SCB_SHP_SET_VALUE; // set Priority for Cortex-M  SysTickSystem Interrupt
//		SysTick->LOAD  = m_loopCounts;                                // set reload register (set-value is one less than the actual number of ticks (see above))
//		SysTick->VAL   = 0;                                          // reset the SysTick Counter Value
//		SysTick->CTRL |= SYSTICK_CTRL_ENABLE_MASK;                   // Configure SysTick clock/counter
//		// Don't enable the SysTick timer yet. This is done from main
//	}


	//setup TIM7 to see if we're keeping correct time
	TIM_PrescalerConfig(TIM7, (TIM7_PRESCALE_VAL - 1), TIM_PSCReloadMode_Immediate);
	TIM_SetAutoreload(TIM7, TIM7_RELOAD_VAL);
	TIM_Cmd(TIM7, ENABLE);


}
uint32_t getTimeInMicroSeconds(void){
	return m_time;
}
/**
 * used by slow code to make a period timer
 * This will return true every time the specified number of microseconds has passed
 * @param timeReg a register that this timer uses for it's state
 * @param microseconds the period of this timer
 * @returns true just as another period has elapsed, false otherwise
 */
bool slowTimer(uint32_t* timeReg, uint32_t microseconds){
	bool result = false;

	uint32_t t = getTimeInMicroSeconds();
	if((t - *timeReg) >= microseconds){
		*timeReg = t;
		result = true;
	}
	return result;

}
void resetTimeInMicroSeconds(void){
	m_time = 0;
}

void resetSlowTimer(uint32_t* timeReg){
	*timeReg = getTimeInMicroSeconds();
}

bool isInFastCode(void){
	return inFastCode;
}
/**
 * @return the maximum SysTick timer value at the end of fastcode
 */
uint32_t getMaxSysTick(void){
	return m_minSysTick;
}
/**
 * compares two microsecond integer times. Result is positive if t1>t2
 * @return time in seconds.
 */
float compareTimeMicroSec(uint32_t t1, uint32_t t2){
		uint32_t uerror = t1 - t2;
		//top bits should be either 11 or 00
		//if they are 10 or 01 then the number is bad
		//that means the number should not be
		//greater than or e


		if(uerror <= 0x3fffffff || //top bits are 0b00
				uerror >= 0xc0000000){//top bits are 0b11

		} else {
			//otherwise flip the top bit because we've likely overflowed
			uerror ^= 1<<31;
		}
		//dereference and cast to use as signed
		int32_t error = *((int32_t*)(&uerror));
		return ((float)error)*1e-6f;
}

/**
 * compares specified microsecond timer value to the time now.
 * @return time in seconds, positive if time < now
 */
float compareTimeToNow(uint32_t t){
	return compareTimeMicroSec(getTimeInMicroSeconds(), t);
}
static void setHertz(float hertz){
	m_fastCodeHertz = (uint32_t)roundf(hertz);
	float f = 1.0f / hertz;
	m_secondsPerFastloop = f;
	m_microsecondsPerFastloop = (uint32_t)roundf(1e6 / hertz);
}

float getSecondsPerFastLoop(void){
	return m_secondsPerFastloop;
}
float getSecondsPerBranch(void){
	return m_secondsPerFastloop*2.0f;

}
uint32_t getMicrosecondsPerFastLoop(void){
	return m_microsecondsPerFastloop;
}
float getFastloopMaxTime(void){
	return m_maxTimeToHere;
}
/**
 * sets the sinks for the various time measurements
 * @param index 0 -> worst fastcode time; 1 -> average fastcode time; 2 -> worst slowcode time; 3 -> average slowcode time
 */
void setFastcodeSink(uint32_t index, SinkSource sink){
	switch(index){
	case 0:
		m_fastWorstTimeSink = sink;
		break;
	case 1:
		m_fastAverageTimeSink = sink;
		break;
	case 2:
		m_slowWorstTimeSink = sink;
		break;
	case 3:
		m_slowAverageTimeSink = sink;
		break;
	}
}
/**
 * gets the sinks for the various time measurements
 * @param index 0 -> worst fastcode time; 1 -> average fastcode time; 2 -> worst slowcode time; 3 -> average slowcode time
 */
SinkSource getFastcodeSink(uint32_t index){
	SinkSource result = NULL_SINK_SOURCE;
	switch(index){
	case 0:
		result = m_fastWorstTimeSink;
		break;
	case 1:
		result = m_fastAverageTimeSink;
		break;
	case 2:
		result = m_slowWorstTimeSink;
		break;
	case 3:
		result = m_slowAverageTimeSink;
		break;
	}
	return result;
}

