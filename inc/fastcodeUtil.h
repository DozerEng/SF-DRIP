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
#ifndef FASTCODEUTIL_H_
#define FASTCODEUTIL_H_

#include <stdbool.h>
#include <stdint.h>
#include <sinkSource.h>



//#define TIMER_TICKS_PER_MS (20)
//#define MICROSECONDS_PER_FASTLOOP (1000/TIMER_TICKS_PER_MS)
//#define SECONDS_PER_FASTLOOP ((1e-6)*((float)MICROSECONDS_PER_FASTLOOP))
//#define SECONDS_PER_BRANCH (SECONDS_PER_FASTLOOP*2)
#define SECONDS 1000000
#define MILLISECONDS 1000
#define MINUTES (60*SECONDS)

void initialiseFastCode(float hertz);
uint32_t getTimeInMicroSeconds(void);
void resetTimeInMicroSeconds(void);
void resetSlowTimer(uint32_t* timeReg);
bool isInFastCode(void);
void dontPutAnyFastCodeBeforeThisFunction(void);
void dontPutAnyFastCodeAfterThisFunction(void);
void checkSysTick(bool checkForOverrun);
/**
 * Performs execution time stats on slow code loop time.
 * This should be called once per slowcode loop.
 */
void slowCodeTimeCheck(void);
/**
 * This function works with the next function to measure elapsed time in fastcode
 * This is only for debugging
 */
void testFastcodeTimeFromHere(void);
/**
 * this is a function for testing execution times in fastcode.
 * This should only be called from fastcode
 */
void testFastcodeTimeToHere(void);
/**
 * compares two microsecond integer times.
 * @return time in seconds, positive if t1>t2.
 */
float compareTimeMicroSec(uint32_t t1, uint32_t t2);
/**
 * compares specified microsecond timer value to the time now.
 * @return time in seconds, positive if time < now
 */
float compareTimeToNow(uint32_t t);

/**
 * used by slow code to make a period timer
 * This will return true every time the specified number of microseconds has passed
 * @param timeReg a register that this timer uses for it's state
 * @param microseconds the period of this timer
 * @returns true just as another period has elapsed, false otherwise
 */
bool slowTimer(uint32_t* timeReg, uint32_t microseconds);
/**
 * @return the maximum SysTick timer value at the end of fastcode
 */
uint32_t getMaxSysTick(void);

float getSecondsPerFastLoop(void);
float getSecondsPerBranch(void);
uint32_t getMicrosecondsPerFastLoop(void);
/**
 * This is for debugging purposes. It allows time check points to be added to see how long fast code takes
 */
float getFastloopMaxTime(void);

/**
 * sets the sinks for the various time measurements
 * @param index 0 -> worst fastcode time; 1 -> average fastcode time; 2 -> worst slowcode time; 3 -> average slowcode time
 */
void setFastcodeSink(uint32_t index, SinkSource sink);

/**
 * gets the sinks for the various time measurements
 * @param index 0 -> worst fastcode time; 1 -> average fastcode time; 2 -> worst slowcode time; 3 -> average slowcode time
 */
SinkSource getFastcodeSink(uint32_t index);


#endif /* FASTCODEUTIL_H_ */
