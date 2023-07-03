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

#ifndef TIMESYNC_H_
#define TIMESYNC_H_

#include <stdint.h>

/**
 * Must be run in fast code to keep time up to date
 */
void timeSyncFastCode(void);

/**
 * Runs periodically when host packets are received with a host time update
 */
void updateTimeSync(uint32_t hostTime, uint32_t packetTime);

/**
 * @return the local time that is hopefully synchronized with the host time
 */
uint32_t getLocalTimeMillis(void);

float getTimeSyncGain(void);
float getTimeSyncError(void);
/**
 * Compares two host times. result is positive if time1 is greater (i.e. is later) than time2
 * result in microseconds
 */
float compareTimesMilliSec(uint32_t time1, uint32_t time2);

#endif /* TIMESYNC_H_ */
