/*
Copyright (c) 2018 STARFISH PRODUCT ENGINEERING INC.

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

#include <timeSync.h>
#include <fastcodeUtil.h>
#include <ports.h>
#include <math.h>
#include <error.h>


#define FILTER_CONSTANT 1.0f

static uint32_t m_localMilliseconds = 0;
static float m_fastCodeDelta = NAN;//in seconds

static float m_millisecondAccumulator = 0.0f;//in seconds
static float m_error = NAN;
static float m_errorIntegral = 0;

/**
 * Must be run in fast code to keep time up to date
 */
void timeSyncFastCode(void){

	float nominalTime = getSecondsPerFastLoop();
	float minTime = nominalTime * 0.2f;
	float maxTime = nominalTime * 5.0f;
	if(!finitef(m_fastCodeDelta)){
		m_fastCodeDelta = nominalTime;
	} else if(m_fastCodeDelta > maxTime){
		m_fastCodeDelta  = maxTime;
	} else if(m_fastCodeDelta < minTime){
		m_fastCodeDelta = minTime;
	}
	m_millisecondAccumulator += m_fastCodeDelta;

	float ms = m_millisecondAccumulator * 1000;
	int32_t msi = (int32_t)ms;

	m_millisecondAccumulator -= ((float)msi)*0.001;
	m_localMilliseconds += msi;


//	if(m_millisecondAccumulator > 1.0e-3f){
//		m_millisecondAccumulator -= 1.0e-3f;
//		++m_localMilliseconds;
//
//
//	}
}

/**
 * Runs periodically when host packets are received with a host time update
 * Host time is millisecond portion of the hosts long time
 * This routine carries out the following steps
 * - computes time error
 * - detects roll-over either in host or local time
 * - computes time increment correction
 * @param hostTime the host time from the last received packet
 * @param packetTime the local time when the last packet was received
 */
void updateTimeSync(uint32_t hostTime, uint32_t packetTime){
//	int64_t ht = (int64_t)hostTime;
//	int64_t pt = (int64_t)packetTime;
//	int64_t error = ht - pt;
//	//see if we can correct a rollover
//
//	if(error >= (1L<<31L)){//packet time has rolled and host hasn't
//		error -= 1L<<31L;
//	} else if(error <= -(1L<<31L)){//host time has rolled and packet hasn't
//		error += 1L<<31L;
//	}
	



	m_error = 0.001f*compareTimesMilliSec(hostTime, packetTime);
	if(m_error < -1.0f || m_error > 1.0f){
		//do a large correction if we're really off
		m_fastCodeDelta = getSecondsPerFastLoop();
		m_localMilliseconds = hostTime;
		m_millisecondAccumulator = 0.0f;
		logError(TIME_SYNC_ERROR, 0);


	} else {
//		m_error *= 1e-3;
		m_errorIntegral += m_error;

		//now error should be the actual error in microseconds

	//	if(m_error > 10000 || m_error < -10000){
	//		//error is too big so reset time
	//		m_localMilliseconds = hostTime;
	//		m_fastCodeDelta = SECONDS_PER_FASTLOOP;
	//	} else if(__isnanf(m_error) || __isinff(m_error)){
	//		//error is NaN so don't do anything
	//
	//	} else if(m_fastCodeDelta > SECONDS_PER_FASTLOOP*1.2f){
	//		//delta is too big so reset to nominal
	//		m_fastCodeDelta = SECONDS_PER_FASTLOOP;
	//	} else if(m_fastCodeDelta < SECONDS_PER_FASTLOOP*0.8f){
	//		//delta is too small so reset to nominal
	//		m_fastCodeDelta = SECONDS_PER_FASTLOOP;
	//	} else if(m_millisecondAccumulator != m_millisecondAccumulator){
	//		//accumulator is NaN so reset
	//		m_millisecondAccumulator = 0.0f;
	//	} else {
	//		//now apply the error to the delta
	//		float a = 0.0001;
	//		float b = 0.000001;
	//		float delta = m_fastCodeDelta;
	//		delta -= delta*a;
	//		delta += m_error*b*a;
	//
	//		m_fastCodeDelta = delta;
	//	}
		//m_fastCodeDelta = SECONDS_PER_FASTLOOP;
	//	m_fastCodeDelta = (100.0*m_error + 0.02*m_errorIntegral)*SECONDS_PER_FASTLOOP/0.1;

		//m_fastCodeDelta += (100.0e-6*m_error + 5.0e-6*m_errorIntegral - m_fastCodeDelta) * FILTER_CONSTANT;
		m_fastCodeDelta = 100.0e-6*m_error + 5.0e-6*m_errorIntegral;
		//catch illegal value
		if(!finitef(m_fastCodeDelta)){
			m_fastCodeDelta = getSecondsPerFastLoop();
		} else if(m_fastCodeDelta == 0){
			 m_fastCodeDelta = getSecondsPerFastLoop();
		}
	}
}


/**
 * @return the local time that is hopefully synchronized with the host time
 */
uint32_t getLocalTimeMillis(void){

	return m_localMilliseconds;
}
float getTimeSyncError(void){
	//return (m_fastCodeDelta);
	return m_error;
}
float getTimeSyncGain(void){
	return (m_fastCodeDelta);
}
/**
 * Compares two host times. result is positive if time1 is greater (i.e. is later) than time2
 * result in milliseconds
 */
float compareTimesMilliSec(uint32_t time1, uint32_t time2){
	uint32_t uerror = time1 - time2;
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
	return (float)error;

}
