/*
Copyright (c) 2021 STARFISH PRODUCT ENGINEERING INC.

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


//*************************************************
//includes
//*************************************************
#include <stddef.h>
#include <math.h>
#include <streamingManager.h>
#include <queue.h>
#include <timeSync.h>
#include <appData.h>
#include <ports.h>
#include <fastcodeUtil.h>





//*************************************************
//defines
//*************************************************
#define STREAM_COUNT 8
//#define QUEUE_LENGTH 150

//*************************************************
//Types
//*************************************************
typedef struct {
	uint32_t queueFront;
	uint32_t queueBack;
	uint32_t lastQueueBack;

	FeedbackInputType type;//assign to FB_INPUT_NULL to disable channel
	int32_t inputIndex;
	bool milliNotMicroseconds;
	uint32_t time;
	float secondsFromBootUp;
	float timeRemainder;
	float* values;
//	uint32_t times[QUEUE_LENGTH];
	uint32_t queueLength;
	float periodSeconds;
	bool itsTime;
} Stream;


//*************************************************
//Variables
//*************************************************
static Stream m_streams[STREAM_COUNT];

//*************************************************
//function prototypes
//*************************************************
/**
 * @index the index to the desired stream. Assumes index is valid
 * @return a pointer to a stream structure based on the specified index. Returns 0 if invalid index
 */
static Stream* getStream(uint8_t index);

/**
 * does one stream worth of slow code
 */
static void oneStreamSlowCode(uint32_t index);

static float getSeconds();



//*************************************************
//code
//*************************************************


/**
 * initializes the stream manager. Should be called once at power up
 */
void initStreaming(void){
	for(uint8_t i = 0; i < STREAM_COUNT; ++i){
		Stream *s = getStream(i);
		if(s != 0){
			s->queueFront = 0;
			s->queueBack = 0;
			s->lastQueueBack = 0;
			s->secondsFromBootUp = 0.0f;
			s->inputIndex = 0;
			s->type = FB_INPUT_NULL;
			s->milliNotMicroseconds = true;
			s->periodSeconds = 0.0f;
			s->time = 0;
			s->values = NULL;
			s->timeRemainder = 0.0f;
			s->itsTime = false;
		}
	}
}
/**
 * assigns an array to back the queue for the specified channel
 * @param index the index of the streaming channel
 * @param array the array to back the queue
 * @param the length of the array. This determines the queue length.
 */
void setupStreamQueue(uint32_t index, float* array, uint32_t length){
	Stream *s = getStream(index);
	if(s != NULL){
		s->values = array;
		s->queueLength = length;
	}
}
/**
 * configures the indexed stream.
 * @param index the index of the stream to be configured. This can be 0 to 7
 * @param type - the type of input that will be used to make this stream
 * @param inputIndex - the index of the input channel
 * @param periodSeconds the time interval that the app data should be sampled to fill this stream
 * @param milliNotMicroseconds if true then milliseconds of timeSync.h are used, otherwise microseconds of fastcodeUtil are used
 */

void configStreaming(uint32_t index, FeedbackInputType type, uint32_t inputIndex, float periodSeconds, bool milliNotMicroseconds){
	Stream *s = getStream(index);
	if(s != NULL){

		s->inputIndex = inputIndex;
		s->type = type;
		s->milliNotMicroseconds = milliNotMicroseconds;
		s->periodSeconds = periodSeconds;



	}
}
/**
 * inidicates whether the specified stream has any samples available in its queue
 * @index the index of the stream of interest
 */
bool isNextStreamSampleAvailable(uint32_t index){
	bool result = false;
	Stream *s = getStream(index);
	if(s != NULL){
		result = isQueueNotEmpty(&(s->queueFront), &(s->queueBack), s->queueLength);
	}
	return result;
}
/**
 * gets the next sample off the front of the specified streams sample queue
 * @param index the index of the stream of interest
 * @return the value of the next sample
 */
float pullNextStreamSample(uint32_t index){
	float result = 0.0f;
	Stream *s = getStream(index);
	if(s != NULL && s->values != NULL){
		result = s->values[s->queueFront];
		doneWithQueueFront(&(s->queueFront), &(s->queueBack), s->queueLength);
	}
	return result;
}
/**
 * grabs the last added sample to this stream
 * @param index the index of the stream of interest
 */
float peekLastSampleAdded(uint32_t index){
	float result = 0.0f;
	Stream *s = getStream(index);
	if(s != NULL && s->values != NULL){
		result = s->values[s->lastQueueBack];
	}
	return result;
}

/**
 * @return the sample period of the specified stream.
 */
float getStreamSamplePeriod(uint32_t index){
	float result = 0.0f;
	Stream *s = getStream(index);
	if(s != NULL){
		result = s->periodSeconds;
	}
	return result;
}

/**
 * @index the index to the desired stream. Assumes index is valid
 * @return a pointer to a stream structure based on the specified index. Returns 0 if invalid index
 */
static Stream* getStream(uint8_t index){
	Stream *result = 0;
	if(index >= 0 && index < STREAM_COUNT){
		result =  &(m_streams[index]);
	}
	return result;
}

/**
 * inidicates whether the specified stream should have data added
 * This function is intended for data source modules to check before they push in data.
 * It is expected that if this indicates that it is time, then data WILL be pushed.
 * It updates the streams timekeeping
 * @index the index of the stream of interest
 */
bool isItTimeToSample(uint32_t index){
	bool result = false;
	Stream *s = getStream(index);
	uint32_t t;

	float delta;
	float seconds = getSeconds();
	if(s != NULL){
		if(s->itsTime){
			result = true;
		} else {
			t = getLocalTimeMillis();
			if(s->milliNotMicroseconds){
				//use synchronized milliseconds time

				delta = compareTimesMilliSec(t, s->time)*0.001f;
			} else {


				delta = seconds - s->secondsFromBootUp;
			}
			if(finitef(s->timeRemainder)){
				delta -= s->timeRemainder;//remove the remainder of the last time period.
			}
			//make sure we don't have a large negative error
			if(delta < -(s->periodSeconds)){
				delta = s->periodSeconds;//the error is large and negative so fake a elapsed sample period
			}
			if(delta >= (s->periodSeconds)){
				//it's time to sample
				result = true;
				s->timeRemainder = delta - s->periodSeconds;
				s->time = t;
				s->secondsFromBootUp = seconds;
			}
		}
	}

	return result;

}



///**
// * add a new datapoint to the specified stream.
// * It will spill the newest data if the queue is full.
// * @param index the index of the specified stream
// * @param time the time in milliseconds that the sample was sampled. It is assumed that this will be retrieved using the getLocalTimeMillis() function in timeSync.h
// * @param value the new value to put in the stream. If this value is NAN, it won't be put on the queue
// */
//void addToStream(uint32_t index, uint32_t time, float value){
//	Stream *s = getStream(index);
//	if(s != NULL && s->values != NULL && !isnanf(value) && isQueueNotFull(&(s->queueFront), &(s->queueBack), s->queueLength)){
//		s->values[s->queueBack] = value;
////		s->times[s->queueBack] = time;
//		s->time = time;
//		s->lastQueueBack = s->queueBack;
//		justAddedToQueueBack(&(s->queueFront), &(s->queueBack), s->queueLength);
//	}
//}

/**
 * add a new datapoint to the specified stream, assuming it was generated at this precise time
 * @param index the index of the specified stream
 * @param value the new value to put in the stream
 */
void addToStreamNow(uint32_t index, float value){
//	addToStream(index, getLocalTimeMillis(), value);
	Stream *s = getStream(index);
	if(s != NULL && s->values != NULL && !isnanf(value) && isQueueNotFull(&(s->queueFront), &(s->queueBack), s->queueLength)){

		s->values[s->queueBack] = value;
//		s->times[s->queueBack] = time;
		s->time = getLocalTimeMillis();
		s->secondsFromBootUp = getSeconds();
		s->itsTime = false;
		s->lastQueueBack = s->queueBack;
		justAddedToQueueBack(&(s->queueFront), &(s->queueBack), s->queueLength);
	}
}

float getStreamFirstSeconds(uint32_t index){
	float result = 0.0f;
	Stream *s = getStream(index);
	if(s != NULL){
		float t = (float)getQueueItemCount(&(s->queueFront), &(s->queueBack), s->queueLength);
		if(t > 0){
			t -= 1.0f;
		}
		t *= (s->periodSeconds);

		result = s->secondsFromBootUp - t;
	}
	return result;
}

/**
 * @param index the index of the stream of interest
 * @return the host time of the first sample of this data
 */
uint32_t getStreamFirstSampleTime(uint32_t index){

	uint32_t result = 0.0f;
	Stream *s = getStream(index);
	if(s != NULL){
		float t = (float)getQueueItemCount(&(s->queueFront), &(s->queueBack), s->queueLength);
		if(t > 0){
			t -= 1.0f;
		}
		t *= (s->periodSeconds);
		t *= 1e3f;
		t += 0.5;
		uint32_t diff = (uint32_t)t;
		result = s->time - diff;
//		result = s->times[s->queueFront];
	}
	return result;
}

/**
 * Adds a new datapoint to the specified stream if the stream's sample time has elapsed since the last time
 * @param index the specified stream index
 * @param value the new value to push into the stream
 */
bool addToStreamIfTime(uint32_t index, float value){
	bool result = false;
//	uint32_t t = getLocalTimeMillis();
//	Stream *s = getStream(index);

	if(isItTimeToSample(index)){
		if(index == 0){
			togglePin(GPIO_A12_PIN);
		}
		addToStreamNow(index, value);
	}
	return result;
}

/**
 * samples the various streams as necessary
 */
void streamManagerSlowCode(void){
	for(uint8_t i = 0; i < STREAM_COUNT; ++i){
		oneStreamSlowCode(i);
	}
}

/**
 * does one stream worth of slow code
 */
static void oneStreamSlowCode(uint32_t index){
	Stream *s = getStream(index);

	if(s != NULL && s->type != FB_INPUT_NULL && s->type != FB_INPUT_STREAM_QUEUE){


		if(isItTimeToSample(index)){
			addToStreamNow(index, getFeedbackInputOfType(s->type, s->inputIndex));
		}



	}
}

bool isStreamSecondsNotHostTime(uint32_t index){
	bool result = false;
	Stream *s = getStream(index);
	if(s != NULL){
		result = !s->milliNotMicroseconds;
	}
	return result;
}

static float getSeconds(void){
	uint32_t us = getTimeInMicroSeconds();
	float s = (float)us;
	s *= 1e-6;
	return s;
}
