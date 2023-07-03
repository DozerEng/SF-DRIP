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

#ifndef INC_STREAMINGMANAGER_H_
#define INC_STREAMINGMANAGER_H_

//*************************************************
//notes
//*************************************************
//this module samples a specified input source and
//provides the data for the sfdqPackets module to stream it to the bus controller

//*************************************************
//includes
//*************************************************
#include <stdbool.h>
#include <stdint.h>
#include <feedbackControl.h>

//*************************************************
//defines
//*************************************************


//*************************************************
//Types
//*************************************************



//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************

/**
 * initializes the stream manager. Should be called once at power up
 */
void initStreaming(void);
/**
 * configures the indexed stream.
 * @param index the index of the stream to be configured. This can be 0 to 7

 * @param inType - the type of input that will be used to make this stream
 * @param inIndex - the index of the input channel
 * @param milliNotMicroseconds if true then milliseconds of timeSync.h are used, otherwise microseconds of fastcodeUtil are used
 * @param periodSeconds the time interval that the app data should be sampled to fill this stream
 */

void configStreaming(uint32_t index, FeedbackInputType type, uint32_t inputIndex, float periodSeconds, bool milliNotMicroseconds);
/**
 * inidicates whether the specified stream has any samples available in its queue
 * @index the index of the stream of interest
 */
bool isNextStreamSampleAvailable(uint32_t index);
/**
 * inidicates whether the specified stream should have data added
 * This function is intended for data source modules to check before they push in data
 * @index the index of the stream of interest
 */
bool isItTimeToSample(uint32_t index);

/**
 * gets the next sample off the front of the specified streams sample queue
 * @param index the index of the stream of interest
 * @return the value of the next sample
 */
float pullNextStreamSample(uint32_t index);
/**
 * grabs the last added sample to this stream
 * @param index the index of the stream of interest
 */
float peekLastSampleAdded(uint32_t index);
/**
 * @return the sample period of the specified stream in seconds
 */
float getStreamSamplePeriod(uint32_t index);




/**
 * @param index the index of the stream of interest
 * @return the host time of the first sample of this data
 */
uint32_t getStreamFirstSampleTime(uint32_t index);



///**
// * add a new datapoint to the specified stream
// * @param index the index of the specified stream
// * @param time the time in milliseconds of the sample
// * @param value the new value to put in the stream
// */
//void addToStream(uint32_t index, uint32_t time, float value);
/**
 * add a new datapoint to the specified stream, assuming it was generated at this precise time
 * @param index the index of the specified stream
 * @param value the new value to put in the stream
 */
void addToStreamNow(uint32_t index, float value);

/**
 * assigns an array to back the queue for the specified channel
 * @param index the index of the streaming channel
 * @param array the array to back the queue
 * @param the length of the array. This determines the queue length.
 */
void setupStreamQueue(uint32_t index, float* array, uint32_t length);

/**
 * Adds a new datapoint to the specified stream if the stream's sample time has elapsed since the last time
 * @param index the specified stream index
 * @param value the new value to push into the stream
 */
bool addToStreamIfTime(uint32_t index, float value);

/**
 * samples the various streams as necessary
 */
void streamManagerSlowCode(void);
bool isStreamSecondsNotHostTime(uint32_t index);
float getStreamFirstSeconds(uint32_t index);


#endif /* INC_STREAMINGMANAGER_H_ */
