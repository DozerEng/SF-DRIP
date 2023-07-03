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

#ifndef INC_SINKSOURCE_H_
#define INC_SINKSOURCE_H_

//*************************************************
//notes
//This module provides a general method to access a stream or app data

//*************************************************

//*************************************************
//includes
//*************************************************

#include <stdint.h>
#include <stdbool.h>

//*************************************************
//defines
//*************************************************
#define NULL_SINK_SOURCE 0


#define STREAM_SINK (0b01<<6)
#define RX_APP_DATA_SINK (0b10<<6)
#define TX_APP_DATA_SINK (0b11<<6)

//*************************************************
//Types
//*************************************************
/**
 * a type to contain the sink or source definition
 * consists of a uint8
 * bits 7,6 type: 0b00 undefined, 0b01 stream, 0b10 rxAppData, 0b11 txAppata
 * bits 5..0 index
 */
typedef uint8_t SinkSource;

//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************




/**
 * @return the index of the specified sink or source
 */
uint32_t getSinkSourceIndex(SinkSource s);
/**
 * sets the specified sink or source to the specified value
 * @param s the specified sink or source
 * @param value the value to set it to
 */
void setSinkSource(SinkSource s, float value);

bool isSinkSourceAppData(SinkSource s);

bool isSinkSourceStream(SinkSource s);

/**
 * construct a sink source for tx app data
 * @param i the index of the app data blackboard
 * @return the specified sink source
 */
SinkSource txAppDataSink(uint32_t i);
/**
 * construct a sink source for rx app data
 * @param i the index of the app data blackboard
 * @return the specified sink source
 */
SinkSource rxAppDataSink(uint32_t i);
/**
 * construct a sink source for a stream
 * @param i the index of the desired stream
 * @return the specified sink source
 */
SinkSource streamSink(uint32_t i);
/**
 * indicates if the specified sink is a stream and if it is ready to sample
 */
bool isStreamSampleTime(SinkSource s);

#endif /* INC_SINKSOURCE_H_ */
