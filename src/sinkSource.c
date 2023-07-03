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
//includes
//*************************************************


#include <sinkSource.h>

#include <appData.h>
#include <streamingManager.h>






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


//*************************************************
//code
//*************************************************

/**
 * @return the index of the specified sink or source
 */
uint32_t getSinkSourceIndex(SinkSource s){
	return s & 0b111111;
}
/**
 * sets the specified sink or source to the specified value
 * @param s the specified sink or source
 * @param value the value to set it to
 */
void setSinkSource(SinkSource s, float value){
	uint32_t i = getSinkSourceIndex(s);
	switch(s >> 6){
	case 0b00:
		break;
	case 0b01:
		addToStreamIfTime(i, value);
		break;
	case 0b10:
		setRxAppData(i, value);
		break;
	case 0b11:
		setTxAppData(i, value);
		break;

	}
}

bool isSinkSourceAppData(SinkSource s){
	 // bits 7,6 type: 0b00 undefined, 0b01 stream, 0b10 rxAppData, 0b11 txAppata
	return s & (0b10<<6);

}

bool isSinkSourceStream(SinkSource s){
 // bits 7,6 type: 0b00 undefined, 0b01 stream, 0b10 rxAppData, 0b11 txAppata
	return (s & (0b11<<6)) == (0b01<<6);
}
/**
 * construct a sink source for tx app data
 * @param i the index of the app data blackboard
 * @return the specified sink source
 */
SinkSource txAppDataSink(uint32_t i){
	return (i & 0x3f) | TX_APP_DATA_SINK;
}
/**
 * construct a sink source for rx app data
 * @param i the index of the app data blackboard
 * @return the specified sink source
 */
SinkSource rxAppDataSink(uint32_t i){
	return (i & 0x3f) | RX_APP_DATA_SINK;
}
/**
 * construct a sink source for a stream
 * @param i the index of the desired stream
 * @return the specified sink source
 */
SinkSource streamSink(uint32_t i){
	return (i & 0x3f) | STREAM_SINK;

}
/**
 * indicates if the specified sink is a stream and if it is ready to sample
 * If this indicates that it is time, it is expected that data WILL be added.
 */
bool isStreamSampleTime(SinkSource s){
	return isSinkSourceStream(s) && isItTimeToSample(getSinkSourceIndex(s));
}

