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

#ifndef INC_HARDWARE_ENCODER_H_
#define INC_HARDWARE_ENCODER_H_

//************************************************
//note
//************************************************
//this uses the quadrature encoder functionality built in to many of the timers in the '446
//currently TIM1,2,3,4,5 & 8 support this


//************************************************
//includes
//************************************************
#include <stdint.h>
#include <stdbool.h>
#include <ports.h>
#include <sinkSource.h>

//*************************************************
//defines
//*************************************************


//*************************************************
//TypeDefs
//*************************************************

typedef enum {
	HW_ENCODER_TIMER1 = 0,
	HW_ENCODER_TIMER2 = 1,
	HW_ENCODER_TIMER3 = 2,
	HW_ENCODER_TIMER4 = 3,
	HW_ENCODER_TIMER5 = 4,
	HW_ENCODER_TIMER8 = 5
} HardwareEncoderTimer;

//*************************************************
//Variables;
//*************************************************



//*************************************************
//function prototypes
//*************************************************

/**
 * initialize this module. This should be called at startup
 */
void initHardwareEncoder(void);
/**
 * configures the specified encoder channel
 * @param timer - the timer to configure
 * @param distanceSink - the conversion from encoder counts to a meaningful distance unit, like mm for instance. Setting to NAN disables this channel
 */
void setupHardwareEncoder(HardwareEncoderTimer timer, SinkSource distanceSink);
/**
 * @param timer the specified timer channel
 * @return the sink value for this channel
 */
SinkSource getHardwareEncoderSink(HardwareEncoderTimer timer);
/**
 * @return the position of the encoder in the configured meaningful distance unit
 */
float getHardwareEncoderDistance(HardwareEncoderTimer timer);
/**
 * must be run in fast code.
 * updates distance based on configured inputs
 */
//void hardwareEncoderFastcode(void);


/**
 * @return true if the index is a valid channel, false if the index is too big or disabled.
 */
bool isHardwareEncoderEnabled(HardwareEncoderTimer timer);

/**
 * set the encode channel to a specific value
 */
void setHardwareEncoderDistance(HardwareEncoderTimer timer, float value);


/**
 * set the distance per count on the specified channel
 * @param timer - the timer to configure
 * @param value the distance per count. This is essentially a scale factor
 */
void setHardwareEncoderDistancePerCount(HardwareEncoderTimer timer, float value);
float getHardwareEncoderDistancePerCount(HardwareEncoderTimer timer);
/**
 * sets the circular limit of this encoder
 * @param limit - set to NAN for a linear encoder. Set to a max value for a rotary encoder - the total counts per revolution
 */
void setHardwareEncoderCountsPerRev(HardwareEncoderTimer timer, float value);
float getHardwareEncoderCountsPerRev(HardwareEncoderTimer timer);


void hardwareEncoderSlowcode(void);

//void setHardwareEncoderSampleTime(HardwareEncoderTimer timer, float seconds);
/**
 * sets an offset (in distance units) that will be applied to this encoder
 * This means that when the zero flag is detected, the output will be this specified offset
 * @param value the value (in distance units) of the desired offset
 */
void setHardwareEncoderOffset(HardwareEncoderTimer timer, float value);
float getHardwareEncoderOffset(HardwareEncoderTimer timer);
/**
 * sets up a sink that will indicate when a zeroing event occurs and to what encoder value
 */
void setHardwareEncoderZeroSink(HardwareEncoderTimer timer, SinkSource sink);
/**
 * sets up a sink that will indicate the calculated encoder velocity
 */
void setHardwareEncoderVelocitySink(HardwareEncoderTimer timer, SinkSource sink);

#endif /* INC_HARDWARE_ENCODER_H_ */
