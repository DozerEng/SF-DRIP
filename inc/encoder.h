/*
Copyright (c) 2019 STARFISH PRODUCT ENGINEERING INC.

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

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include <stdbool.h>
#include <ports.h>
#include <sinkSource.h>

/**
 * initialize this module. This should be called at startup
 */
void initEncoder(void);
/**
 * configures the specified encoder channel
 * @param index - the channel being configured
 * @param inputA - the IO index of the 'A' quadrature input
 * @param inputB - the IO index of the 'B' quadrature input
 * @param distancePerCount - the conversion from encoder counts to a meaningful distance unit, like mm for instance.
 * @param debounceNum - the number of fastcode cycles required to accept a new quadrature state
 */
void setupEncoder(uint8_t index, PortPin inputA, PortPin inputB, PortPin inputZ, float distancePerCount, uint8_t debounceNum);
/**
 * @return the position of the encoder in the configured meaningful distance unit
 */
float getEncoderDistance(uint8_t index);
float getEncoderVelocity(uint8_t index);
uint32_t getEncoderCount(void);
/**
 * must be run in fast code.
 * updates distance based on configured inputs
 */
void encoderFastcode(void);


/**
 * @return true if the index is a valid channel, false if the index is too big or disabled.
 */
bool isEncoderEnabled(uint8_t index);

/**
 * zero the specified channel
 */
void zeroEncoder(uint8_t index);

/**
 * set the encode channel to a specific value
 */
void setEncoderDistance(uint8_t index, float value);

/**
 * set the sink for the specified encoder channel
 * Presently this must only be app data
 * @param index the channel to set the sink for
 * @param sink the specified sink
 */
void setEncoderSink(uint8_t index, SinkSource sink);

#endif /* INC_ENCODER_H_ */
