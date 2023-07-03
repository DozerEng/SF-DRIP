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

#ifndef INC_SFM3019_H_
#define INC_SFM3019_H_

#include <stdbool.h>
#include <stdint.h>
#include <i2c.h>
#include <ports.h>




/**
 * @param o2Mix the O2/air mixture. 1 means 100% O2, 0 means 0% O2
 */
void sfm3019SlowCode(I2cDev dev, uint32_t microsecondsBetweenSamples, float o2Mix, PortPin notResetPin);

/**
 * @return flow in SLPM
 */
float getSfm3019Flow(void);
/**
 * @return the temperature in Celsius
 */
float getSfm3019Temperature(void);
/**
 * @return the serial number
 */
float getSfm3019SerialNumber(void);
/**
 * @return the absolute number of samples successfully taken
 */
uint32_t getSfm3019SampleCount(void);
/**
 * @return the raw status word of the sensor
 */
uint32_t getSfm3019Status(void);
/**
 * @return integral of flow in litres since the last the last time the flow was essentially zero.
 */
float getSfm3019Volume(void);
/**
 * sets flow integral to reset when no flow detected for specified time
 * if the specified time is zero then it will reset immediately this function is called and then not until it is called again
 * @param time the time that the flow magnitude must be below the threshold to trigger a reset. If zero then this is disabled but will reset during this call.
 */
void sfm3019AutoResetIntegral(float time, float threshold);

#endif /* INC_SFM3019_H_ */
