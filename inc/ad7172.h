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

#ifndef AD7172_H_
#define AD7172_H_

#include <stdint.h>
#include <stdbool.h>
#include <spi.h>
#include <ports.h>
#include <sinkSource.h>
#endif /* AD7172_H_ */

typedef enum {
	AIN0      = 0,
	AIN1      = 1,
	AIN2      = 2,
	AIN3      = 3,
	AIN4      = 4,
	TEMP_POS  = 5,
	TEMP_NEG  = 6,
	REF_POS   = 7,
	REF_NEG   = 8,
} Ad7172Inputs;


void ad7172SlowCode(SpiDev dev, PortPin csPin);
/**
 * configures the inputs for the 4 channels
 */
void ad7172ConfigAll(Ad7172Inputs pos0, Ad7172Inputs neg0, Ad7172Inputs pos1, Ad7172Inputs neg1, Ad7172Inputs pos2, Ad7172Inputs neg2, Ad7172Inputs pos3, Ad7172Inputs neg3);
//void ad7172Config(Ad7172Inputs pos0, neg0, pos1, neg1, pos2, neg2, pos3, neg3);

void ad7172Config(uint32_t channel, Ad7172Inputs pos0, Ad7172Inputs neg0, bool enable);

float ad7172GetValue(uint32_t channel);
Ad7172Inputs ad7172GetInput(uint32_t channel, bool posNotNeg);

void ad7172SetGain(uint32_t channel, float gain);
void ad7172SetOffset(uint32_t channel, float offset);
void ad7172SetSink(uint32_t channel, SinkSource sink);

float ad7172GetGain(uint32_t channel);
float ad7172GetOffset(uint32_t channel);
SinkSource ad7172GetSink(uint32_t channel);
