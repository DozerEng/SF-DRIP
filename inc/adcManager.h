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

#ifndef INC_ADCMANAGER_H_
#define INC_ADCMANAGER_H_
#include <stdint.h>
#include <stdbool.h>
#include <sinkSource.h>

void setAdcChannelMask(uint32_t mask);
float getAdcValue(uint32_t index);
uint32_t getAdcChannelMask(void);
//uint32_t getAdcChannelIndex(uint32_t index);
uint32_t getAdcCount(void);
//float getAdcValueFromMask(uint32_t mask);
/**
 * sets up an adc channel
 * @param index the channel to setup
 * @param timeConstantSeconds the time constant for the low pass filter
 * @param scale the gain to apply to this channel
 * @param offset the value to subtract from the channel before applying the gain
 */
void setAdcParams(uint32_t index, float timeConstantSeconds, float scale, float offset);
void adcManagerSlowCode(void);
void adcManagerFastCode(void);
void adcManagerInit(void);
bool isAdcEnabled(uint32_t index);
void setAdcEnabled(uint32_t index, bool enabled);
/**
 * sets up a sink for the specified channel's adc data
 * @param index the channel to setup
 * @param sink the sink to funnel the data to
 *
 */
void setAdcSink(uint32_t index, SinkSource sink);
/**
 * sets up the gain for the specified channel
 * @param index the channel to setup
 * @param sink the sink to funnel the data to
 *
 */
void setAdcGain(uint32_t index, float value);
/**
 * sets up the dc offset for the specified channel
 * @param index the channel to setup
 * @param sink the sink to funnel the data to
 *
 */
void setAdcOffset(uint32_t index, float value);
/**
 * sets up the time constant for the specified channel
 * @param index the channel to setup
 * @param sink the sink to funnel the data to
 *
 */
void setAdcTimeConstant(uint32_t index, float value);

SinkSource getAdcSink(uint32_t index);
float getAdcGain(uint32_t index);
float getAdcOffset(uint32_t index);
void zeroAdc(uint32_t index, float currentValue);
float getAdcTimeConstant(uint32_t index);


#endif /* INC_ADCMANAGER_H_ */
