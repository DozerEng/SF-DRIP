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

#ifndef INC_ADS131A0X_H_
#define INC_ADS131A0X_H_

#include <stdbool.h>
#include <stdint.h>
#include <spi.h>



void ads131a0xInit(void);

/**
 * @param enable enables or disables this module
 * @param spi the spi port that the adc is attached to
 * @param cs the gpio pin that the chip select is attached to
 * @param drdy the gpio pin that the data ready pin is attached to
 */
void ads131a0xConfig(bool enable, SpiDev spi, PortPin cs, PortPin drdy);
/**
 * queries whether there are any samples in the queue.
 */
bool isAds131a0xSampleAvailable(void);
/**
 * reads the last samples acquired.
 * Each sample consists of a number of values, depending on the channels on the ADC.
 * For now it will be 2 for the ads131a02
 * @param the channel of the sample
 */
float getAds131a0xSample(uint32_t channel);
/**
 * when we're done with the last sample of channels, then discard it.
 */
void ads131a0xUnqueue(void);
/**
 * This function causes the adc to sample, at the specified sampling period.
 * * It is intended to be called from SlowCode but it could be called from fast code for super fast sampling.
 * @param microseconds the period of sampling
 */
bool ads131a0xControl(const uint32_t microseconds);

#endif /* INC_ADS131A0X_H_ */
