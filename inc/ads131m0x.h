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

#ifndef INC_ads131m0X_H_
#define INC_ads131m0X_H_

#include <stdbool.h>
#include <stdint.h>
#include <spi.h>
#include <sinkSource.h>

//below sampling frequencies are valid only when using a 2.048MHz clock
typedef enum {
	SR_8000HZ = 0b000, //OSR 128
	SR_4000HZ = 0b001, //OSR 256
	SR_2000HZ = 0b010, //OSR 512
	SR_1000HZ = 0b011, //OSR 1024
	SR_500HZ  = 0b100, //OSR 2048
	SR_250HZ  = 0b101, //OSR 4096
	SR_125HZ  = 0b110, //OSR 8192
	SR_62_5HZ = 0b111, //OSR 16256
} ADS131M0xSampleRate;

void ads131m0xInit(void);

/**
 * Configures the module.
 * If no drdy pin is specified then the ADS131 is polled for valid data.  The module throws out data if status bit indicate all channels are not DataReady.
 * @param spi the spi port that the adc is attached to. Part is disabled if set to SPINULL_DEV
 * @param cs the gpio pin that the chip select is attached to
 * @param drdy the gpio pin that the data ready pin is attached to.  If using polling approach, set to NULL_PIN
 * @param sampleFrequency encoded in type ADS131M0xSampleRate, the ADS131M0x is the sampling/timing master
 */
void ads131m0xConfig(SpiDev spi, ADS131M0xSampleRate sampleFrequency, uint32_t numChannels, PortPin cs, PortPin drdy);

SpiDev getAds131m0xSpi(void);
ADS131M0xSampleRate getAds131m0xSampleRate(void);
uint32_t getAds131m0xNumChannels(void);
PortPin getAds131m0xCsPin(void);
PortPin getAds131m0xDrdyPin(void);

/**
 * This function causes the adc to sample, at the specified sampling period.
 * * It is intended to be called from SlowCode but it could be called from fast code for super fast sampling.
 */
bool ads131m0xSlowCode();

/**
 * handles the really fast part of getting adc data
 */
void ads131m0xFastCode(void);


/**
 * set the sink for the specified channel
 */
void setAds131m0xSink(uint32_t channel, SinkSource sink);

SinkSource getAds131m0xSink(uint32_t channel);

void setAds131m0xGain(uint32_t channel, float gain);
void setAds131m0xOffset(uint32_t channel, float offset);
float getAds131m0xGain(uint32_t channel);
float getAds131m0xOffset(uint32_t channel);
/**
 * applies the specified adc reading to cause the ADC to now measure zero at this point
 */
void zeroAds131m0x(uint32_t channel, float currentValue);

/**
 * convenience function to set a bunch of params at the same time
 * @param channel the channel to configure
 * @param sink the sink to place the output result
 * @param gain the scale to apply to this channel
 * @param offset the offset to apply to this channel
 */
 void setAds131m0xParams(uint32_t channel, SinkSource sink, float gain, float offset);
#endif /* INC_ads131m0X_H_ */
