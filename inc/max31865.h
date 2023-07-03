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
#ifndef INC_MAX31865_H_
#define INC_MAX31865_H_

#include <stdint.h>
#include <ports.h>
#include <spi.h>
#include <math.h>
#include "appData.h"

/**
 * must be run regularly to ensure proper operation of chip
 * @param pollingTimeSeconds time between sampling temperatures. All chips will be sampled at once.
 */
void max31865SlowCode(float pollingTimeSeconds);
void max31865Init(void);
/**
 * Configure a connected MAX31865
 * @param i specify the index of the chip configuration, to a maximum of 4
 * @param spi specify which SPI port this part in connected to
 * @param cs specify port pin used for chip select
 * @param appDataIndex specify which item in the appdata array will be used for the temperature data. If -1 then nothing happens
 * @param lowSpeedSpi when true then lowest SPI clock speed used
 * @param timeConstantSeconds the time constant applied to the data
 */
void max31865Config(uint32_t i, SpiDev spi, PortPin cs, bool enable, int32_t appDataIndex, bool threeWireNotFourWire, bool lowSpeedSpi, float timeConstantSeconds);
float getMax31865Temperature(uint32_t i);
#endif /* INC_MAX31865_H_ */
