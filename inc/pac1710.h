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

#ifndef PAC1710_H_
#define PAC1710_H_

#include "i2c.h"
/**
 * must be called at least once at the beginning
 */
void pac1710Init(I2cDev dev);
/**
 * must be run in main loop more than 10 times per second for each chip
 * @param address I2C address of this chip
 * @param timer a variable used by the driver to generate periodic samples
 * @param amps the current reading in amps
 * @return true if a new value is available
 */
bool pac1710SlowCode(const uint8_t address, uint32_t *timer, float *amps, float *volts);

#endif /* PAC1710_H_ */
