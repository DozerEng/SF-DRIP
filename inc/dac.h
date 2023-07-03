/*
Copyright (c) 2017-2018 STARFISH PRODUCT ENGINEERING INC.

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
#ifndef DAC_H_
#define DAC_H_

//#include "_processorGlobal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * initialize the driver
 */
void dacInit(void);
/**
 * this should run in fast code
 */
void dacFastCode(void);


void setDac1Value(uint16_t v);
void setDac2Value(uint16_t v);
void setDacAsFloat(uint32_t channel, float value);
bool isDacEnable(uint32_t channel);
void setDacEnable(uint32_t channel, bool e);

#endif /* DAC_H_ */
