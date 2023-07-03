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

#ifndef INC_APPDATA_H_
#define INC_APPDATA_H_

#include <stdint.h>

void initAppData(float* txData, uint32_t txLength, float* rxData, uint32_t rxLength);
void setRxAppData(uint32_t index, float value);
void setTxAppData(uint32_t index, float value);
float getRxAppData(uint32_t index);
float getTxAppData(uint32_t index);
uint32_t getTxAppDataLength(void);
uint32_t getRxAppDataLength(void);
/**
 * casts the 32 bit value as a uint32 instead of a float32.
 * Note that this is not the same as (uint32_t)getRxAppData(index)
 */
uint32_t getRxAppDataAsInt(uint32_t index);
/**
 * casts the 32 bit value as a uint32 instead of a float32.
 * Note that this is not the same as (uint32_t)getTxAppData(index)
 */
uint32_t getTxAppDataAsInt(uint32_t index);
#endif /* INC_APPDATA_H_ */

