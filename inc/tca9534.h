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

#ifndef INC_TCA9534_H_
#define INC_TCA9534_H_

#include <stdbool.h>
#include <stdint.h>
#include <i2c.h>



/**
 * @param i2c specifies which bus to use
 * @param address the i2c address offset of this port expander.
 * This corresponds to the 3 address select pins combined as binary digits into an integer
 * address  actual bus address
 *  0b000  0x38
 *  0b001  0x39
 *  0b010  0x3a
 *  0b011  0x3b
 *  0b100  0x3c
 *  0b101  0x3d
 *  0b110  0x3e
 *  0b111  0x3f
 *  @param dir the direction register mask, where a bit of 1 is an input and 0 is an output
 *  @param output the values to set the output pins to
 *  @param input a pointer to a variable where to put the read in pin values
 *  @return true if queue was successful
 */
bool queueTca9534(I2cDev dev, uint8_t address, uint8_t dir, uint8_t output, uint8_t* inputs);

uint32_t getTca9534QueueRoom(void);

void tca9534SlowCode(void);

#endif /* INC_TCA9534_H_ */
