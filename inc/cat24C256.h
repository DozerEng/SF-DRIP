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

#ifndef INC_CAT24C256_H_
#define INC_CAT24C256_H_

#include <stdbool.h>
#include <stdint.h>
#include <i2c.h>


/**
 * queues a write to a single EEPROM word location.
 * this first checks to see if the EEPROM value is already correct
 * This will sample the data variable at the start and will not write to it
 * @param dev the I2C bus that the eeprom is on
 * @param i2cAddress the I2C address of the device
 * @param memAddress the 19 bit address to read from in the EEPROM memory
 * @param data the variable to put the result into
 * @param done will be incremented when the read is complete. It will be set to 0xffff if it fails.
 * @return true if successfully queued, false if the queue is full
 */
bool queueCat24WriteWords(I2cDev dev, uint8_t i2cAddress, uint32_t memAddress, uint32_t* data, uint32_t* done);
/**
 * queue a read from a single EEPROM word location
 * @param dev the I2C bus that the eeprom is on
 * @param i2cAddress the I2C address of the device
 * @param memAddress the 19 bit address to read from in the EEPROM memory
 * @param data the variable to put the result into
 * @param done will be incremented when the read is complete. It will be set to 0xffff if it fails.
 * @return true if successfully queued, false if the queue is full
 */
bool queueCat24ReadWords(I2cDev dev, uint8_t i2cAddress, uint32_t memAddress, uint32_t* data, uint32_t* done);

bool isCat24C256QueueEmpty(void);

/**
 * this must be run periodically to ensure proper operation.
 * It expects at least 10ms between calss.
 */
void cat24C256SlowCode(void);

#endif /* INC_CAT24C256_H_ */
