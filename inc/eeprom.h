/*
Copyright (c) 2021 STARFISH PRODUCT ENGINEERING INC.

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

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

//*************************************************
//notes
//*************************************************
//this module handles high level communication to an EEPROM
//for now, let's assume there may be various EEPROM backends to implement the non-volatile storage


//*************************************************
//includes
//*************************************************
#include <stdint.h>
#include <i2c.h>

//*************************************************
//defines
//*************************************************


//*************************************************
//Types
//*************************************************



//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************

/**
 * reads a value from EEPROM (or more likely from shadow RAM)
 * @index index the address in words, not bytes
 */
uint32_t getEepromValue(uint32_t index);
/**
 * reads a float value from EEPROM (or more likely from shadow RAM)
 * @param index the address in words, not bytes
 */
float getEepromFloat(uint32_t index);
/**
 * writes an array of 4 byte words to EEPROM
 * @param startIndex the first address in words, not bytes, to write the data to
 * @param numberToWrite the number of words that will be written
 * @param values an array of values at least numberToWrite long. The values in this array must not change during this function call. After that it's ok.
 *
 */
void writeEepromValues(uint32_t startIndex, uint32_t numberToWrite, uint32_t* values);
/**
 * writes an array of float32 to EEPROM
 * @param startIndex the first address in words, not bytes, to write the data to
 * @param numberToWrite the number of words that will be written
 * @param values an array of values at least numberToWrite long. The values in this array must not change during this function call. After that it's ok.
 *
 */
void writeEepromFloats(uint32_t startIndex, uint32_t numberToWrite, float* values);
/**
 * configures this module to use a cat24 as a nonvolatile backend
 * @param arraySize the maximum number of 4-byte words in the EEPROM. Note that this is not in bytes like the starting address
 * @param shadowRam an array of words to use as local storage for EEPROM values. This must be at least the size indicated by arraySize
 * @param dev the I2C bus that the eeprom is on
 * @param i2cAddress the I2C address of the device
 */
void setupEepromAsCat24(uint32_t arraySize, uint32_t* shadowRam, I2cDev dev, uint8_t i2cAddress);
/**
 * configures this module to use the STM32F446 flash as a nonvolatile backend
 * @param startingAddress the address of the first location of storage. This will be in the base units of the backend, likely bytes
 * @param arraySize the maximum number of 4-byte words in the EEPROM. Note that this is not in bytes like the starting address
 * @param shadowRam an array of words to use as local storage for EEPROM values. This must be at least the size indicated by arraySize
 */
void setupEepromAsFlash(uint32_t startingAddress, uint32_t arraySize, uint32_t* shadowRam);
/**
 * configures this module to use nothing as a nonvolatile backend
 * @param arraySize the maximum number of 4-byte words in the EEPROM. Note that this is not in bytes like the starting address
 * @param shadowRam an array of words to use as local storage for EEPROM values. This must be at least the size indicated by arraySize
 */
void setupEepromAsNothing(uint32_t arraySize, uint32_t* shadowRam);
/**
 * slowcode to manage the eeprom backend
 */
void eepromSlowCode(void);
/**
 * @return the number of elements in the EEPROM
 */
uint32_t getEepromSize(void);

uint32_t getLastEepromReadIndex();

#endif /* INC_EEPROM_H_ */
