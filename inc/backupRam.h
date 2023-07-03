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

#ifndef INC_BACKUPRAM_H_
#define INC_BACKUPRAM_H_


/**
 * Handy library of access commands for battery backed up ram
 * This requires the following things to be set up:
 * - RCC_AHB1Periph_BKPSRAM enabled
 * - PWR_BackupAccessCmd(ENABLE);
 * - RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
 * - RCC_RTCCLKCmd(ENABLE);
 * - PWR_BackupRegulatorCmd(ENABLE);//enable backup domain regulator
 *
 *
 */



//*************************************************
//includes
//*************************************************
#include "stdint.h"


/**
 * reads a signed integer from backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 */
int32_t backupRamReadInt(uint32_t address);
/**
 * reads an unsigned integer from backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 */
uint32_t backupRamReadUint(uint32_t address);
/**
 * reads a float from backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 */
float backupRamReadFloat(uint32_t address);
/**
 * writes a signed integer to the backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 * @param data is the signed integer to write
 */
void backupRamWriteInt(uint32_t address, int32_t data);
/**
 * writes an unsigned integer to the backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 * @param data is the unsigned integer to write
 */
void backupRamWriteUint(uint32_t address, uint32_t data);
/**
 * writes a float to the backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 * @param data is the float to write
 */
void backupRamWriteFloat(uint32_t address, float data);

#endif /* INC_BACKUPRAM_H_ */
