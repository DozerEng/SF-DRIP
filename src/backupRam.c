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


//*************************************************
//includes
//*************************************************
#include "backupRam.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_pwr.h"
#include <math.h>


//*************************************************
//defines
//*************************************************

#define MAX_BACKUP_RAM_ADDRESS 1023

//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************

static __IO int32_t* backupRamInt = (__IO int32_t *) (BKPSRAM_BASE);
static __IO uint32_t* backupRamUint = (__IO uint32_t *) (BKPSRAM_BASE);
static __IO float* backupRamFloat = (__IO float *) (BKPSRAM_BASE);

//*************************************************
//function prototypes
//*************************************************


//*************************************************
//code
//*************************************************

/**
 * reads a signed integer from backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 */
int32_t backupRamReadInt(uint32_t address){
	int32_t result = 0;
	if(address < MAX_BACKUP_RAM_ADDRESS){
		result = backupRamInt[address];
	}
	return result;
}
/**
 * reads an unsigned integer from backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 */
uint32_t backupRamReadUint(uint32_t address){
	uint32_t result = 0;
	if(address < MAX_BACKUP_RAM_ADDRESS){
		result = backupRamUint[address];
	}
	return result;
}
/**
 * reads a float from backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 */
float backupRamReadFloat(uint32_t address){
	float result = NAN;
	if(address < MAX_BACKUP_RAM_ADDRESS){
		result = backupRamFloat[address];
	}
	return result;
}
/**
 * writes a signed integer to the backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 * @param data is the signed integer to write
 */
void backupRamWriteInt(uint32_t address, int32_t data){
	if(address < MAX_BACKUP_RAM_ADDRESS){
		PWR_BackupAccessCmd(ENABLE);
		backupRamInt[address] = data;
	}
}
/**
 * writes an unsigned integer to the backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 * @param data is the unsigned integer to write
 */
void backupRamWriteUint(uint32_t address, uint32_t data){
	if(address < MAX_BACKUP_RAM_ADDRESS){
		PWR_BackupAccessCmd(ENABLE);
		backupRamUint[address] = data;
	}
}
/**
 * writes a float to the backup ram
 * @param address the index to the ram word to read from. This can be 0 to 1023.
 * @param data is the float to write
 */
void backupRamWriteFloat(uint32_t address, float data){
	if(address < MAX_BACKUP_RAM_ADDRESS){
		PWR_BackupAccessCmd(ENABLE);
		backupRamFloat[address] = data;
	}
}

