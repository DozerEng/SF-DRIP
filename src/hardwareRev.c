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

//*************************************************
//notes
//*************************************************


//*************************************************
//includes
//*************************************************
#include <hardwareRev.h>
#include <stm32f4xx_flash.h>

//*************************************************
//defines
//*************************************************


#define OTP_START_ADDRESS (0x1FFF7800)
#define OTP_END_ADDRESS (0x1FFF7A0F)
//*************************************************
//Types
//*************************************************



//*************************************************
//Variables
//*************************************************
volatile FLASH_Status fs;

//*************************************************
//function prototypes
//*************************************************


static void writeWordToFlash(uint32_t address, uint32_t word);
static uint32_t readWordFromFlash(uint32_t address);
/**
 * determines the first OTP value that has NOT been written yet
 */
static uint32_t findLastAddress(void);
//*************************************************
//code
//*************************************************

/**
 * Reads the hardware revision from OTP Flash
 * The result will be the last, non-zero value of the hardware rev section of OTP Flash
 */
uint32_t getHardwareRev(void){
	uint32_t result = HARDWARE_REV_UNDEFINED;
	uint32_t a = findLastAddress();
	if(a > OTP_START_ADDRESS){
		a -= 4;
	}
	result = readWordFromFlash(a);
	if(result == 0){
		result = HARDWARE_REV_UNDEFINED;
	}
	return result;

}

/**
 * sets the hardware revision in OTP Flash.
 * If it has been set before then it will move ahead to the next word and set that
 */
void setHardwareRev(uint32_t rev){

	uint32_t r = getHardwareRev();
	if(rev != r){
		uint32_t a = findLastAddress();
		writeWordToFlash(a, rev);
	}
}


static void writeWordToFlash(uint32_t address, uint32_t word){
	FLASH_Unlock();
	fs = FLASH_ProgramWord(address, word);
	FLASH_Lock();
}

static uint32_t readWordFromFlash(uint32_t address){
	return *(uint32_t*)(address);

}
/**
 * determines the first OTP value that has NOT been written yet
 */
static uint32_t findLastAddress(void){
	uint32_t result = OTP_START_ADDRESS;

		for(uint32_t a = OTP_START_ADDRESS; a <= OTP_END_ADDRESS; a += 4){
			uint32_t t = readWordFromFlash(a);
			result = a;

			if(t == 0xffffffff){
				break;
			}
		}
		return result;
}

