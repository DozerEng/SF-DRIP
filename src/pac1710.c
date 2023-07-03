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
#include "pac1710.h"
#include "fastcodeUtil.h"
//#include "i2c.h"
//********************************************************
//Defines
//********************************************************
#define CURRENT_MULT (80.0e-3/1e-3/1023.0)//80mV full scale, 1mR resistor, 1023 adc counts
#define VOLTAGE_MULT ((40.0 - 40.0/1023.0)/1023.0)//TODO check this

//********************************************************

//********************************************************
//variables
//********************************************************
static TransactionStatus configStatus;
static TransactionStatus readStatus;
static I2cDev i2cDev;
static uint8_t configTxData[] = {
//		0x01,
//		0b01,
//		0b0101000,
//		0b00000100
		0x0a,//sampling config register address
//		0x0b,//channel 1 vsense sampling config reg
		0b11001000,//config register value
		0b01000011, //ch1 vsense config reg
		0b01000011, //ch2 vsense config reg

};
#define NUM_BYTES_TO_WRITE 4
#define NUM_BYTES_TO_READ 6
static uint8_t readData[NUM_BYTES_TO_READ];
//static uint8_t readTxData[] = {0x1e};
static uint8_t testAddress = 0;
static bool found = false;


//********************************************************



//********************************************************
//Function Prototypes
//********************************************************

//********************************************************


/**
 * must be called at least once at the beginning
 */
void pac1710Init(I2cDev dev){
	i2cDev = dev;
}

/**
 * must be run in main loop more than 10 times per second for each chip
 * @param address I2C address of this chip
 * @param timer a variable used by the driver to generate periodic samples
 * @param amps the current reading in amps
 */
bool pac1710SlowCode(const uint8_t address, uint32_t *timer, float *amps, float *volts){
	bool result = false;
	float temp;
	if(readStatus == TRANSACTION_COMPLETED_OK){
		found = true;
		result = true;

		temp = readData[0] << 8;
		temp += readData[1];
		*amps = temp*CURRENT_MULT;
		temp = readData[4] << 8;
		temp += readData[5];
		*volts = temp*VOLTAGE_MULT;
	}
	if(slowTimer(timer, 10000)){
		//i2cQueue(i2cDev, address, configTxData, NUM_BYTES_TO_WRITE, readData, NUM_BYTES_TO_READ, &configStatus);
		if(!found){
			++testAddress;
			if(testAddress > 127){
				testAddress = 0;
			}
		}
		i2cQueue(i2cDev, testAddress, configTxData, NUM_BYTES_TO_WRITE, readData, NUM_BYTES_TO_READ, &configStatus);
		//i2cQueue(i2cDev, address | 0b1, readTxData, 1, readData,  , &readStatus);
	}
	return result;
}
