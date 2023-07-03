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
#include <stdbool.h>

#include <eeprom.h>
#include <cat24C256.h>
#include <queue.h>
#include <fastcodeUtil.h>
#include <appData.h>//TODO this is only for debugging


//*************************************************
//defines
//*************************************************
#define WRITE_QUEUE_SIZE 100
#define PERIOD (1000*MILLISECONDS) //this is the time between testing the queue for more writes
#define PERIOD_IN_SECONDS 0.05

//*************************************************
//Types
//*************************************************
typedef enum {
	EEPROM_NULL, //this indicates that there is no non-volatile backend to this module.
	EEPROM_CAT24C256,
	EEPROM_FLASH,//this indicates that the backend with be the MCU flash. This is not implemented yet
} EepromType;

typedef struct {
	uint32_t startIndex;
	uint32_t numWords;
} WriteDetails;

//*************************************************
//Variables
//*************************************************
static EepromType m_type = EEPROM_NULL;
static uint32_t m_startingAddress;
static uint32_t m_arraySize = 0;

static uint32_t* m_shadowRam = 0;

static uint32_t cat24Done = 0;
static I2cDev m_i2cDev = I2CNULL_DEV;
static uint8_t m_i2cAddress = 0;


static WriteDetails m_writeQueue[WRITE_QUEUE_SIZE];
static uint32_t m_writeQueueFront = 0;
static uint32_t m_writeQueueBack = 0;

static uint32_t m_eepromTimer = 0;
static float m_readCycleCounter = 0;
static float m_writeCycleCounter = 0;
static bool m_startup = true;//used to stop writes until we've done one complete read
static uint32_t readIndex = 0;//this is used to keep track of reading

//*************************************************
//function prototypes
//*************************************************
/**
 * internal trigger to write to the backend
 * @return true if successful
 */
static bool write(uint32_t startIndex, uint32_t numberToWrite);
/**
 * Read from backend
 * @return true if we've read from the last element of the array
 */
static bool read(void);
static bool isNotBusy(void);

/**
 * configures this module so it know what backend to use and what the maximum index will be
 * @param startingAddress the address of the first location of storage. This will be in the base units of the backend, likely bytes
 * @param arraySize the maximum number of 4-byte words in the EEPROM. Note that this is not in bytes like the starting address
 * @param shadowRam an array of words to use as local storage for EEPROM values. This must be at least the size indicated by arraySize
 */
void setupEeprom(EepromType type, uint32_t startingAddress, uint32_t arraySize, uint32_t* shadowRam);


//*************************************************
//code
//*************************************************

/**
 * reads a value from EEPROM (or more likely from shadow RAM)
 * @index index the address in words, not bytes
 */
uint32_t getEepromValue(uint32_t index){
	uint32_t result = 0;
	if(index < m_arraySize){
		result = m_shadowRam[index];
	}
	return result;
}
/**
 * reads a float value from EEPROM (or more likely from shadow RAM)
 * @param index the address in words, not bytes
 */
float getEepromFloat(uint32_t index){
	uint32_t i = getEepromValue(index);
	float* result = (float*)(&i);

	return *result;
}
/**
 * writes an array of 4 byte words to EEPROM
 * @param startIndex the first address in words, not bytes, to write the data to
 * @param numberToWrite the number of words that will be written
 * @param values an array of values at least numberToWrite long. The values in this array must not change during this function call. After that it's ok.
 *
 */
void writeEepromValues(uint32_t startIndex, uint32_t numberToWrite, uint32_t* values){

	if(m_shadowRam != 0 && !m_startup){//prevent writes if ram is not configured or we've not had one full cycle of reads
		if((startIndex + numberToWrite) <= m_arraySize){
			bool didActuallyChange = false;
			for(uint32_t i = 0; i < numberToWrite; ++i){
				uint32_t* sr = &(m_shadowRam[startIndex + i]);
				uint32_t nv = values[i];
				if(*sr != nv){
					*sr = nv;
					didActuallyChange = true;
				}

			}
			if(didActuallyChange){
				m_writeQueue[m_writeQueueBack].startIndex = startIndex;
				m_writeQueue[m_writeQueueBack].numWords = numberToWrite;
				justAddedToQueueBack(&m_writeQueueFront, &m_writeQueueBack, WRITE_QUEUE_SIZE);
			}
		}
	}
}
/**
 * writes an array of float32 to EEPROM
 * @param startIndex the first address in words, not bytes, to write the data to
 * @param numberToWrite the number of words that will be written
 * @param values an array of values at least numberToWrite long. The values in this array must not change during this function call. After that it's ok.
 *
 */
void writeEepromFloats(uint32_t startIndex, uint32_t numberToWrite, float* values){
	writeEepromValues(startIndex, numberToWrite, (uint32_t*)(values));
}
/**
 * configures this module so it know what backend to use and what the maximum index will be
 * @param startingAddress the address of the first location of storage. This will be in the base units of the backend, likely bytes
 * @param arraySize the maximum number of 4-byte words in the EEPROM. Note that this is not in bytes like the starting address
 * @param shadowRam an array of words to use as local storage for EEPROM values. This must be at least the size indicated by arraySize
 */
void setupEeprom(EepromType type, uint32_t startingAddress, uint32_t arraySize, uint32_t* shadowRam){
	m_type = type;
	m_startingAddress = startingAddress;
	m_arraySize = arraySize;
	m_shadowRam = shadowRam;
}
/**
 * configures this module to use a cat24 as a nonvolatile backend
 * @param arraySize the maximum number of 4-byte words in the EEPROM. Note that this is not in bytes like the starting address
 * @param shadowRam an array of words to use as local storage for EEPROM values. This must be at least the size indicated by arraySize
 * @param dev the I2C bus that the eeprom is on
 * @param i2cAddress the I2C address of the device
 */
void setupEepromAsCat24(uint32_t arraySize, uint32_t* shadowRam, I2cDev dev, uint8_t i2cAddress){
	m_i2cDev = dev;
	m_i2cAddress = i2cAddress;
	setupEeprom(EEPROM_CAT24C256, 0, arraySize, shadowRam);
}
/**
 * configures this module to use the STM32F446 flash as a nonvolatile backend
 * @param startingAddress the address of the first location of storage. This will be in the base units of the backend, likely bytes
 * @param arraySize the maximum number of 4-byte words in the EEPROM. Note that this is not in bytes like the starting address
 * @param shadowRam an array of words to use as local storage for EEPROM values. This must be at least the size indicated by arraySize
 */
void setupEepromAsFlash(uint32_t startingAddress, uint32_t arraySize, uint32_t* shadowRam){
	setupEeprom(EEPROM_FLASH, startingAddress, arraySize, shadowRam);
}

/**
 * configures this module to use nothing as a nonvolatile backend
 * @param arraySize the maximum number of 4-byte words in the EEPROM. Note that this is not in bytes like the starting address
 * @param shadowRam an array of words to use as local storage for EEPROM values. This must be at least the size indicated by arraySize
 */
void setupEepromAsNothing(uint32_t arraySize, uint32_t* shadowRam){
	setupEeprom(EEPROM_NULL, 0, arraySize, shadowRam);
}



/**
 * slowcode to manage the eeprom backend
 */
void eepromSlowCode(void){
	bool stop = false;
	switch(m_type){
	case EEPROM_NULL:
		break;
	case EEPROM_CAT24C256:
		cat24C256SlowCode();
		if(!isCat24C256QueueEmpty()){
			stop = true;//bail if the eeprom is busy
		}
		break;
	case EEPROM_FLASH:
		break;

	}
	if(stop){
		//wait before we do anything else
		m_eepromTimer = getTimeInMicroSeconds() + 10*MILLISECONDS;
	} else if(compareTimeToNow(m_eepromTimer) > 0.0f){
//	if(slowTimer(&m_eepromTimer, PERIOD)){

		if(isQueueNotEmpty(&m_writeQueueFront, &m_writeQueueBack, WRITE_QUEUE_SIZE)){
			m_eepromTimer = getTimeInMicroSeconds() + 100*MILLISECONDS;
			WriteDetails* wd = &m_writeQueue[m_writeQueueFront];
//			if(isNotBusy()){//for now assume we cannot be busy after 100 milliseconds!

				if(write(wd->startIndex, wd->numWords)){
					doneWithQueueFront(&m_writeQueueFront, &m_writeQueueBack, WRITE_QUEUE_SIZE);
				}
//			}
		} else {
			//if there's nothing on the queue then cycle through reading periodically.
			if(read()){
				//trigger a 100ms delay before starting to read again if we've cycled through the whole EEPROM
				m_eepromTimer = getTimeInMicroSeconds() + 100*MILLISECONDS;
			} else {
				m_eepromTimer = getTimeInMicroSeconds() + 2*MILLISECONDS;
			}
		}
	}
}

static bool write(uint32_t startIndex, uint32_t numberToWrite){
	static uint32_t index = 0;//this is used to keep track of writing if we can't do a block write
	bool result = false;
	switch(m_type){
	case EEPROM_NULL:
		//don't do anything
		result = true;
		break;
	case EEPROM_CAT24C256:
		if(index < startIndex){
			index = startIndex;

		}
		uint32_t address = m_startingAddress + index*4;
		if(queueCat24WriteWords(m_i2cDev, m_i2cAddress, address, &(m_shadowRam[index]), &cat24Done)){
			++index;

		}
		if(index >= startIndex + numberToWrite){
			++m_writeCycleCounter;

			result = true;
			index = 0;
		}
		break;
	case EEPROM_FLASH:
		//TODO:figure this out at some point!
		result = true;
		break;
	}
	return result;
}

/**
 * Read from backend
 * @return true if we've read from the last element of the array
 */
static bool read(void){
	bool result = false;
	switch(m_type){
	case EEPROM_NULL:
		//don't do anything
		result = true;
		break;
	case EEPROM_CAT24C256:
		;//an empty statement to prevent a silly error

		if(isCat24C256QueueEmpty()){
			uint32_t address = m_startingAddress + readIndex*4;

			if(queueCat24ReadWords(m_i2cDev, m_i2cAddress, address, &(m_shadowRam[readIndex]), &cat24Done)){
				++readIndex;

			}

			if(readIndex >= m_arraySize){
				readIndex = 0;
				result = true;
				++m_readCycleCounter;

				m_startup = false;//we've done one complete read cycle
			}
		}

		break;
	case EEPROM_FLASH:
		//TODO:figure this out at some point!
		result = true;
		break;
	}
	return result;

}
/**
 * indicates that the backend is not busy and is ready for another write
 */
static bool isNotBusy(void){
	bool result = false;
	switch(m_type){
	case EEPROM_NULL:
		//don't do anything
		result = true;
		break;
	case EEPROM_CAT24C256:
		if(cat24Done == 0){
			result = true;
		}
		break;
	case EEPROM_FLASH:
		result = true;
		break;
	}
	return result;
}
/**
 * @return the number of elements in the EEPROM
 */
uint32_t getEepromSize(void){
	return m_arraySize;
}

uint32_t getLastEepromReadIndex(){
	return readIndex;
}
