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

#include "cat24C256.h"
#include "queue.h"
#include "fastcodeUtil.h"


//*************************************************
//Defines
//*************************************************

#define QUEUE_SIZE 100
#define I2C_ADDRESS 0b01010000
#define BYTE_BUFFER_SIZE 10


//*************************************************
//Types
//*************************************************
typedef enum {
	CAT_NEW,
	CAT_DOING_READ,
	CAT_DOING_READ_BEFORE_WRITE,
	CAT_DOING_WRITE,
	CAT_DONE,
	CAT_FAIL
} Status;

typedef struct {
	I2cDev dev;
	uint8_t i2cAddress;
	uint32_t memAddress;
	uint32_t dataToWrite;
	uint32_t* variableToReadTo;
	Status status;
	bool readNotWrite;
	uint32_t* doneReg;
} EepromTransaction;

//*************************************************
//Variables
//*************************************************
static uint32_t queueFront = 0;
static uint32_t queueBack = 0;
static EepromTransaction queue[QUEUE_SIZE];
static uint8_t dataToWrite[BYTE_BUFFER_SIZE];
static uint8_t dataRead[BYTE_BUFFER_SIZE];
static TransactionStatus i2cStatus;
static uint32_t timer;
//static uint32_t timeoutTimer;

//*************************************************
//function prototypes
//*************************************************

//*************************************************
//Code
//*************************************************







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
bool queueCat24WriteWords(I2cDev dev, uint8_t i2cAddress, uint32_t memAddress, uint32_t* data, uint32_t* done){
	bool result = false;
	if(getQueueRoom(&queueFront, &queueBack, QUEUE_SIZE) >= 1){
		queue[queueBack].dev = dev;
		queue[queueBack].i2cAddress = (I2C_ADDRESS | (i2cAddress & 0b111))<<1;
		queue[queueBack].memAddress = memAddress;
		queue[queueBack].dataToWrite = *data;
		queue[queueBack].variableToReadTo = 0;
		queue[queueBack].status = CAT_NEW;
		queue[queueBack].readNotWrite = false;
		queue[queueBack].doneReg = done;
		justAddedToQueueBack(&queueFront, &queueBack, QUEUE_SIZE);
		result = true;
	}
	return result;
}
/**
 * queue a read from a single EEPROM word location
 * @param dev the I2C bus that the eeprom is on
 * @param i2cAddress the I2C address of the device
 * @param memAddress the 19 bit address to read from in the EEPROM memory
 * @param data the variable to put the result into
 * @param done will be incremented when the read is complete. It will be set to 0xffff if it fails.
 * @return true if successfully queued, false if the queue is full
 */
bool queueCat24ReadWords(I2cDev dev, uint8_t i2cAddress, uint32_t memAddress, uint32_t* data, uint32_t* done){
	bool result = false;
	if(isQueueNotFull(&queueFront, &queueBack, QUEUE_SIZE)){
		queue[queueBack].dev = dev;
		queue[queueBack].i2cAddress = (I2C_ADDRESS | (i2cAddress & 0b111))<<1;
		queue[queueBack].memAddress = memAddress;
		queue[queueBack].dataToWrite = 0;
		queue[queueBack].variableToReadTo = data;
		queue[queueBack].status = CAT_NEW;
		queue[queueBack].readNotWrite = true;
		queue[queueBack].doneReg = done;
		justAddedToQueueBack(&queueFront, &queueBack, QUEUE_SIZE);
		result = true;
	}
	return result;
}

bool isCat24C256QueueEmpty(void){
	return !isQueueNotEmpty(&queueFront, &queueBack, QUEUE_SIZE);
}

/**
 * this must be run periodically to ensure proper operation.
 * It expects at least 10ms between calss.
 */
void cat24C256SlowCode(void){
	static uint32_t timeoutTimer = 0;
	if(slowTimer(&timer, 1000) && isQueueNotEmpty(&queueFront, &queueBack, QUEUE_SIZE)){
		EepromTransaction* et = &queue[queueFront];

		switch(et->status){
		case CAT_NEW:
			//this is a new transaction so start fresh
			resetSlowTimer(&timeoutTimer);
			if(et->readNotWrite){
				dataToWrite[0] = 0xff & (et->memAddress >> 8);
				dataToWrite[1] = 0xff & (et->memAddress);
				i2cQueue(et->dev, et->i2cAddress, dataToWrite, 2, dataRead, 4, &i2cStatus);
				et->status = CAT_DOING_READ;
			} else {
				//this must be a write request. First do a read
				dataToWrite[0] = 0xff & (et->memAddress >> 8);
				dataToWrite[1] = 0xff & (et->memAddress);
				i2cQueue(et->dev, et->i2cAddress, dataToWrite, 2, dataRead, 4, &i2cStatus);
				et->status = CAT_DOING_READ_BEFORE_WRITE;
			}
			break;
		case CAT_DOING_READ:
			if(i2cStatus == TRANSACTION_COMPLETED_OK){
				uint32_t temp = 0;
				temp |= ((uint32_t)dataRead[0]) << 24;
				temp |= ((uint32_t)dataRead[1]) << 16;
				temp |= ((uint32_t)dataRead[2]) << 8;
				temp |= ((uint32_t)dataRead[3]) << 0;
				*(et->variableToReadTo) = temp;

				et->status = CAT_DONE;
			} else if(i2cStatus != TRANSACTION_QUEUED && i2cStatus != TRANSACTION_IN_PROGRESS){
				et->status = CAT_FAIL;
			} else if(compareTimeToNow(timeoutTimer) > 0.03){
				et->status = CAT_FAIL;
			}
			break;
		case CAT_DOING_READ_BEFORE_WRITE:
			if(i2cStatus == TRANSACTION_COMPLETED_OK){
				uint32_t temp = 0;
				temp |= ((uint32_t)dataRead[0]) << 24;
				temp |= ((uint32_t)dataRead[1]) << 16;
				temp |= ((uint32_t)dataRead[2]) << 8;
				temp |= ((uint32_t)dataRead[3]) << 0;
				if( et->dataToWrite == temp){
					et->status = CAT_DONE;
				} else {
					//setup write
					dataToWrite[0] = 0xff & (et->memAddress >> 8);
					dataToWrite[1] = 0xff & (et->memAddress);
					dataToWrite[2] = 0xff & (et->dataToWrite >> 24);
					dataToWrite[3] = 0xff & (et->dataToWrite >> 16);
					dataToWrite[4] = 0xff & (et->dataToWrite >> 8);
					dataToWrite[5] = 0xff & (et->dataToWrite >> 0);
					i2cQueue(et->dev, et->i2cAddress, dataToWrite, 6, 0, 0, &i2cStatus);
					et->status = CAT_DOING_WRITE;
					resetSlowTimer(&timeoutTimer);
				}


			} else if(i2cStatus != TRANSACTION_QUEUED && i2cStatus != TRANSACTION_IN_PROGRESS){
				et->status = CAT_FAIL;
			}
			break;
		case CAT_DOING_WRITE:
			if(i2cStatus == TRANSACTION_COMPLETED_OK){
				et->status = CAT_DONE;
			} else {
				et->status = CAT_FAIL;
			}
			break;
		default:
			break;
		}

		if(et->status == CAT_DONE){
			if(et->doneReg != 0){
				++(*(et->doneReg));
			}
			doneWithQueueFront(&queueFront, &queueBack, QUEUE_SIZE);
		} else if(et->status == CAT_FAIL){
			if(et->doneReg != 0){
				*(et->doneReg) = 0xffff;
			}
			doneWithQueueFront(&queueFront, &queueBack, QUEUE_SIZE);
		}


	}
}

//

