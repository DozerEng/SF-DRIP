/*
Copyright (c) 2012-2018 STARFISH PRODUCT ENGINEERING INC.

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

 
#ifndef I2C_H_
#define I2C_H_


#include "stm32f4xx_i2c.h"
#include <stdbool.h>
#include <stdint.h>





typedef enum {
	NULL_TRANSACTION = 0,
	NO_TRANSACTION_YET = 1,  // after firmware initialised and before any transaction
	TRANSACTION_QUEUED = 2,
	TRANSACTION_IN_PROGRESS = 3,
	TRANSACTION_COMPLETED_OK = 4,
	TRANSACTION_TIMED_OUT = 5,
	TRANSACTION_QUEUE_FULL = 6,
	TRANSACTION_TX_ADDRESS_NACK = 7,//address prior to tx was nacked
	TRANSACTION_RX_ADDRESS_NACK = 8,//address prior to rx was nacked
	TRANSACTION_TX_DATA_NACK = 9,//tx data byte was nacked
	TRANSACTION_RX_DATA_NACK = 10,//rx data was nacked
	TRANSACTION_RX_BUS_ERROR = 11//bus error
} TransactionStatus;

typedef enum {
	I2CNULL_DEV = -1,
	I2C1_DEV = 0,
	I2C2_DEV = 1,
	I2C3_DEV = 2,

} I2cDev;



/**
 * returns the transaction status
 */
TransactionStatus i2cGetStatus(I2cDev dev);

/**
 * puts a new transaction in the queue
 * @return true if successful, false if queue is full
 */
bool i2cQueue(I2cDev dev, uint8_t i2cAddr, uint8_t *txData, uint32_t numBytesToWrite, uint8_t *rxData, uint32_t numBytesToRead, TransactionStatus *status);

/**
 * queries the amount of room left in the queue.
 * It is advised to call this prior to adding to the queue to ensure there's room
 */
uint32_t getI2cQueueRoom(I2cDev dev);

/**
 * call at a periodic interval
 * preferably faster than 1ms intervals for all active i2c devices
 * (and definitely faster than 2ms, or the timeout will trip for no good reason)
 */
void i2cFastcode();
/**
 * must call this before starting the periodic poll to i2cFastcode
 */
void i2cInit(I2cDev dev);

uint32_t getI2cResetTimeoutCount(I2cDev dev);
uint32_t getI2cBusyTimeoutCount(I2cDev dev);


#endif // I2C_H_

