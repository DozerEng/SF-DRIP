/*
Copyright (c) 2017-2018 STARFISH PRODUCT ENGINEERING INC.

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

#ifndef SPI_H_
#define SPI_H_

#include <stdbool.h>
#include "stm32f4xx_spi.h"
#include "ports.h"
//#include "_processorGlobal.h"


typedef enum {
	SPINULL_DEV = -1,
	SPI1_DEV = 0,
	SPI2_DEV = 1,
	SPI3_DEV = 2
} SpiDev;


typedef enum {
	SPI_DIV_2 = SPI_BaudRatePrescaler_2,
	SPI_DIV_4 = SPI_BaudRatePrescaler_4,
	SPI_DIV_8 = SPI_BaudRatePrescaler_8,
	SPI_DIV_16 = SPI_BaudRatePrescaler_16,
	SPI_DIV_32 = SPI_BaudRatePrescaler_32,
	SPI_DIV_64 = SPI_BaudRatePrescaler_64,
	SPI_DIV_128 = SPI_BaudRatePrescaler_128,
	SPI_DIV_256 = SPI_BaudRatePrescaler_256,
} BaudDiv;
typedef enum {
	SPI_CPOL_0_CPHA_0,
	SPI_CPOL_0_CPHA_1,
	SPI_CPOL_1_CPHA_0,
	SPI_CPOL_1_CPHA_1,
} ClockSpec;







/**
 * initialize the spi driver
 */
void spiInit();
/**
 * this should run in fast code
 */
void spiFastCode(void);




/**
 * @return the amount of bytes in the queue
 */
uint32_t getSpiQueueUsage(void);
/**
 * @return the amount of bytes in the queue
 */
uint32_t getMaxSpiQueueUsage(void);
/**
 * tests if the current spi transaction is using the specified cs pin
 * @return true if it is
 */
bool isCurrentCsPin(uint8_t csPin);


bool isSpiFailed(uint32_t complete);

/**
 * called from slow code to push a transaction onto the queue
 * @param spi - the spi port to use
 * @param csPin - the pin for chip select
 * @param txData - data to be sent in one 16 bit transaction
 * @param rxReg - a pointer to a variable to store the result in
 * @param complete - a pointer to a reg that will be incremented when transaction complete
 * @param deassertCs - if true then CS will be deasserted at end of 16 bit transaction. If false then CS will NOT be asserted. This assumes that transaction to same CS will follow!
 * @return true if successfully queued. False if queue full.
 */
bool spiQueue16(SpiDev dev, PortPin csPin, uint16_t* txData,  uint16_t* rxData, uint32_t wordCount, ClockSpec clockSpec, BaudDiv baudDiv, bool deassertCs, uint32_t* complete);
/**
 * called from slow code to push a transaction onto the queue
 * @param dev - the spi port to use
 * @param csPin - the pin for chip select
 * @param txData - pointer to data to be sent in one 8 bit transaction
 * @param rxData - a pointer to a variable to store the result in
 * @param status - a pointer to a reg that will be incremented when transaction complete
 * @param byteCount - the number of bytes to send. This doesn't do anything yet
 * @param deassertCs - if true then CS will be deasserted at end of 8 bit transaction. If false then CS will NOT be asserted. This assumes that transaction to same CS will follow!
 * @param complete - increments whenever an transaction successfully completes.
 * @return true if successfully queued. False if queue full.
 */
bool spiQueue8(SpiDev dev, PortPin csPin, uint8_t* txData,  uint8_t* rxData, uint32_t wordCount, ClockSpec clockSpec, BaudDiv baudDiv, bool deassertCs, uint32_t* complete);

#endif /* SPI_H_ */
