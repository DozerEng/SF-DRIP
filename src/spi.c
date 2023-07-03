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

//*************************************************
//includes
//*************************************************


#include <stdbool.h>
#include "spi.h"
#include "fastcodeUtil.h"
#include "ports.h"
#include "error.h"
#include "queue.h"
#include "stm32f4xx_dma.h"
#include "dma.h"


//*************************************************
//Notes
//*************************************************


//*************************************************
//Defines
//*************************************************


#define COMPLETE_FAIL -1

#define SPI_QUEUE_LENGTH 100
#define NUM_SPI_DEVICES 3

//*************************************************
//Types
//*************************************************
typedef enum {
	SPI_UNCONFIGURED,
	SPI_QUEUED,
	SPI_LOST,
	SPI_IN_PROGRESS,
	SPI_COMPLETE,
	SPI_FAILED,
	SPI_QUEUE_FULL,
} SpiStatus;

typedef enum {
	SPI_BITS_8,
	SPI_BITS_16,
} BitNum;



typedef struct {
	SpiDev dev;
	PortPin csPin;
	uint32_t* txPointer;
	uint32_t* rxPointer;
	SpiStatus status;
	uint32_t* completePointer;
	BitNum bitNum;
	ClockSpec clockSpec;
	BaudDiv baudDiv;
	bool deassertCs;
	int32_t index;
	uint32_t wordCount;
	uint32_t timeStarted;
} SpiTransaction;

typedef struct {
	SPI_TypeDef* port;
	DMA_Stream_TypeDef* rxStream;
	uint32_t rxChannel;
	DMA_Stream_TypeDef* txStream;
	uint32_t txChannel;
} PortDetails;

//*************************************************
//Variables
//*************************************************


static const PortDetails portDetails[] = {
		{SPI1, DMA2_Stream0, DMA_Channel_3, DMA2_Stream3, DMA_Channel_3},
		{SPI2, DMA1_Stream3, DMA_Channel_0, DMA1_Stream4, DMA_Channel_0},//same DMA controller and stream as USART3 and UART4
		{SPI3, DMA1_Stream2, DMA_Channel_0, DMA1_Stream7, DMA_Channel_0}//DMA1_Stream7 conflicts with UART5
};





static SpiTransaction spiTransactions[NUM_SPI_DEVICES][SPI_QUEUE_LENGTH];
//static SpiStatus spiStatus[NUM_SPI_DEVICES];
static uint32_t queueBack[NUM_SPI_DEVICES];
static uint32_t queueFront[NUM_SPI_DEVICES];

static uint32_t resetTimesCounter[NUM_SPI_DEVICES];
static uint32_t busyTimeoutTimesCounter[NUM_SPI_DEVICES];



static uint32_t maxQueueUsed = 0;
static PortPin oldCsPin = 0;
#define CS_PIN_LIST_SIZE 5
static PortPin csPinList[CS_PIN_LIST_SIZE];
static uint32_t csPinListIndex = 0;

static bool inProgress = false;


static uint32_t queueUsed = 0;



//*************************************************
//function prototypes
//*************************************************
static void oneBusFastCode(SpiDev dev);

static void wait();

static void addCsPinToList(PortPin p);
static bool isDmaComplete(SpiDev dev);
static void setupDmaTransfer(SpiTransaction* t);
static void disableDmaAfterTransfer(SpiTransaction* t);
static bool isStatus(SpiTransaction* t, SpiStatus s);
static bool isSpiBusy(SpiDev dev);



//*************************************************
//Code
//*************************************************




/**
 * initialize the spi driver
 */
void spiInit(){




	uint32_t i;

	for (i = 0; i < NUM_SPI_DEVICES; i++){
//		spiDevState[i] = IDLE;
		queueBack[i] = 0;
		queueFront[i] = 0;
		resetTimesCounter[i] = 0;
		busyTimeoutTimesCounter[i] = 0;
	}


}
/**
 * move off-index to the next location
 * don't do anything if there's no data
 */
//static void unqueueTransaction(){
//	if(isTransaction()){
//		int32_t i = offIndex + 1;
//		if(i >= SPI_QUEUE_LENGTH){
//			i = 0;
//		}
//		offIndex = i;
//	}
//}

static void wait(){
	volatile uint32_t i = 0;
	while(i < 3){
		++i;
	}
}



uint32_t getSpiQueueUsage(void){

	return queueUsed;
}
uint32_t getMaxSpiQueueUsage(void){
	return maxQueueUsed;
}












/**
 * this should run in fast code
 */
void spiFastCode(){
//	for(SpiDev i = SPI1_DEV; i <= SPI3_DEV; ++i){
//		oneBusFastCode(i);
//	}

	oneBusFastCode(SPI1_DEV);

	oneBusFastCode(SPI3_DEV);

	oneBusFastCode(SPI2_DEV);

}




static void oneBusFastCode(SpiDev dev){
	if(dev == SPINULL_DEV || dev > SPI3_DEV){
		return;
	}

	SpiTransaction* t = &spiTransactions[dev][queueFront[dev]];

	bool doneTrans = false;

	//first check if an ongoing transaction is complete.

	if(isQueueNotEmpty(&queueFront[dev], &queueBack[dev], SPI_QUEUE_LENGTH) && isStatus(t, SPI_IN_PROGRESS)){


		//0.7us
		if(isDmaComplete(dev) && !isSpiBusy(dev)){
			//indicate that we're done to whoever requested the transaction, if they bothered to give us a pointer
			t->status = SPI_COMPLETE;
			++(*(t->completePointer));
			//clear the last cs pin unless we've been specifically asked to leave it on
			if(t->deassertCs){// || (!isTransaction()) || currentCsPin != transactions[offIndex].csPin){
				setPin(t->csPin, true);

				//1.9us
				wait();


			}
			doneTrans = true;

			inProgress = false;
		} else if(compareTimeToNow(t->timeStarted) > 0.1) {
			//we've timed out so fail
			inProgress = false;
			t->status = SPI_FAILED;
			(*(t->completePointer)) = COMPLETE_FAIL;
			setPin(t->csPin, true);

			oldCsPin = NULL_PIN;
			doneTrans = true;


			wait();

			doneTrans = true;

		}
	}
	//2.4us from start
	if(doneTrans){
		disableDmaAfterTransfer(t);
		//2.9us
		//we've got what we need from this transaction so we can unqueue it now
		doneWithQueueFront(&queueFront[dev], &queueBack[dev], SPI_QUEUE_LENGTH);
	}
	//3.2us
	//now see if a new transaction should be started
	t = &spiTransactions[dev][queueFront[dev]];


	if(isQueueNotEmpty(&queueFront[dev], &queueBack[dev], SPI_QUEUE_LENGTH) && isStatus(t, SPI_QUEUED)){// && SPI_I2S_GetFlagStatus(transactions[offIndex].port, SPI_I2S_FLAG_TXE) == SET){
		//bool rxne = SPI_I2S_GetITStatus(currentPort, SPI_I2S_IT_RXNE);
		//assert cs pin in prep for upcoming send

		//3.5us

		uint32_t c = *(t->completePointer);
		if(isSpiFailed(c)){
			//fail this right away becuase the complete register is indicating a fail
			t->status = SPI_FAILED;
			doneWithQueueFront(&queueFront[dev], &queueBack[dev], SPI_QUEUE_LENGTH);
		} else {




			//add this pin to the list of known chip selects. Make sure all other CS pins are not asserted
			addCsPinToList(t->csPin);
			//if we've changed Cs pin then assume port needs to be reconfigured
			if(oldCsPin == NULL_PIN || oldCsPin != t->csPin){

				oldCsPin = t->csPin;

			}


//			//trying this here
//
//			//slight delay before and after setting CS pin
//
//			wait();
//
//			setPin(t->csPin, false);
//
//			//slight delay before and after setting CS pin
//			wait();


			//to here
			t->timeStarted = getTimeInMicroSeconds();


			setupDmaTransfer(t);


			//10.4us

			t->status = SPI_IN_PROGRESS;
		}
	}
}

static bool isStatus(SpiTransaction* t, SpiStatus s){
	SpiStatus st = t->status;
	return st == s;
}


bool isSpiFailed(uint32_t complete){
	return complete == COMPLETE_FAIL;
}
bool isSpiLost(SpiStatus s){
	return s == SPI_LOST;
}
/**
 * called from slow code to push a transaction onto the queue
 * @param dev - the spi port to use
 * @param csPin - the pin for chip select
 * @param txData - pointer to data to be sent in one 16 bit transaction
 * @param rxData - a pointer to a variable to store the result in
 * @param status - a pointer to a reg that will be incremented when transaction complete
 * @param byteCount - the number of bytes to send. This doesn't do anything yet
 * @param deassertCs - if true then CS will be deasserted at end of 16 bit transaction. If false then CS will NOT be asserted. This assumes that transaction to same CS will follow!
 * @param complete - increments whenever an transaction successfully completes.
 * @return true if successfully queued. False if queue full.
 */
//bool spiQueue(SpiDev spi, PortPin csPin, uint16_t txData, BitNum bitNum, ClockSpec clockSpec, BaudDiv baudDiv, bool deassertCs);

bool spiQueue16(SpiDev dev, PortPin csPin, uint16_t* txData,  uint16_t* rxData, uint32_t wordCount, ClockSpec clockSpec, BaudDiv baudDiv, bool deassertCs, uint32_t* complete){
	if(dev == SPINULL_DEV){
		return false;
	}
	if(isSpiFailed(*complete)){
		return false;
	}
	bool result = false;

	if(isQueueNotFull(&queueFront[dev], &queueBack[dev], SPI_QUEUE_LENGTH)){//check to be sure we haven't over-run the queue
		result = true;

		SpiTransaction *t = &(spiTransactions[dev][queueBack[dev]]);

		// set up the transaction
		t->dev = dev;
		t->csPin = csPin;
		t->txPointer = (uint32_t*)txData;
		t->rxPointer = (uint32_t*)rxData;
		t->bitNum = SPI_BITS_16;
		t->clockSpec = clockSpec;
		t->deassertCs = deassertCs;
		t->baudDiv = baudDiv;
		t->status = SPI_QUEUED;
		t->completePointer = complete;
		t->wordCount = wordCount;


		justAddedToQueueBack(&queueFront[dev], &queueBack[dev], SPI_QUEUE_LENGTH);
	} else {

	}
	return result;
}

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
bool spiQueue8(SpiDev dev, PortPin csPin, uint8_t* txData,  uint8_t* rxData, uint32_t wordCount, ClockSpec clockSpec, BaudDiv baudDiv, bool deassertCs, uint32_t* complete){
	if(dev == SPINULL_DEV){
		return false;
	}
	bool result = false;
	if(isSpiFailed(*complete)){
		return false;
	}

	if(isQueueNotFull(&queueFront[dev], &queueBack[dev], SPI_QUEUE_LENGTH)){//check to be sure we haven't over-run the queue
		result = true;

		SpiTransaction *t = &(spiTransactions[dev][queueBack[dev]]);

		// set up the transaction
		t->dev = dev;
		t->csPin = csPin;
		t->txPointer = (uint32_t*)txData;
		t->rxPointer = (uint32_t*)rxData;
		t->bitNum = SPI_BITS_8;
		t->clockSpec = clockSpec;
		t->deassertCs = deassertCs;
		t->baudDiv = baudDiv;
		t->status = SPI_QUEUED;
		t->completePointer = complete;
		t->wordCount = wordCount;

		justAddedToQueueBack(&queueFront[dev], &queueBack[dev], SPI_QUEUE_LENGTH);
	} else {

	}
	return result;
}



/**
 * adds the new pin to the list of used pins. De-asserts all other pins
 */
static void addCsPinToList(PortPin p){
	if(csPinListIndex >= CS_PIN_LIST_SIZE){
		csPinListIndex = CS_PIN_LIST_SIZE - 1;
		return;
	}
	bool found = false;
	for(uint32_t i = 0; i < csPinListIndex; ++i){
		PortPin pt = csPinList[i];
		if(pt == p){
			found = true;
		} else {
			if(pt != NULL_PIN && !isPinOutputSet(pt)){
				logError(SPI_CS_FAIL_ERROR, pt<<8 | p);
			}
			setPin(pt, true);
		}
	}
	if(!found && csPinListIndex < CS_PIN_LIST_SIZE){
		csPinList[csPinListIndex] = p;
		++csPinListIndex;
	}
}

static bool isDmaComplete(SpiDev dev){
	bool result = false;

	PortDetails pd = portDetails[dev];



	if(pd.rxStream == 0 || DMA_GetCmdStatus(pd.rxStream) == DISABLE){
		result = true;
	}


	return result;
}

static void disableDmaAfterTransfer(SpiTransaction* t){
	PortDetails pd = portDetails[t->dev];
	//Disable DMA streams for Tx and Rx in the DMA registers, if the streams are used.

//	DMA_Cmd(pd.rxStream, DISABLE);
//	DMA_Cmd(pd.txStream, DISABLE);

	pd.rxStream->CR &= ~(uint32_t)DMA_SxCR_EN;
	pd.txStream->CR &= ~(uint32_t)DMA_SxCR_EN;

	//Disable the SPI by following the SPI disable procedure.
//	SPI_Cmd(pd.port, DISABLE);

	//Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the SPI_CR2 register, if DMA Tx and/or DMA Rx are used.
	SPI_I2S_DMACmd(pd.port, SPI_I2S_DMAReq_Tx + SPI_I2S_DMAReq_Rx, DISABLE);
}
static void setupDmaTransfer(SpiTransaction* t){
	PortDetails pd = portDetails[t->dev];


	//first ensure the port is disabled
//	SPI_Cmd(pd.port, DISABLE);

	//Enable DMA Rx buffer in the RXDMAEN bit in the SPI_CR2 register, if DMA Rx is used.
	SPI_I2S_DMACmd(pd.port, SPI_I2S_DMAReq_Rx, ENABLE);

	//Enable DMA streams for Tx and Rx in DMA registers, if the streams are used.





	//disable streams prior to configuring
//	DMA_Cmd(portDetails[t->dev].rxStream, DISABLE);
//	DMA_Cmd(portDetails[t->dev].txStream, DISABLE);
	portDetails[t->dev].rxStream->CR &= ~(uint32_t)DMA_SxCR_EN;
	portDetails[t->dev].txStream->CR &= ~(uint32_t)DMA_SxCR_EN;


	//first do rx stream. This will take from the SPI data reg and put in memory

	DMA_InitTypeDef ditd;
//	DMA_StructInit(&ditd);
	ditd.DMA_Channel = portDetails[t->dev].rxChannel;
	ditd.DMA_PeripheralBaseAddr = (uint32_t)(&(pd.port->DR));
	ditd.DMA_Memory0BaseAddr = (uint32_t)t->rxPointer;
	ditd.DMA_DIR = DMA_DIR_PeripheralToMemory;
	ditd.DMA_BufferSize = t->wordCount;
	ditd.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	ditd.DMA_MemoryInc = DMA_MemoryInc_Enable;
	ditd.DMA_PeripheralDataSize = (t->bitNum == SPI_BITS_8) ? DMA_PeripheralDataSize_Byte : DMA_PeripheralDataSize_HalfWord;
	ditd.DMA_MemoryDataSize = (t->bitNum == SPI_BITS_8) ? DMA_MemoryDataSize_Byte : DMA_MemoryDataSize_HalfWord;
	ditd.DMA_Mode = DMA_Mode_Normal;
	ditd.DMA_Priority = DMA_Priority_VeryHigh;
	ditd.DMA_FIFOMode = DMA_FIFOMode_Disable;
	ditd.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	ditd.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	ditd.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(portDetails[t->dev].rxStream, &ditd);


	//5.8us
	//now do tx stream. This will take from memory and put in the SPI data reg


//	DMA_StructInit(&ditd);
	ditd.DMA_Channel = portDetails[t->dev].txChannel;
	ditd.DMA_PeripheralBaseAddr = (uint32_t)(&(pd.port->DR));
	ditd.DMA_Memory0BaseAddr = (uint32_t)t->txPointer;
	ditd.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	ditd.DMA_BufferSize = t->wordCount;
	ditd.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	ditd.DMA_MemoryInc = DMA_MemoryInc_Enable;
	ditd.DMA_PeripheralDataSize = (t->bitNum == SPI_BITS_8) ? DMA_PeripheralDataSize_Byte : DMA_PeripheralDataSize_HalfWord;
	ditd.DMA_MemoryDataSize = (t->bitNum == SPI_BITS_8) ? DMA_MemoryDataSize_Byte : DMA_MemoryDataSize_HalfWord;
	ditd.DMA_Mode = DMA_Mode_Normal;
	ditd.DMA_Priority = DMA_Priority_VeryHigh;
	ditd.DMA_FIFOMode = DMA_FIFOMode_Disable;
	ditd.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	ditd.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	ditd.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(portDetails[t->dev].txStream, &ditd);






	//6.7us

	//clear all event flags for stream
	clearDmaFlags(portDetails[t->dev].rxStream);
	clearDmaFlags(portDetails[t->dev].txStream);

	//enable both streams
//	DMA_Cmd(portDetails[t->dev].rxStream, ENABLE);
//	DMA_Cmd(portDetails[t->dev].txStream, ENABLE);

	portDetails[t->dev].rxStream->CR |= (uint32_t)DMA_SxCR_EN;
	portDetails[t->dev].txStream->CR |= (uint32_t)DMA_SxCR_EN;


	//now configure the SPI port
	SPI_InitTypeDef sitd;
//	SPI_StructInit(&sitd);
	sitd.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	sitd.SPI_Mode = SPI_Mode_Master;
	switch(t->bitNum){
	case SPI_BITS_8:
		sitd.SPI_DataSize = SPI_DataSize_8b;
		break;
	default:
	case SPI_BITS_16:
		sitd.SPI_DataSize = SPI_DataSize_16b;
		break;
	}
	switch(t->clockSpec){
	case SPI_CPOL_0_CPHA_0:
		sitd.SPI_CPOL = SPI_CPOL_Low;
		sitd.SPI_CPHA = SPI_CPHA_1Edge;
		break;
	case SPI_CPOL_0_CPHA_1:
		sitd.SPI_CPOL = SPI_CPOL_Low;
		sitd.SPI_CPHA = SPI_CPHA_2Edge;
		break;
	case SPI_CPOL_1_CPHA_0:
		sitd.SPI_CPOL = SPI_CPOL_High;
		sitd.SPI_CPHA = SPI_CPHA_1Edge;
		break;
	case SPI_CPOL_1_CPHA_1:
		sitd.SPI_CPOL = SPI_CPOL_High;
		sitd.SPI_CPHA = SPI_CPHA_2Edge;
		break;
	}
	sitd.SPI_NSS = SPI_NSS_Soft;
	sitd.SPI_BaudRatePrescaler = (uint16_t)t->baudDiv;
	sitd.SPI_FirstBit = SPI_FirstBit_MSB;
	sitd.SPI_CRCPolynomial = 7;




	//init port
	SPI_Init(pd.port, &sitd);

	//trying this here

	//slight delay before and after setting CS pin

	wait();

	setPin(t->csPin, false);

	//slight delay before and after setting CS pin
	wait();


	//to here


	//now send

	//Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CR2 register, if DMA Tx is used.
	SPI_I2S_DMACmd(pd.port, SPI_I2S_DMAReq_Tx, ENABLE);

//	SPI_I2S_ClearFlag(pd.port, SPI_I2S_FLAG_TXE);
	SPI_Cmd(pd.port, ENABLE);




}
static bool isSpiBusy(SpiDev dev){
	bool result = true;
	if(dev > SPINULL_DEV && dev <= SPI3_DEV){
		if(RESET == SPI_I2S_GetFlagStatus(portDetails[dev].port, SPI_I2S_FLAG_BSY)){
			result = false;
		}
	}
	return result;
}




