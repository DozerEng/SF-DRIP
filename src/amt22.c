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
//notes
//*************************************************


//*************************************************
//includes
//*************************************************
#include "amt22.h"
#include "fastcodeUtil.h"

//*************************************************
//defines
//*************************************************
#define POLLING_INTERVAL_US 1000
#define MAX_ENC_COUNT 4
#define CLK_DIV SPI_DIV_256 //SPI_DIV_32
//*************************************************
//Types
//*************************************************

typedef struct {
    PortPin cs;
    SpiDev spi;
	int32_t appDataIndex;
	bool enabled;
	float scale;
	float offset;
	float position;
	bool twelveNotFourteenBits;
	bool setZeroFlag;
	bool setResetFlag;
	uint32_t complete;
	uint8_t txData1[9]; //= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //8 registers + address byte
	uint8_t rxData1[9];//8 registers + address byte
	uint8_t txData2[9]; //= {0x80, 0xC4};	//Write //VBIAS On, Normally off Conversion, start 1 conversion, 3 wire, auto delay fault detect, fault status, 60Hz filter
	uint8_t rxData2[9];
}Encs;

//*************************************************
//Variables
//*************************************************
static uint32_t pollingTimer = 0;
static Encs m_encs[MAX_ENC_COUNT];

//*************************************************
//function prototypes
//*************************************************
static void decodeRxData(Encs* e);
static void setAmt22Zero(uint32_t i);
static void resetAmt22(uint32_t i);

//*************************************************
//code
//*************************************************
void amt22SlowCode(float samplePeriodSeconds){
	uint32_t t = getTimeInMicroSeconds();
	if(compareTimeMicroSec(t, pollingTimer) > samplePeriodSeconds){
		pollingTimer = t;
//	if(slowTimer(&pollingTimer, POLLING_INTERVAL_US)){
		//create and indexed pointer to each struct
		for(uint32_t i = 0; i < MAX_ENC_COUNT; ++i){
			Encs* e = &(m_encs[i]);
			//did previous transaction complete?
			if(e->complete >= 1){	//rxdata1 must have results in it

				decodeRxData(e);	//convert to temperature value
				e->complete = 0;
			}
			//setup the next transaction
			if(e->enabled){
				//queue read
				e->complete = 0;
				uint32_t wordCount = 0;
				e->txData1[wordCount++] = 0x00;
				//e->txData1[wordCount++] = 0x00;
				spiQueue8(e->spi, e->cs, e->txData1, e->rxData1, wordCount, SPI_CPOL_0_CPHA_0, CLK_DIV, false, &(e->complete));
				spiQueue8(e->spi, e->cs, e->txData1, e->rxData2, wordCount, SPI_CPOL_0_CPHA_0, CLK_DIV, true, &(e->complete));
			}
			//set the zero position
			if(e->setZeroFlag && e->enabled){
				e->complete = 0;
				uint32_t wordCount = 0;
				e->txData2[wordCount++] = 0x00;
				e->txData2[wordCount++] = 0x70;
				spiQueue8(e->spi, e->cs, e->txData2, e->rxData2, wordCount, SPI_CPOL_0_CPHA_0, CLK_DIV, true, &(e->complete));
				e->setZeroFlag = false;
			}
			//resets the encoder
			if(e->setResetFlag && e->enabled){
				e->complete = 0;
				uint32_t wordCount = 0;
				e->txData2[wordCount++] = 0x00;
				e->txData2[wordCount++] = 0x60;
				spiQueue8(e->spi, e->cs, e->txData2, e->rxData2, wordCount, SPI_CPOL_0_CPHA_0, CLK_DIV, true, &(e->complete));
				e->setResetFlag = false;
			}
//			setPin(GPIO_B4_PIN, false);//TODO remove
		}
	}
}
void amt22Init(void){

}
/**
 * Configure a connected amt22
 * @param i specify the index of the chip configuration, to a maximum of 4
 * @param spi specify which SPI port this part in connected to
 * @param cs specify port pin used for chip select
 * @param appDataIndex specify which item in the appdata array will be used for the temperature data. If -1 then nothing happens
 */
void amt22Config(uint32_t i, SpiDev spi, PortPin cs, bool enable, int32_t appDataIndex, bool twelveNotFourteenBits, float scale, float offset){

	if(i >= MAX_ENC_COUNT){
		return;
	}
	if(spi == SPINULL_DEV){
		m_encs[i].enabled = false;
		return;
	}
	m_encs[i].cs = cs;
	m_encs[i].spi = spi;
	m_encs[i].appDataIndex = appDataIndex;
	m_encs[i].enabled = enable;
	m_encs[i].twelveNotFourteenBits = twelveNotFourteenBits;
	m_encs[i].scale = scale;
	m_encs[i].offset = offset;

}

float getAmt22Position(uint32_t i){

	float result = NAN;

	if(i < MAX_ENC_COUNT){
		result = m_encs[i].position;
	}

	return result;
}

void setAmt22Zero(uint32_t i){

	if(i < MAX_ENC_COUNT){
		m_encs[i].setZeroFlag = true;	//tells the slow code to set the zero point when it comes round
	}
}

void resetAmt22(uint32_t i){

	if(i < MAX_ENC_COUNT){
		m_encs[i].setResetFlag = true;	//tells the slow code to reset the ecoder when it comes round
	}
}

static void decodeRxData(Encs* e){

	uint32_t recievedData = 0;
//	uint32_t recievedMask = 0;
//	uint32_t i = 0;
//	uint32_t check1 = 0;
//	uint32_t check2 = 0;
//	uint32_t bitmaskodd = 0b0010101010101010;
//	uint32_t bitmaskeven = 0b0001010101010101;
//
//	check1 = recievedData & 0x01;
//	for(i = 1; i < 6; i++ ){
//		recievedMask = recievedData & bitmaskeven;
//		check1 ^= recievedMask >> i*2;
//	}
//	check2 = (recievedData & 0x02) >> 1;
//	for(i = 1; i < 6; i++ ){
//		recievedMask = recievedData & bitmaskodd;
//		check1 ^= recievedMask >> ((i*2)+1);
//	}
//	if(e->rxData1[0x80] == check1 && e->rxData1[0x40] == check2){
//		asm("nop");
//	}


	uint32_t highByte = (uint32_t)(e->rxData1[0]);
	uint32_t lowByte = (uint32_t)(e->rxData2[0]);

	uint32_t check = highByte ^ lowByte;
	check = (check >> 0) ^ (check >> 2) ^ (check >> 4) ^ (check >> 6);
	check ^= 0b11;
	check &= 0b11;
	//check should now be 0 I think.

	//once checkbits are shown to be working put this stuff in the embedded ifs above
	recievedData += highByte & 0x3f;	//put data into
	recievedData <<= 8;
	recievedData += lowByte;


//	recievedData = recievedData & 0b0011111111111111;					//removes the checksums
	if(check == 0){
		e->position = (((float)recievedData)*(1.0f/16384.0f)*(e->scale))-(e->offset);	//divide by max value, scale & offset
	} else {
		asm("nop");//this will run if the checksum fails. It's only useful for debugging
	}




//	if(e->position < 20.0f){
//		setPin(GPIO_B4_PIN, true);//debug remove
//	}

	if(e->appDataIndex >= 0){
		setTxAppData(e->appDataIndex, e->position);
	}
}
