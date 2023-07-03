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
#include <as5048A.h>
#include "fastcodeUtil.h"

//*************************************************
//defines
//*************************************************
#define POLLING_INTERVAL_US 50000
#define MAX_ENC_COUNT 4
#define READ_ANGLE_REGISTER_CMD (1<<15 | 1<<14 | 0x3fff) //this has 16 high bits so it should pass parity check
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
	uint32_t complete;
	uint16_t txData[1];
	uint16_t rxData[1];

} Encoder;

//*************************************************
//Variables
//*************************************************
static uint32_t m_pollingTimer = 0;
static Encoder m_encoders[MAX_ENC_COUNT];
static uint32_t m_pollingTimeUs = 50000;

//*************************************************
//function prototypes
//*************************************************
static void decodeRxData(Encoder* e);
//static uint16_t addParity(uint16_t n);

//*************************************************
//code
//*************************************************
void as5048ASlowCode(void){
	if(slowTimer(&m_pollingTimer, m_pollingTimeUs)){
		//create and indexed pointer to each struct
		for(uint32_t i = 0; i < MAX_ENC_COUNT; ++i){
			Encoder* e = &(m_encoders[i]);
			//did previous transaction complete?
			if(e->complete >= 1){	//rxdata1 must have results in it

				decodeRxData(e);	//convert to temperature value
				e->complete = 0;
			}
			//setup the next transaction
			if(e->enabled){
				//queue read
				e->complete = 0;
				e->txData[0] = READ_ANGLE_REGISTER_CMD;//addParity(0x4000 | 0x3fff);//read angle from register 0x3fff
				spiQueue16(e->spi, e->cs, e->txData, e->rxData, 1, SPI_CPOL_0_CPHA_1, SPI_DIV_8, true, &(e->complete));
			}
		}
	}
}
/**
 * @param seconds the time between requesting samples
 */
void as5048AInit(float seconds){
	float pTimeUs = seconds*1e6;

	uint32_t t = (uint32_t)pTimeUs;
	m_pollingTimeUs = t;
}
/**
 * Configure a connected as5048
 * @param i specify the index of the chip configuration, to a maximum of 4
 * @param spi specify which SPI port this part in connected to
 * @param cs specify port pin used for chip select
 * @param appDataIndex specify which item in the appdata array will be used for the temperature data. If -1 then nothing happens
 */
void as5048AConfig(uint32_t i, SpiDev spi, PortPin cs, bool enable, int32_t appDataIndex, float scale, float offset){

	if(i >= MAX_ENC_COUNT){
		return;
	}
	if(spi == SPINULL_DEV){
		m_encoders[i].enabled = false;
		return;
	}
	m_encoders[i].cs = cs;
	m_encoders[i].spi = spi;
	m_encoders[i].appDataIndex = appDataIndex;
	m_encoders[i].enabled = enable;
	m_encoders[i].scale = scale;
	m_encoders[i].offset = offset;

}

float getAs5048APosition(uint32_t i){

	float result = NAN;
	Encoder* e = &(m_encoders[i]);
	if(i < MAX_ENC_COUNT){
		result = e->position;
	}

	return result;
}

//void setAs5048(uint32_t i, float pos){
//	if(i < MAX_ENC_COUNT){
//
//	}
//}



static void decodeRxData(Encoder* e){
	//TODO: might want to check parity

	uint32_t recievedData = 0;



	recievedData += e->rxData[0];	//put data into



	recievedData &= 0x3fff;			//remove the top 2 bits

	e->position = ((((float)recievedData)*(1.0f/16384.0f))*(e->scale))-(e->offset);	//divide by max value, scale & offset

	if(e->appDataIndex >= 0){
		setTxAppData(e->appDataIndex, e->position);
	}
}
//static uint16_t addParity(uint16_t in){
//	uint16_t result = in & 0x7fff;
//	uint32_t bitCount = 0;
//	uint16_t acc = result;
//	for(uint32_t i = 0; i < 16; ++i){
//		if((acc & 0b1) != 0){
//			++bitCount;
//		}
//		acc >>= 1;
//	}
//	if((bitCount & 0b1) != 0){//if the count is odd then add a parity bit
//		result |= 0x8000;
//
//	}
//	return result;
//}
