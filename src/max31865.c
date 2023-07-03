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
#include "max31865.h"
#include "fastcodeUtil.h"

//*************************************************
//defines
//*************************************************
#define POLLING_INTERVAL_US 1000000
#define MAX_RTD_COUNT 4
#define FAST_SPI_DIV SPI_DIV_16
#define SLOW_SPI_DIV SPI_DIV_256
//*************************************************
//Types
//*************************************************

typedef struct {
    PortPin cs;
    SpiDev spi;
	int32_t appDataIndex;
	bool enabled;
	float temperatureC;
	bool threeWireNotFourWire;
	uint32_t complete;
	uint8_t txData1[9]; //= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //8 registers + address byte
	uint8_t rxData1[9];//8 registers + address byte
	uint8_t txData2[9]; //= {0x80, 0xC4};	//Write //VBIAS On, Normally off Conversion, start 1 conversion, 3 wire, auto delay fault detect, fault status, 60Hz filter
	uint8_t rxData2[9];
	bool lowSpeedSpi;
	float timeConstantCoefficient;
}Rtds;

//*************************************************
//Variables
//*************************************************
static uint32_t pollingTimer = 0;
static Rtds m_rtds[MAX_RTD_COUNT];
static float m_samplePeriodSeconds = 0.1f;

//*************************************************
//function prototypes
//*************************************************
static void decodeRxData(Rtds* r);

//*************************************************
//code
//*************************************************

/**
 * must be run regularly to ensure proper operation of chip
 * @param pollingTimeSeconds time between sampling temperatures. All chips will be sampled at once.
 */
void max31865SlowCode(float pollingTimeSeconds){
	m_samplePeriodSeconds = pollingTimeSeconds;
	if(compareTimeToNow(pollingTimer) > pollingTimeSeconds){
		pollingTimer = getTimeInMicroSeconds();
		//create and indexed pointer to each struct
		for(uint32_t i = 0; i < MAX_RTD_COUNT; ++i){
			Rtds* r = &(m_rtds[i]);
			//did previous transaction complete?



			if(!r->enabled){
				//don't do anything if this channel is not enabled
			} else if(r->complete >= 2){
				//if we've got results then decode them

				decodeRxData(r);	//convert to temperature value
				r->complete = 0;
			} else {
				//setup read for next sample
				//queue read from all registers
				r->complete = 0;
				uint32_t wordCount = 0;
				r->txData1[wordCount++] = 0;		//read byte
				r->txData1[wordCount++] = 0;
				r->txData1[wordCount++] = 0;
				r->txData1[wordCount++] = 0;
				r->txData1[wordCount++] = 0;
				r->txData1[wordCount++] = 0;
				r->txData1[wordCount++] = 0;
				r->txData1[wordCount++] = 0;
				r->txData1[wordCount++] = 0;
				BaudDiv bd = r->lowSpeedSpi ? SLOW_SPI_DIV : FAST_SPI_DIV;
				spiQueue8(r->spi, r->cs, r->txData1, r->rxData1, wordCount, SPI_CPOL_0_CPHA_1, bd, true, &(r->complete));

				//queue chip configuration, start single conversion
				wordCount = 0;
				r->txData2[wordCount++] = 0b10000000;		//address write byte
				r->txData2[wordCount++] = r->threeWireNotFourWire ? 0b10110000 : 0b10100000;//0b10110000 : 10100000;
				spiQueue8(r->spi, r->cs, r->txData2, r->rxData2, wordCount, SPI_CPOL_0_CPHA_1, bd, true, &(r->complete));

			}
		}
	}
}
void max31865Init(void){

}
/**
 * Configure a connected MAX31865
 * @param i specify the index of the chip configuration, to a maximum of 4
 * @param spi specify which SPI port this part in connected to
 * @param cs specify port pin used for chip select
 * @param appDataIndex specify which item in the appdata array will be used for the temperature data. If -1 then nothing happens
 * @param lowSpeedSpi when true then lowest SPI clock speed used
 * @param timeConstantSeconds the time constant applied to the data
 *
 */
void max31865Config(uint32_t i, SpiDev spi, PortPin cs, bool enable, int32_t appDataIndex, bool threeWireNotFourWire, bool lowSpeedSpi, float timeConstantSeconds){

	if(i >= MAX_RTD_COUNT){
		return;
	}
	if(spi == SPINULL_DEV){
		m_rtds[i].enabled = false;
		return;
	}
	m_rtds[i].cs = cs;
	m_rtds[i].spi = spi;
	m_rtds[i].appDataIndex = appDataIndex;
	m_rtds[i].enabled = enable;
	m_rtds[i].threeWireNotFourWire = threeWireNotFourWire;
	m_rtds[i].lowSpeedSpi = lowSpeedSpi;
	m_rtds[i].timeConstantCoefficient = m_samplePeriodSeconds/timeConstantSeconds;//this is an approximation

}

float getMax31865Temperature(uint32_t i){

	float result = NAN;

	if(i < MAX_RTD_COUNT){
		result = m_rtds[i].temperatureC;
	}

	return result;
}

static void decodeRxData(Rtds* r){

	uint32_t receivedData = 0;
	float temperatureCelcius = NAN;
	if( true /*!(r->rxData1[0] == 0x00)*/){	//there is something strange happening that isnt reflected in the fault bits.
		receivedData = ((uint32_t)(r->rxData1[2])) << 8;
		receivedData |= (uint32_t)(r->rxData1[3]);
		receivedData >>= 1;						//get rid of the fault bit, this is now the ratio between the RTD resistance and the 400R reference

		//THIS EXPECTS A 400R RESISTER AS RREF!!!!!!!!!!!!!!!!!!!!!!!!!!
		if(receivedData >= 0x7fff){
			//consider this inf
			temperatureCelcius = INFINITY;
		} else if(receivedData <= 0){
			//consider this -inf
			temperatureCelcius = -INFINITY;
		} else {
			temperatureCelcius = (((float)receivedData)/32.0f)-256.0f; //from page 11 on the datasheet this should give us within 2C error bound and avoids a lot of horrible math
		}
		if(finitef(r->temperatureC)){
			r->temperatureC += (temperatureCelcius - r->temperatureC) * r->timeConstantCoefficient;
		} else {
			r->temperatureC = temperatureCelcius;
		}

		if(r->appDataIndex >= 0){				//put into appdata if desired (if appdata >= 0)
			//not sure what this means yet
			setTxAppData(r->appDataIndex, r->temperatureC);
		}
	}
}
