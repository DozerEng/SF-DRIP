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

#include "fastcodeUtil.h"
#include "ad7172.h"
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

//#include "spi.h"
//#include "ports.h"


//*************************************************
//Notes
//*************************************************


//*************************************************
//Defines
//*************************************************

#define AD7172_NUM_CHANNELS 4

#define BYTES_TO_WRITE_5 5
#define BYTES_TO_WRITE_3 3

#define BYTES_TO_WRITE_RESET 8
#define AD7172_REV_VOLTAGE (5.0f)
//*************************************************
//Types
//*************************************************

typedef enum {
	AD7172_IDLE,
	AD7172_FAIL,
	AD7172_COMMS,
	AD7172_DONE,
	AD7172_RESET,
} AdcState;

typedef struct {
	Ad7172Inputs posInputType;
	Ad7172Inputs negInputType;

	uint32_t posInput;
	uint32_t negInput;

	float value;
	SinkSource sink;
	float gain;
	float offset;

	bool enabled;


} Ad7172Channel;

//*************************************************
//Variables
//*************************************************

static uint32_t spiComplete;
static AdcState state = AD7172_IDLE;

static uint8_t bytesToWriteIfMode[BYTES_TO_WRITE_3] = {0b00000010, 0b00001000, 0b01000000};//write to interface mode register: disable hide delay, enable status register appended to data read
static uint8_t rxBytes1[BYTES_TO_WRITE_3];


static uint8_t bytesToReadData[BYTES_TO_WRITE_5] = {0b01000100, 0b00000000, 0b00000000, 0b00000000, 0b00000000};//read from data registers. This will read three bytes of data plus status register
static uint8_t rxBytes2[BYTES_TO_WRITE_5];

static uint8_t bytesToWriteSetupCon[BYTES_TO_WRITE_3] = {0b00100000, 0b00011111, 0b00000000};//write to config reg 0
static uint8_t rxBytes3[BYTES_TO_WRITE_3];

static uint8_t bytesToWriteFiltCon[BYTES_TO_WRITE_3] = {0b00101000, 0b00001011, 0b00010100};//write to filter conf reg 0
static uint8_t rxBytes4[BYTES_TO_WRITE_3];

static uint8_t bytesToSetupChannel0[BYTES_TO_WRITE_3];//this will be configured below to write to all 4 channel registers
static uint8_t rxBytes5[BYTES_TO_WRITE_3];

static uint8_t bytesToSetupChannel1[BYTES_TO_WRITE_3];//this will be configured below to write to all 4 channel registers
static uint8_t rxBytes6[BYTES_TO_WRITE_3];

static uint8_t bytesToSetupChannel2[BYTES_TO_WRITE_3];//this will be configured below to write to all 4 channel registers
static uint8_t rxBytes7[BYTES_TO_WRITE_3];

static uint8_t bytesToSetupChannel3[BYTES_TO_WRITE_3];//this will be configured below to write to all 4 channel registers
static uint8_t rxBytes8[BYTES_TO_WRITE_3];

static uint8_t bytesToWriteAdcMode[BYTES_TO_WRITE_3] = {0b00000001, 0b00100000, 0b00010000};//write to ADC mode register
static uint8_t rxBytes9[BYTES_TO_WRITE_3];



static uint8_t bytesToReset[BYTES_TO_WRITE_RESET] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static uint8_t rxBytesReset[BYTES_TO_WRITE_RESET];







static uint32_t adcTimerReg = 0;


static uint32_t lastChannel = 0;

static Ad7172Channel m_adcChannels[AD7172_NUM_CHANNELS] = {
		{AIN0, REF_NEG, AIN0<<5, REF_NEG, 0.0f, NULL_SINK_SOURCE, 1.0f, 0.0f, false},
		{AIN1, REF_NEG, AIN1<<5, REF_NEG, 0.0f, NULL_SINK_SOURCE, 1.0f, 0.0f, false},
		{AIN2, REF_NEG, AIN2<<5, REF_NEG, 0.0f, NULL_SINK_SOURCE, 1.0f, 0.0f, false},
		{AIN3, REF_NEG, AIN3<<5, REF_NEG, 0.0f, NULL_SINK_SOURCE, 1.0f, 0.0f, false}
};


//static uint32_t posInputs[AD7172_NUM_CHANNELS];
//static uint32_t negInputs[AD7172_NUM_CHANNELS];
//static float adcValues[AD7172_NUM_CHANNELS];
//static Ad7172Inputs posInputTypes[AD7172_NUM_CHANNELS];
//static Ad7172Inputs negInputTypes[AD7172_NUM_CHANNELS];




static PortPin m_csPin;
static SpiDev m_spiDev;


//static uint8_t maxSpiIndex = 0;

#define SPI_CS SPI_CPOL_1_CPHA_1
//#define CS CPOL_0_CPHA_0
#define SPI_BD SPI_DIV_16



//*************************************************
//function prototypes
//*************************************************




static Ad7172Channel* getChannel(uint32_t c);
static uint32_t getMask(Ad7172Inputs i);
/**
 * sets up the comms for the specified state
 * @return the state that we should switch to when the comms completes.
 */
static void setupState(AdcState s, uint32_t c);
static void configChannel(uint32_t j, uint32_t c, uint8_t* txBytes);

//*************************************************
//Code
//*************************************************


void ad7172ConfigAll(Ad7172Inputs pos0, Ad7172Inputs neg0, Ad7172Inputs pos1, Ad7172Inputs neg1, Ad7172Inputs pos2, Ad7172Inputs neg2, Ad7172Inputs pos3, Ad7172Inputs neg3){
	m_adcChannels[0].posInput = getMask(pos0)<<5;
	m_adcChannels[1].posInput = getMask(pos1)<<5;
	m_adcChannels[2].posInput = getMask(pos2)<<5;
	m_adcChannels[3].posInput = getMask(pos3)<<5;

	m_adcChannels[0].negInput = getMask(neg0);
	m_adcChannels[1].negInput = getMask(neg1);
	m_adcChannels[2].negInput = getMask(neg2);
	m_adcChannels[3].negInput = getMask(neg3);

	m_adcChannels[0].posInputType = pos0;
	m_adcChannels[1].posInputType = pos1;
	m_adcChannels[2].posInputType = pos2;
	m_adcChannels[3].posInputType = pos3;

	m_adcChannels[0].negInputType = neg0;
	m_adcChannels[1].negInputType = neg1;
	m_adcChannels[2].negInputType = neg2;
	m_adcChannels[3].negInputType = neg3;






















}
void ad7172Config(uint32_t channel, Ad7172Inputs pos0, Ad7172Inputs neg0, bool enable){
	if(channel >= 0 && channel < 4){
		m_adcChannels[channel].posInput = getMask(pos0)<<5;
		m_adcChannels[channel].negInput = getMask(neg0);
		m_adcChannels[channel].posInputType = pos0;
		m_adcChannels[channel].negInputType = neg0;
		m_adcChannels[channel].enabled = enable;
	}
}










/**
 * Presently set to trigger a data read every 50ms
 * When the read is complete it then configures a new conversion
 * This cycle continues indefinitely
 */
void ad7172SlowCode(SpiDev dev, PortPin csPin){
	m_csPin = csPin;
	m_spiDev = dev;
	static uint32_t channel = 0;
	static bool hasNotReset = true;
	AdcState nextState = state;
	switch(state){
	case AD7172_IDLE:
	case AD7172_FAIL:
		if(slowTimer(&adcTimerReg, 100000)){	//if we've got the data from the last conversion and enough time has passed then queue reading new data
			if(hasNotReset){
				nextState = AD7172_RESET;
				hasNotReset = false;
			} else {
				//now setup next conversion
				lastChannel = channel;
				++channel;
				if(channel >= 4){
					channel = 0;
				}
				if(m_adcChannels[channel].enabled){
					nextState = AD7172_COMMS;
				}
			}
		}
		break;
	case AD7172_COMMS:
		if(!isSpiFailed(spiComplete)){
			if(0 == ((rxBytes2[4]) & 0x80)){
				uint32_t v = 0;
				v |= (rxBytes2[1]) & 0xff;
				v <<= 8;
				v |= (rxBytes2[2]) & 0xff;
				v <<= 8;
				v |= (rxBytes2[3]) & 0xff;
				//v <<= 8;

				uint32_t c = rxBytes2[4] & 0b11;
				float f = v*(2.0f/((float)0xffffff));
				f = (f - 1.0f)*AD7172_REV_VOLTAGE;
				m_adcChannels[c].value = f;
				setSinkSource(m_adcChannels[c].sink, f);//this will write the result to the specified sink if configured
			}
			nextState = AD7172_IDLE;
		} else {
			nextState = AD7172_FAIL;
		}
		break;
	case AD7172_DONE:
	case AD7172_RESET:
		nextState = AD7172_IDLE;
		break;
	}
	if(nextState != state){
		setupState(nextState, channel);
	}
}
float ad7172GetValue(uint32_t channel){
	return m_adcChannels[channel].value;
}

static uint32_t getMask(Ad7172Inputs i){
	uint32_t result = 0;

	switch(i){
	case AIN0:
		result = 0;
		break;
	case AIN1:
		result = 1;
		break;
	case AIN2:
		result = 2;
		break;
	case AIN3:
		result = 3;
		break;
	case AIN4:
		result = 4;
		break;
	case TEMP_POS:
		result = 0b10001;
		break;
	case TEMP_NEG:
		result = 0b10010;
		break;
	case REF_POS:
		result = 0b10101;
		break;
	case REF_NEG:
		result = 0b10110;
		break;
	}

	return result & 0b11111;
}
Ad7172Inputs ad7172GetInput(uint32_t channel, bool posNotNeg){
	Ad7172Inputs result = 0;
	if(posNotNeg){
		result = m_adcChannels[channel].posInputType;
	} else {
		result = m_adcChannels[channel].negInputType;
	}
	return result;
}



static void setupState(AdcState s, uint32_t c){


	state = s;
	switch(state){
	case AD7172_FAIL:
		break;
	case AD7172_IDLE:
		break;
	case AD7172_COMMS:

		spiComplete = 0;
		spiQueue8(m_spiDev, m_csPin, bytesToWriteIfMode, rxBytes1, BYTES_TO_WRITE_3, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register
		spiQueue8(m_spiDev, m_csPin, bytesToReadData, rxBytes2, BYTES_TO_WRITE_5, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register

		spiQueue8(m_spiDev, m_csPin, bytesToWriteSetupCon, rxBytes3, BYTES_TO_WRITE_3, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register

		spiQueue8(m_spiDev, m_csPin, bytesToWriteFiltCon, rxBytes4, BYTES_TO_WRITE_3, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register

		configChannel(0, c, bytesToSetupChannel0);
		spiQueue8(m_spiDev, m_csPin, bytesToSetupChannel0, rxBytes5, BYTES_TO_WRITE_3, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register

		configChannel(1, c, bytesToSetupChannel1);
		spiQueue8(m_spiDev, m_csPin, bytesToSetupChannel1, rxBytes6, BYTES_TO_WRITE_3, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register

		configChannel(2, c, bytesToSetupChannel2);
		spiQueue8(m_spiDev, m_csPin, bytesToSetupChannel2, rxBytes7, BYTES_TO_WRITE_3, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register

		configChannel(3, c, bytesToSetupChannel3);
		spiQueue8(m_spiDev, m_csPin, bytesToSetupChannel3, rxBytes8, BYTES_TO_WRITE_3, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register

		spiQueue8(m_spiDev, m_csPin, bytesToWriteAdcMode, rxBytes9, BYTES_TO_WRITE_3, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register
		break;
	case AD7172_RESET:
		spiQueue8(m_spiDev, m_csPin, bytesToReset, rxBytesReset, BYTES_TO_WRITE_RESET, SPI_CS, SPI_BD, true, &spiComplete);	//setup read of data register
		break;

	case AD7172_DONE:
		break;
	}

}
static void configChannel(uint32_t j, uint32_t c, uint8_t* txBytes){
	uint32_t m = m_adcChannels[c].posInput;
	m |= m_adcChannels[c].negInput;
	uint32_t mh = m >> 8;
	uint32_t ml = m & 0xff;


	txBytes[0] = 0b00010000 | j;//compute address of channel register
	txBytes[1] = 0;//default is to turn off the channel
	txBytes[2] = 0;//ditto
	if(j == c){

		txBytes[1] = 0b10000100 | mh;//now turn on the one channel that matches currently selected channel
		txBytes[2] = 0b00000000 | ml;
	}


}

void ad7172SetGain(uint32_t channel, float gain){
	Ad7172Channel* adc = getChannel(channel);
	if(adc != NULL){
		adc->gain = gain;



	}
}
void ad7172SetOffset(uint32_t channel, float offset){
	Ad7172Channel* adc = getChannel(channel);
	if(adc != NULL){
		adc->offset = offset;
	}
}
void ad7172SetSink(uint32_t channel, SinkSource sink){
	Ad7172Channel* adc = getChannel(channel);
	if(adc != NULL){
		adc->sink = sink;
	}
}


static Ad7172Channel* getChannel(uint32_t c){
	Ad7172Channel* result = NULL;

	if(c < AD7172_NUM_CHANNELS){
		result = &m_adcChannels[c];
	}
	return result;
}

float ad7172GetGain(uint32_t channel){
	float result = NAN;
	Ad7172Channel* adc = getChannel(channel);
	if(adc != NULL){
		result = adc->gain;
	}
	return result;

}
float ad7172GetOffset(uint32_t channel){
	float result = NAN;
	Ad7172Channel* adc = getChannel(channel);
	if(adc != NULL){
		result = adc->offset;
	}
	return result;
}
SinkSource ad7172GetSink(uint32_t channel){
	SinkSource result = NULL_SINK_SOURCE;
	Ad7172Channel* adc = getChannel(channel);
	if(adc != NULL){
		result = adc->sink;
	}
	return result;
}
