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

//a driver to setup and pull samples from an ads131a0x ADC
//assumes that I2SPLL is set to output 16.384MHz on MCO2 (PC9) pin
//this driver assumes the chip is configured (using the M1 pin pulled high) to communicate with a 32 bit word size.
//this driver assumes the chip is configured (using M0 pin pulled high) to be in the asynchronous interrupt mode
//this does not use hamming code for error correction (M2 pin should be GND)
//this does not use CRC for error detection
//this outputs a normalized value. To convert to volts, multiply by Vref/gain.
//default gain (which is hard-coded presently) is 1.



//*************************************************
//includes
//*************************************************
#include "ads131a0x.h"

#include <stm32f4xx_rcc.h>
#include <stm32f4xx.h>
#include <fastcodeUtil.h>
#include <queue.h>
#include <math.h>


//*************************************************
//defines
//*************************************************

//commands
#define CMD_LOCK_MSB         0x05
#define CMD_LOCK_LSB         0x55
#define CMD_NULL_MSB         0x00
#define CMD_NULL_LSB         0x00
#define CMD_RESET_MSB        0x00
#define CMD_RESET_LSB        0x11
#define CMD_STANDBY_MSB      0x00
#define CMD_STANDBY_LSB      0x22
#define CMD_UNLOCK_MSB       0x06
#define CMD_UNLOCK_LSB       0x55
#define CMD_WAKEUP_MSB       0x00
#define CMD_WAKEUP_LSB       0x33
#define CMD_RREG_MSB         0b00100000
#define CMD_RREGS_MSB        0b00100000
#define CMD_WREG_MSB         0b01000000
#define CMD_WREGS_MSB        0b01100000

//read only registers
#define REG_ID_MSB    0x00
#define REG_ID_LSB    0x01

//status registers
#define REG_STAT_1    0x02
#define REG_STAT_P    0x03
#define REG_STAT_N    0x04
#define REG_STAT_S    0x05
#define REG_ERROR_CNT 0x06
#define REG_STAT_M2   0x07
//#define REG_Reserved  0x08
//#define REG_Reserved  0x09

//user config registers
#define REG_Reserved  0x0A
#define REG_A_SYS_CFG 0x0B
#define REG_D_SYS_CFG 0x0C
#define REG_CLK1      0x0D
#define REG_CLK2      0x0E
#define REG_ADC_ENA   0x0F
//#define REG_Reserved  0x10
#define REG_ADC1      0x11
#define REG_ADC2      0x12
#define REG_ADC3      0x13 //this is only for ADS131A04
#define REG_ADC4      0x14 //this is only for ADS131A04


#define SPI_BUFFER_LENGTH 24
#define SPI_UNLOCK_WORDS_TO_SEND 4
#define SPI_CS SPI_CPOL_0_CPHA_1
//#define CS CPOL_0_CPHA_0
#define SPI_BD SPI_DIV_4

#define RESPONSE_STATUS 0
#define RESPONSE_CHANNEL0_LSB 1
#define RESPONSE_CHANNEL0_MSB 2
#define RESPONSE_CHANNEL1_LSB 3
#define RESPONSE_CHANNEL1_MSB 4
#define RESPONSE_CHANNEL2_LSB 5
#define RESPONSE_CHANNEL2_MSB 6
#define RESPONSE_CHANNEL3_LSB 7
#define RESPONSE_CHANNEL3_MSB 8

//the following values will give (assuming a 20MHz input clock
//VCO = 114.667 MHz
//phase detector frequency = 1.3333 MHz
//output freq of 16.381 MHz which is 0.0186% below desired of 16.384 MHz
#define PLL_M 15 //this is the input clock divider resulting in the phase detector frequency (which should be between 1 and 2 MHz)
#define PLL_N 86 //this is the clock multiplier
#define PLL_P 0b11 //this means divide of 8. We don't use this as it sets the SPDIF_Rx clock
#define PLL_Q 15 //this is division factor for SAI1 clock, which we don't use. Apply max divide
#define PLL_R 7
#define PLL_M_MASK 0b111111
#define PLL_N_MASK 0b111111111
#define PLL_P_MASK 0b11
#define PLL_Q_MASK 0b1111
#define PLL_R_MASK 0b111

#define PLLI2SCFGR_CONST ((PLL_M<<0) | (PLL_N<<6) | (PLL_P<<16) | (PLL_Q<<24) | (PLL_R<<28))
#define PLLI2SCFGR_MASK ((PLL_M_MASK<<0) | (PLL_N_MASK<<6) | (PLL_P_MASK<<16) | (PLL_Q_MASK<<24) | (PLL_R_MASK<<28)) //this is to avoid the reserved bits

#define SAMPLE_QUEUE_SIZE 200


//*************************************************
//Types
//*************************************************
typedef enum {
	ADS_STARTUP_STATE,
	ADS_UNLOCK_STATE,
	ADS_WAKEUP_STATE,
	ADS_CONFIG_STATE,
	ADS_LOCK_STATE,
	ADS_RUNNING_STATE,
	ADS_RESET_STATE
} State;

//*************************************************
//Variables
//*************************************************
static State m_state = ADS_STARTUP_STATE;
static bool m_enabled = false;
static PortPin m_csPin = NULL_PIN;
static PortPin m_drdyPin = NULL_PIN;
static SpiDev m_spiDev = SPINULL_DEV;
static uint8_t m_txData[SPI_BUFFER_LENGTH];
static uint8_t m_rxData[SPI_BUFFER_LENGTH];
static uint32_t m_spiComplete;
static uint32_t m_sampleTimer = 0;
static float m_sampleQueue[SAMPLE_QUEUE_SIZE][4];
static uint32_t m_qFront = 0;
static uint32_t m_qBack = 0;
static bool m_unlock = true;
static uint32_t m_stateTimeoutTimer = 0;
static uint32_t m_channelCount = 0;
static volatile uint32_t m_totalSampleCount = 0;
static volatile uint32_t m_totalSamplesLost = 0;

//*************************************************
//function prototypes
//*************************************************
static void checkAndConfigPll(void);
/**
 * This triggers an SPI transaction to get the latest value
 * For the adc to run at it's fastest this should be called in fastcode, just before the SPI fastcode driver
 * This also will chip away at chip configuration over a number of calls
 */
static void ads131a0xRequestNewValue(void);
static void processRxData(uint32_t status);
static float getData(uint32_t i);
/**
 * this will queue an appropriate SPI command and then wait for a response by queuing null commands
 */
static void queueSpi(State s);
/**
 * this will be continuously run
 * it will check for the response of the last command. If it's good or there's a timeout then a state transition will be triggered if necessary
 * @return true if current state is ok
 */
static bool manageState(void);
//*************************************************
//code
//*************************************************

/**
 * This triggers an SPI transaction to get the latest value
 * For the adc to run at it's fastest this should be called in fastcode, just before the SPI fastcode driver
 * This also will chip away at chip configuration over a number of calls
 */
static void ads131a0xRequestNewValue(void){
	checkAndConfigPll();
	if(m_enabled){








		manageState();


		//now setup spi transaction for next sample
		m_spiComplete = 0;

	}
}

void ads131a0xInit(void){
	m_enabled = true;
	checkAndConfigPll();
}

/**
 * @param enable enables this module
 * @param spi the spi port that the adc is attached to.
 * @param cs the gpio pin that the chip select is attached to
 * @param drdy the gpio pin that the data ready pin is attached to
 */
void ads131a0xConfig(bool enable, SpiDev spi, PortPin cs, PortPin drdy){
	m_enabled = enable;
	m_spiDev = spi;
	m_csPin = cs;
	m_drdyPin = drdy;
}
/**
 * queries whether there are any samples in the queue.
 */
bool isAds131a0xSampleAvailable(void){
	return isQueueNotEmpty(&m_qFront, &m_qBack, SAMPLE_QUEUE_SIZE);
}
/**
 * reads the last samples acquired.
 * Each sample consists of a number of values, depending on the channels on the ADC.
 * For now it will be 2 for the ads131a02
 * @param the channel of the sample
 */
float getAds131a0xSample(uint32_t channel){
	float result = NAN;
	if(channel < 4){
		result = m_sampleQueue[m_qFront][channel];
	}
	return result;
}


/**
 * when we're done with the last sample of channels, then discard it.
 */
void ads131a0xUnqueue(void){
	doneWithQueueFront(&m_qFront, &m_qBack, SAMPLE_QUEUE_SIZE);
}

static void checkAndConfigPll(void){
	if(m_enabled){

		//make sure I2SPLL and MCO2 pin are setup correctly
		if(
				((RCC->CR & RCC_CR_PLLI2SON) != RCC_CR_PLLI2SON) |//check control reg
				((RCC->CFGR & RCC_CFGR_MCO2PRE) != 0) | //check config reg
				((RCC->CFGR & RCC_CFGR_MCO2) != RCC_CFGR_MCO2_0) |//check more config reg
				((RCC->PLLI2SCFGR & PLLI2SCFGR_MASK) != PLLI2SCFGR_CONST)){

			//ok setup everything because something is not configured correctly
			//turn off stuff before changing settings
			//TODO: what stuff?


			//disable PLL before configuring.
			RCC->CR &= ~RCC_CR_PLLI2SON;

			//disable prescaler to MCO2 pin.
			RCC->CFGR &= ~RCC_CFGR_MCO2PRE;
			//no bits need to be set here

			//enable the MCO2 output. only change if necessary.
			RCC->CFGR &= ~RCC_CFGR_MCO2;
			RCC->CFGR |= RCC_CFGR_MCO2_0;

			RCC->PLLI2SCFGR &= ~PLLI2SCFGR_MASK;
			RCC->PLLI2SCFGR |= PLLI2SCFGR_CONST;

			//now enable PLL
			RCC->CR |= RCC_CR_PLLI2SON;

			//now enable the alternate function on the C9 pin, which is where the MCO2 pin is mapped
			setPinRawAf(GPIO_C9_PIN, 0);
		}
	} else {
		//disable PLL
		RCC->CR &= ~RCC_CR_PLLI2SON;
	}



}
/**
 * This function causes the adc to sample, at the specified sampling period.
 * It is intended to be called from SlowCode but it could be called from fast code for super fast sampling.
 * @param microseconds the period of sampling
 * @return true if it updated this time running
 */
bool ads131a0xControl(const uint32_t microseconds){
	bool result = false;
	if(slowTimer(&m_sampleTimer, microseconds)){
		ads131a0xRequestNewValue();
		result = true;
	}
	return result;
}
static void processRxData(uint32_t status){



	//should check status to see if data valid

	if(status != 0xffffffff){





		//add to sample queue if there's room
		if(isQueueNotFull(&m_qFront, &m_qBack, SAMPLE_QUEUE_SIZE)){

			m_sampleQueue[m_qFront][0] = getData(4);
			m_sampleQueue[m_qFront][1] = getData(8);
			if(m_channelCount > 2){
				m_sampleQueue[m_qFront][2] = getData(12);
				m_sampleQueue[m_qFront][3] = getData(16);
			} else {
				m_sampleQueue[m_qFront][2] = 0;
				m_sampleQueue[m_qFront][3] = 0;
			}
			++m_totalSampleCount;
			justAddedToQueueBack(&m_qFront, &m_qBack, SAMPLE_QUEUE_SIZE);
		} else {
			++m_totalSamplesLost;
		}

//	} else if(status == 0x655){
//		//we've received an unlock status response so make sure we don't request to unlock again
//		m_unlock = false;
//	} else if((status & 0xfff0) == 0xff00){//this denotes the ready status response. The bottom nibble is the number of adc channels
//		//TODO: make it send an unlock command
//		m_unlock = true;
	}
}
/**
 * construct a 32 bit number (with the 24 bits of data) where i indicates the index of the MSB
 */
static float getData(uint32_t i){
	float result = NAN;

	int32_t v = 0;

	v |= ((uint32_t)m_rxData[i + 0]) << 16;
	v |= ((uint32_t)m_rxData[i + 1]) << 8;
	v |= ((uint32_t)m_rxData[i + 2]) << 0;
	//bit extend the msb if necessary (i.e. if negative)
	if((v & 0x800000) != 0){
		v |= 0xff000000;
	}
	result = v;
	result *= 119.2092896e-9;//normalize to 1.0. This magic number is 2^-23

	return result;
}


/**
 * this will queue an appropriate SPI command and then wait for a response by queuing null commands
 */
static void queueSpi(State s){
	uint32_t byteCount = 0;
	switch(s){
	case ADS_STARTUP_STATE:
		m_txData[byteCount++] = CMD_NULL_MSB;
		m_txData[byteCount++] = CMD_NULL_LSB;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		break;
	case ADS_UNLOCK_STATE:
		//queue unlock comms
		m_txData[byteCount++] = CMD_UNLOCK_MSB;
		m_txData[byteCount++] = CMD_UNLOCK_LSB;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		break;
	case ADS_WAKEUP_STATE:
		m_txData[byteCount++] = CMD_WAKEUP_MSB;
		m_txData[byteCount++] = CMD_WAKEUP_LSB;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		break;
	case ADS_CONFIG_STATE:
		m_txData[byteCount++] = CMD_WREGS_MSB | REG_A_SYS_CFG;//start address to write to
		m_txData[byteCount++] = 9;             //write 10 registers
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;            //byte pad to 32 bits
		m_txData[byteCount++] = 0b01100000;     //REG_A_SYS_CFG: charge pump off, hi res, ref voltage 2.442, ref off, fault thresh: 5% & 95%
		m_txData[byteCount++] = 0b00111100;     //REG_D_SYS_CFG: default config
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;            //byte pad to 32 bits
		m_txData[byteCount++] = 0b10000010;     //REG_CLK1: clk source SCLK pin, ficlk = fclkin/2. Assume fclkin = 16.384MHz
		m_txData[byteCount++] = 0b00101001;     //REG_CLK2: fmod = ficlk /2, fdata = fmod/200. This gives fmod of 4.096MHz and fdata of 20.48kHz
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;            //byte pad to 32 bits
		m_txData[byteCount++] = 0b00001111;     //REG_ADC_ENA: power up all channels
		m_txData[byteCount++] = 0;              //REG_Reserved: don't set anything, this byte is reserved
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;            //byte pad to 32 bits
		m_txData[byteCount++] = 0b00000000;     //REG_ADC1: gain = 1
		m_txData[byteCount++] = 0b00000000;     //REG_ADC2: gain = 1
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;            //byte pad to 32 bits
		m_txData[byteCount++] = 0b00000000;     //REG_ADC3: gain = 1
		m_txData[byteCount++] = 0b00000000;     //REG_ADC4: gain = 1
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;            //byte pad to 32 bits
		break;
	case ADS_LOCK_STATE:
		m_txData[byteCount++] = CMD_LOCK_MSB;
		m_txData[byteCount++] = CMD_LOCK_LSB;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		break;
	case ADS_RESET_STATE:
		m_txData[byteCount++] = CMD_RESET_MSB;
		m_txData[byteCount++] = CMD_RESET_LSB;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		break;
	case ADS_RUNNING_STATE:
		//queue normal commms
		m_txData[byteCount++] = CMD_NULL_MSB;
		m_txData[byteCount++] = CMD_NULL_LSB;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;

		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;

		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;

		if(m_channelCount > 2){

			m_txData[byteCount++] = 0;
			m_txData[byteCount++] = 0;
			m_txData[byteCount++] = 0;
			m_txData[byteCount++] = 0;

			m_txData[byteCount++] = 0;
			m_txData[byteCount++] = 0;
			m_txData[byteCount++] = 0;
			m_txData[byteCount++] = 0;
		}
		//that should be enough clocks to get all data


		break;
	}
	if(byteCount >= SPI_BUFFER_LENGTH){
		byteCount = SPI_BUFFER_LENGTH - 1;
	}
	spiQueue8(m_spiDev, m_csPin, m_txData,  m_rxData, byteCount, SPI_CS, SPI_BD, true, &m_spiComplete);

}
/**
 * this will be continuously run
 * it will check for the response of the last command. If it's good or there's a timeout then a state transition will be triggered if necessary
* @return true if current state is ok
 */
static bool manageState(void){


	uint32_t status = 0xffffffff;
	//first check result of last spi transaction
	if(m_spiComplete > 0){
		//first check status work

		status = ((uint16_t)m_rxData[0]) << 8;
		status |= (uint16_t)m_rxData[1];
	}


	bool result = true;
	switch(m_state){
	case ADS_STARTUP_STATE:
		if((status & 0xfff0) == 0xff00){
			m_channelCount = status & 0xf;
			m_state = ADS_UNLOCK_STATE;
			resetSlowTimer(&m_stateTimeoutTimer);
			queueSpi(m_state);
		} else {
			queueSpi(m_state);
		}
		break;
	case ADS_UNLOCK_STATE:
		if(status == 0x655){
			m_state = ADS_CONFIG_STATE;
			resetSlowTimer(&m_stateTimeoutTimer);
			queueSpi(m_state);
		} else {
			queueSpi(m_state);
		}
		break;
	case ADS_WAKEUP_STATE:
		if(status == 0x0033){
			m_state = ADS_LOCK_STATE;
			resetSlowTimer(&m_stateTimeoutTimer);
			queueSpi(m_state);
		} else {
			queueSpi(m_state);
		}
		break;
	case ADS_CONFIG_STATE:
		if((status & 0xff00) == 0x4b00){
			m_state = ADS_WAKEUP_STATE;
			resetSlowTimer(&m_stateTimeoutTimer);
			queueSpi(m_state);
		} else {
			queueSpi(m_state);
		}
		break;
	case ADS_LOCK_STATE:
		if(status == 0x0555){
			m_state = ADS_RUNNING_STATE;
			resetSlowTimer(&m_stateTimeoutTimer);
			queueSpi(m_state);
		} else {
			queueSpi(m_state);
		}
		break;
	case ADS_RESET_STATE:
		m_state = ADS_STARTUP_STATE;
		queueSpi(m_state);
		break;
	case ADS_RUNNING_STATE:
		processRxData(status);
		resetSlowTimer(&m_stateTimeoutTimer);
		//Only queue a read if the dataready pin is asserted. Note: negative logic.
		if(!isPinInputSet(m_drdyPin)){
			queueSpi(m_state);
			resetSlowTimer(&m_stateTimeoutTimer);
		} else if(slowTimer(&m_stateTimeoutTimer, 1000)) {
			m_state = ADS_UNLOCK_STATE;
			queueSpi(m_state);
		}

		break;
	}
	return result;
}
