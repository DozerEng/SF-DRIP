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
/**
 * a driver for the ADS131m0x Sigma-Delta ADC
 * see https://www.ti.com/lit/ds/symlink/ads131m08.pdf for details of the 8 channel part
 * see https://www.ti.com/lit/ds/symlink/ads131m04.pdf for details of the 4 channel part
 */
//Note: if SPI issues are encountered, such as the last bit not reading (Rx) as the correct polarity, increase the configured SCLK pin speed from 2MHz to 25MHz or 50MHz.
//*************************************************
//notes
//*************************************************
//TODO: CM: Add received data CRC from ADC SPI transaction
//TODO: CM: Enable Incoming Message CRC to ADC (ADC needs to accept message based on CRC

//A driver to setup and transfer samples from an ads131m0x ADC
//Assumes internal 1.2V reference is used.  Note: IC can accept +/-1.2V.  If using 0V to 1.2V input, usable range is essentially 2^23b
//assumes an external ADC Xtal (clock) at 2.048MHz, low power mode

//Sampling rate of ADC and polling rate of the micro is asynchronous.

//This module is currently set up such that the ADS131M08 is clocked via the crystal, and maintains sample rate timing.  This module polls for a logic low signal on the drdy pin, indicating
//there is new data to transfer from the ADC.  ads131m0xSlowCode() should be called at a rate minimum fsample*4, example: (250Hz)*4 = 1kHz, or 1ms intervals.  Practically, slow code usually runs at
// >~5kHz, so a call to ads131m0xSlowCode() should be added to either fast code or slow code to ensure reliable sampling.

//this does not use CRC for error detection
//this outputs a value in volts, with a gain of 1 and a Vref of 1.2V
//default gain (which is hard-coded presently) is 1.

//
//m_LastReceivedSample



//To set up, a chip select pin needs to be configured as an output, a data ready pin as an input, and the relevant SPI bus.
//
//Example set up in main:
//	static uint8_t ads131Drdy = GPIO_A8_PIN;
//	static uint8_t ads131Cs = GPIO_A15_PIN;
//	setPinDirection(ads131Cs, PIN_OUT);  //CS
//	setPinDirection(ads131Drdy, PIN_IN); //DRDY
//	setPin(ads131Cs, true);
//	ads131m0xConfig(true, SPI2_DEV, ads131Cs, ads131Drdy, SR_250HZ);// 1us/sample ensures the module is constantly checking for a low drdy pin.
//                                                           //The module may be alternately configured to be the sampling master and the '1' becomes something more like 4000us, for ~250Hz.
//	ads131m0xInit();

//Example data consumption by application:
// 1)The simple, Lazy way, constantly updating appData with the newest sample.  Note, the sample queue is constantly overrun when used like this
//			setTxAppData(2, getads131m0xLastReceivedSample(0));
//			setTxAppData(3, getads131m0xLastReceivedSample(1));
//			setTxAppData(4, getads131m0xLastReceivedSample(2));
//			setTxAppData(5, getads131m0xLastReceivedSample(3));
//			setTxAppData(6, getads131m0xLastReceivedSample(4));
//			setTxAppData(7, getads131m0xLastReceivedSample(5));
//			setTxAppData(8, getads131m0xLastReceivedSample(6));
//			setTxAppData(9, getads131m0xLastReceivedSample(7));
//
// 2) The queue to queue transfer method.  Transfer Q from module to streaming manager example.  Other ways of doing this...
// 		This method offers buffering and reliable consumption of data.
//		Call the below function at min 4x sample rate:
//		ads131m0xSlowCode();
//
//		//assume this is called at much higher rate than the sample rate of the module
//		if(isads131m0xSampleAvailable()){
//			for(uint32_t i = 0; i < ads131m0xGetNumChannels(); i++ ){
//				addToStreamNow(i, getads131m0xSample(i));
//			}
//			ads131m0xUnqueue(); //remove transferred sample from front of queue
//		}
//
// 3) Free-running transfer method
//		Periodically call getads131m0xLastReceivedSample(X) at a rate min ~4x slower than sample frequency

//*************************************************
//includes
//*************************************************

#include <ads131m0x.h>
#include <core_cmInstr.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx.h>
#include <fastcodeUtil.h>
#include <math.h>
#include "crc1021.h"

//*************************************************
//defines
//*************************************************
#define CONVERT_INT_TO_VOLTS (1.2f / ((float)(1 << 31))) //this will convert the raw integer reading to volts
#define SPI_TRANSACTION_TIMEOUT 2000 //us

#define ADC_OFFSET 0 //TODO: what should this be?
#define ADC_SCALE 1 //TODO: what should this be?


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
#define CMD_WREGS_RESP_MSB   0b01000000


#define STATUS_LOCK_BIT      0x8000 //bit 15 in Status register
#define STATUS_RESET_BIT     0x0400 //bit 10

//read only registers
//#define REG_ID_MSB    0x00
//#define REG_ID_LSB    0x01

//status registers
//#define REG_STAT_1    0x02
//#define REG_STAT_P    0x03
//#define REG_STAT_N    0x04
//#define REG_STAT_S    0x05
//#define REG_ERROR_CNT 0x06
//#define REG_STAT_M2   0x07


//user config registers
//#define REG_MODE      0x02
#define REG_CLOCK     0x03
//#define REG_Reserved  0x0A
//#define REG_A_SYS_CFG 0x0B
//#define REG_D_SYS_CFG 0x0C
//#define REG_CLK1      0x0D
//#define REG_CLK2      0x0E
//#define REG_ADC_ENA   0x0F
//#define REG_Reserved  0x10
//#define REG_ADC1      0x11
//#define REG_ADC2      0x12

#define CRC_BYTE_LENGTH 3
#define SPI_BUFFER_LENGTH 32 //3 bytes command, 8*3 bytes data, 3 bytes CRC
#define SPI_CS SPI_CPOL_0_CPHA_1
#define SPI_BD SPI_DIV_256 //256 runs the bus at 350kHz bit time.  Decrease to 64, 32, 16 to run faster. Lowered for MR headset (over cable).

#define MAX_NUM_CHANNELS 8
#define SAMPLE_QUEUE_SIZE 32
#define NUM_SPI_RETRIES 10


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

typedef struct {
	SinkSource sink;
	float gain;
	float offset;
} ChannelConfig;

//*************************************************
//Variables
//*************************************************
static State m_state = ADS_STARTUP_STATE;
static PortPin m_csPin = NULL_PIN;
static PortPin m_drdyPin = NULL_PIN;
static SpiDev m_spiDev = SPINULL_DEV;//if this is null then the part is disabled
static uint8_t m_txData[SPI_BUFFER_LENGTH];
static uint8_t m_rxData[SPI_BUFFER_LENGTH] = {0x01,
		0x02,
		0x03,
		0x04,
		0x05,
		0x06,
		0x07,
		0x08,
		0x09,
		0x0a,
		0x0b,
		0x0c,
		0x0d,
		0x0e,
		0x0f,
		0x10,
		0x11,
		0x12,
		0x13,
		0x14,
		0x15,
		0x16,
		0x17,
		0x18,
		0x19,
		0x1a,
		0x1b,
		0x1c,
		0x1d,
		0x1e,
		0x1f
};
static ChannelConfig m_channelConfigs[MAX_NUM_CHANNELS];
//static SinkSource m_channelSinks[MAX_NUM_CHANNELS];
static uint8_t m_statusDesired = 0xff;//this is the value that the bottom byte of the status reg should be when a conversion is done.

static uint32_t m_spiComplete;
static uint32_t m_stateTimeoutTimer = 0;
static uint32_t m_pollingPeriodTimer = 0;
static uint32_t m_spiTransactionTimeout = 0;
static uint32_t m_channelCount = MAX_NUM_CHANNELS;
static volatile uint32_t m_totalSamplesLost = 0;
static uint32_t m_ADS131Sr = SR_62_5HZ; //default to slowest, 62.5Hz
static uint32_t m_ADS131ReqSr = SR_62_5HZ;
static uint32_t m_ADS131PollPeriodUs = 0; // default to zero
static bool m_waitingForMessageResponse = false;
static uint32_t m_spiTimeoutCommsCount = 0;
static uint32_t retries = 0;
static uint32_t byteCount = 0;
static volatile uint32_t test = 0;

static bool m_runInFastCode = false;




//static bool newSampleAvailable = false;
//static uint32_t m_totalLargeSampleGap = 0;
//static uint32_t m_samplingRateTimer = 0;
//static uint32_t m_runningTotalSamplesPerSecond = 0;
//static uint32_t m_numSamplesPerSecond =0;
//static uint32_t m_pollingRateTimer = 0;
//static uint32_t m_numPollsPerSecond = 0;
//static uint32_t m_runningTotalPollsPerSecond = 0;
//static uint32_t totalNumSamplesCaptured = 0;
//static uint32_t maxSampleQueueUtilization = 0;

//*************************************************
//function prototypes
//*************************************************
/**
 * This triggers an SPI transaction to get the latest value
 * For the adc to run at it's fastest this should be called in fastcode, just before the SPI fastcode driver
 * This also will chip away at chip configuration over a number of calls
 */
static void processRxData(uint32_t status);

/**
 * this will queue an appropriate SPI command and then wait for a response by queuing null commands
 */
static void queueSpi(void);
/**
 * this will be continuously run
 * it will check for the response of the last command. If it's good or there's a timeout then a state transition will be triggered if necessary
 * @return true if current state is ok
 */
static bool manageState(void);

/**
 * sets up a new state
 */
static void setState(State s);
/**
 * setup the next SPI transaction
 */
static void setupSpi(State s);

static void querySpi(void);
/**
 * swap bytes in a word
 *
 */
static inline uint32_t swap32(uint8_t* in);
//*************************************************
//code
//*************************************************













void ads131m0xInit(void){
//	test = swap32(m_rxData);
	for(uint8_t i = 0; i < SPI_BUFFER_LENGTH; i++){
		m_txData[i] = 0;
		m_rxData[i] = 0;
	}
	for(uint32_t i = 0; i < MAX_NUM_CHANNELS; ++i){
		ChannelConfig* cf = &m_channelConfigs[i];
		cf->gain = 1.0f;
		cf->offset = 0.0f;
		cf->sink = NULL_SINK_SOURCE;
	}
}



/**
 * Configures the module.
 * If no drdy pin is specified then the ADS131 is polled for valid data.  The module throws out data if status bit indicate all channels are not DataReady.
 * @param spi the spi port that the adc is attached to. Part is disabled if set to SPINULL_DEV
 * @param cs the gpio pin that the chip select is attached to
 * @param drdy the gpio pin that the data ready pin is attached to.  If using polling approach, set to NULL_PIN
 * @param sampleFrequency encoded in type ADS131M0xSampleRate, the ADS131M0x is the sampling/timing master
 */
void ads131m0xConfig(SpiDev spi, ADS131M0xSampleRate sampleFrequency, uint32_t numChannels, PortPin cs, PortPin drdy){
	m_spiDev = spi;
	m_csPin = cs;
	m_drdyPin = drdy;
	//m_samplePeriodUs = 1;
	m_ADS131ReqSr = sampleFrequency;
	if(drdy == NULL_PIN){
		//if we're not configured to check the data ready pin then set a polling period about one third the sample period
		switch(sampleFrequency){
		default:
		case SR_8000HZ:
			m_ADS131PollPeriodUs = 40;
			break;
		case SR_4000HZ:
			m_ADS131PollPeriodUs = 80;
			break;
		case SR_2000HZ:
			m_ADS131PollPeriodUs = 160;
			break;
		case SR_1000HZ:
			m_ADS131PollPeriodUs = 320;
			break;
		case SR_500HZ:
			m_ADS131PollPeriodUs = 640;
			break;
		case SR_250HZ:
			m_ADS131PollPeriodUs = 1280;
			break;
		case SR_125HZ:
			m_ADS131PollPeriodUs = 2560;
			break;
		case SR_62_5HZ:
			m_ADS131PollPeriodUs = 5120;
			break;
		}
	}
	switch(numChannels){
	case 1:
		m_channelCount = 1;
		m_statusDesired = 0b1;
		break;
	case 2:
		m_channelCount = 2;
		m_statusDesired = 0b11;

		break;
	case 3:
	case 4:
		m_channelCount = 4;
		m_statusDesired = 0b1111;

		break;
	case 5:
	case 6:
		m_channelCount = 6;
		m_statusDesired = 0b111111;

		break;
	default:
	case 7:
	case 8:
		m_channelCount = 8;
		m_statusDesired = 0b11111111;

		break;

	}

}



SpiDev getAds131m0xSpi(void){
	return m_spiDev;
}
ADS131M0xSampleRate getAds131m0xSampleRate(void){
	return m_ADS131ReqSr;
}
uint32_t getAds131m0xNumChannels(void){
	return m_channelCount;
}
PortPin getAds131m0xCsPin(void){
	return m_csPin;
}
PortPin getAds131m0xDrdyPin(void){
	return m_drdyPin;
}








/**
 * This function causes the adc to sample, at the specified sampling period.
 * It is intended to be called from SlowCode
 * @param microseconds the period of sampling
 * @return true if it updated this time running
 */
bool ads131m0xSlowCode(void){
	bool result = false;
	bool runInSlowCode = false;
	if(m_spiDev != SPINULL_DEV){
		manageState();
		result = true;
	}

	//if we haven't seen a response for a while then force state machine to restart
	if(slowTimer(&m_stateTimeoutTimer, 1000000)) {
		setState(ADS_UNLOCK_STATE);
	} else{
		//m_sampleTimer = 0; //sample is missed due to the ADC not ready.  Set sample timer so the sample period is nulled and the module will try again on next call from slow code.
		//result = false;
	}
	if(m_state == ADS_RUNNING_STATE){
		switch(m_ADS131ReqSr){
		case SR_8000HZ: //OSR 128
		case SR_4000HZ: //OSR 256
		case SR_2000HZ: //OSR 512
		case SR_1000HZ: //OSR 1024
			m_runInFastCode = true;
			runInSlowCode = false;
			break;

		case SR_500HZ: //OSR 2048
		case SR_250HZ: //OSR 4096
		case SR_125HZ: //OSR 8192
		case SR_62_5HZ: //OSR 16256
			m_runInFastCode = false;
			runInSlowCode = true;
			break;
		}
	}

	//if we're not needing to run quickly then run sampling here
	if(runInSlowCode){
		querySpi();
	}
	return result;
}
/**
 * handles the really fast part of getting adc data
 * This only does anything if the adc state is in the ADC_RUNNING_STATE & the sample rate is 1kHz or faster
 */
void ads131m0xFastCode(void){
	if(m_runInFastCode){
		querySpi();
	}
}
/**
 * handles the really fast part of getting adc data
 */
static void querySpi(void){
	uint32_t status = 0;





	//first check result of last spi transaction
	if(m_spiComplete > 0){
		m_spiComplete = 0;
		m_waitingForMessageResponse = false;
		status = swap32(m_rxData) >> 16;
//		status = ((uint16_t)m_rxData[0]) << 8;
//		status |= (uint16_t)m_rxData[1]


		if((m_statusDesired & status) == m_statusDesired){// dataready on all channels
			processRxData(status);
			//resetSlowTimer(&m_pollingPeriodTimer); //synchronize valid data with sampling rate
			resetSlowTimer(&m_stateTimeoutTimer); //reset timer to avoid timeout when receiving valid data
			retries = 0;
		}
	}
	//Only queue if the data ready pin is asserted or if that's not configured then when the polling interval is over
	if(!m_waitingForMessageResponse){
		if(m_drdyPin != NULL_PIN){ //only if using dataReady pin method
			if(!isPinInputSet(m_drdyPin)){
				queueSpi();
			}
		}else if (slowTimer(&m_pollingPeriodTimer, m_ADS131PollPeriodUs)){ // otherwise, use polling method. Send another poll.
			queueSpi();
		}
	}
}


/**
 * checks the received data for status and crc. Then grabs and publishes the adc values.
 */
static void processRxData(uint32_t status){
//	static uint32_t lastSampleTime = 0;
//	static uint32_t sampleCounter = 0;
	static uint16_t crcAcc = 0xffff;
//	static int32_t  sampleTime_ms = 0;
	static uint16_t ads131RxCrc  = 0;
	static uint32_t i = 0;
	//should check status to see if data valid
	float value;

	if(status != 0xffffffff){
		//Calculate crc from ADC SPI packet
		if(byteCount - CRC_BYTE_LENGTH <= SPI_BUFFER_LENGTH){ //use bytecount to determine response size, and ensure we're not looping out-of-bounds of m_rxData
//			crcAcc = 0xffff;
//			for(i=0; i < byteCount - CRC_BYTE_LENGTH; i++){
//				crcAcc = crc1021(crcAcc, m_rxData[i]);
//			}
			crcAcc = crc1021Loop(0xffff, &m_rxData[i], byteCount - CRC_BYTE_LENGTH);
		}

		ads131RxCrc = ((uint32_t)m_rxData[byteCount - CRC_BYTE_LENGTH ] << 8) | (uint16_t)m_rxData[byteCount - CRC_BYTE_LENGTH + 1];
		if(crcAcc == ads131RxCrc){
			for( i = 0; i < m_channelCount; i++ ){
				ChannelConfig *cf = &(m_channelConfigs[i]);

				if(cf->sink != NULL_SINK_SOURCE){
					value = (float)(swap32(&m_rxData[3+i*3]) >> 8);
					value  *= CONVERT_INT_TO_VOLTS;
					value *= cf->gain;
					value += cf->offset;
					setSinkSource(cf->sink, value);
				}
			}

		}else{
			++m_totalSamplesLost;
		}
	}
}




/**
 * this will queue an appropriate SPI command and then wait for a response by queuing null commands
 */
static void setupSpi(State s){
	byteCount = 0;
	retries++;
	switch(s){
	case ADS_STARTUP_STATE:
		m_txData[byteCount++] = CMD_NULL_MSB;
		m_txData[byteCount++] = CMD_NULL_LSB;
		m_txData[byteCount++] = 0;           //pad to 24b
		break;
	case ADS_UNLOCK_STATE:
		//queue unlock comms
		m_txData[byteCount++] = CMD_UNLOCK_MSB;
		m_txData[byteCount++] = CMD_UNLOCK_LSB;
		m_txData[byteCount++] = 0;           //pad to 24b
		break;
	case ADS_WAKEUP_STATE:
		m_txData[byteCount++] = CMD_WAKEUP_MSB;
		m_txData[byteCount++] = CMD_WAKEUP_LSB;
		m_txData[byteCount++] = 0;           //pad to 24b
		break;
	case ADS_CONFIG_STATE:
		m_txData[byteCount++] = CMD_WREGS_MSB | (REG_CLOCK >> 1); //start address to write to = REG_CLOCK
		m_txData[byteCount++] = (0 |((uint16_t)REG_CLOCK << 7)) & 0x00ff; //write 1 registers, n registers - 1.  cast and truncation to avoid warning.
		m_txData[byteCount++] = 0;			 //pad to 24b

		//0x02 Mode register = defaults
		m_txData[byteCount++] = 0b11111111;     //Addr. 0x03 MSB: Clock MSB: All channels enabled
		m_txData[byteCount++] = 0b00000000 | (m_ADS131ReqSr << 2);     //LSB: Xtal osc enable, external ref disable,Very low Power Mode, 4096 oversampling rate, output datarate 250sps
		m_txData[byteCount++] = 0b00000000;		//pad for 24b
		//the rest are defaults
		/*m_txData[byteCount++] = 0b00000000;     //Address 0x04 GAIN1 MSB: defaults, channels 0 to 3 All gain = 1
		m_txData[byteCount++] = 0b00000000;     //LSB
		m_txData[byteCount++] = 0;			    //pad for 24b

		m_txData[byteCount++] = 0b00000000;     //Address 0x05 GAIN2 MSB: defaults, channels 4 to 7 All gain = 1
		m_txData[byteCount++] = 0b00000000;     //LSB
		m_txData[byteCount++] = 0;			    //pad for 24b

		m_txData[byteCount++] = 0b00000110;     //Address 0x06 CFG MSB: defaults, global-chop delay = 16, but disabled.
		m_txData[byteCount++] = 0b00000000;     //LSB
		m_txData[byteCount++] = 0;			    //pad for 24b

		m_txData[byteCount++] = 0b00000000;     //Address 0x07 Threshold_MSB MSB: defaults, zeros
		m_txData[byteCount++] = 0b00000000;     //LSB
		m_txData[byteCount++] = 0;			    //pad for 24b

		m_txData[byteCount++] = 0b00000000;     //Address 0x08 Threshold_LSB MSB: defaults, zeros
		m_txData[byteCount++] = 0b00000000;     //LSB
		m_txData[byteCount++] = 0;			    //pad for 24b*/

		//... see datasheet for registers, including offset correction, etc.

		break;
	case ADS_LOCK_STATE:
		m_txData[byteCount++] = CMD_LOCK_MSB;
		m_txData[byteCount++] = CMD_LOCK_LSB;
		m_txData[byteCount++] = 0;
		break;

	case ADS_RESET_STATE:
		m_txData[byteCount++] = CMD_RESET_MSB;
		m_txData[byteCount++] = CMD_RESET_LSB;
		m_txData[byteCount++] = 0;
		break;

	case ADS_RUNNING_STATE:
		//queue normal commms
		m_txData[byteCount++] = CMD_NULL_MSB;  //This word shifts out the status word
		m_txData[byteCount++] = CMD_NULL_LSB;
		m_txData[byteCount++] = 0; //24b pad

		for(uint8_t i = 0; i < m_channelCount; i++){ //channels (8 nominally) * 24b
			m_txData[byteCount++] = 0;
			m_txData[byteCount++] = 0;
			m_txData[byteCount++] = 0;
		}
		//that should be enough clocks to get all data
		m_txData[byteCount++] = 0; //crc
		m_txData[byteCount++] = 0;
		m_txData[byteCount++] = 0;
		break;

	}
	//the following test can never be true. Worst case is reading 8 values -> 30 bytes
	if(byteCount >= SPI_BUFFER_LENGTH){
		byteCount = SPI_BUFFER_LENGTH - 1;
	}

}
/**
 * setup the next SPI transaction
 */
static void queueSpi(void){
	spiQueue8(m_spiDev, m_csPin, m_txData,  m_rxData, byteCount, SPI_CS, SPI_BD, true, &m_spiComplete);
	resetSlowTimer(&m_spiTransactionTimeout);
	m_waitingForMessageResponse = true;
}

/**
 * this will be continuously run
 * it will check for the response of the last command. If it's good or there's a timeout then a state transition will be triggered if necessary
* @return true if current state is ok
 */
static bool manageState(void){
	bool result = true;
	uint32_t status;

	status = swap32(m_rxData) >> 16;
//	status = ((uint16_t)m_rxData[0]) << 8;
//	status |= (uint16_t)m_rxData[1]

	//first check result of last spi transaction
	if(m_state != ADS_RUNNING_STATE && m_waitingForMessageResponse){
		if(m_spiComplete > 0){
 			m_spiComplete = 0;
			//first check status work


			m_waitingForMessageResponse = false;
		} else if(slowTimer(&m_spiTransactionTimeout,SPI_TRANSACTION_TIMEOUT)){ //this should never really be hit if everything is connected and the module is clocked
			m_waitingForMessageResponse = false;
			++m_spiTimeoutCommsCount;
		}
	}

	if(m_ADS131ReqSr != m_ADS131Sr){ //if there is a different requested sample rate, program the ADC as such
		setState(ADS_CONFIG_STATE);
	}else if(retries > NUM_SPI_RETRIES){

		switch(m_state){
		case ADS_STARTUP_STATE:
			if((status & 0xff00) == 0xff00){ //simply look for a non-zero response.  CM TODO: Decode status and act differentially, if needed.
				if(status & STATUS_LOCK_BIT){
					setState(ADS_UNLOCK_STATE);
				}else{
					setState(ADS_CONFIG_STATE);
				}

			}
			queueSpi();
			break;
		case ADS_UNLOCK_STATE:
			if(status == 0x655){
				retries = 0;
				setState(ADS_CONFIG_STATE);
			}
			queueSpi();
			break;
		case ADS_WAKEUP_STATE:
			if(status == 0x33){
				setState(ADS_LOCK_STATE);
				retries = 0;
			}
			queueSpi();
			break;
		case ADS_CONFIG_STATE:
			if((status & 0xff80) == ((uint16_t)CMD_WREGS_RESP_MSB << 8|(uint16_t)REG_CLOCK << 7)){
				retries = 0;
				setState(ADS_WAKEUP_STATE);
				m_ADS131Sr = m_ADS131ReqSr;
			}
			queueSpi();
			break;
		case ADS_LOCK_STATE:
			if(status == 0x0555){
				retries = 0;
				setState(ADS_RUNNING_STATE);
			}
			queueSpi();
			break;
		case ADS_RESET_STATE:
			retries = 0;
			setState(ADS_STARTUP_STATE);
			queueSpi();
			break;
		case ADS_RUNNING_STATE:
			//The processing and setup of receiving new samples is handled in fast code
			break;
		}
	}
	return result;
}
/**
 * set the sink for the specified channel
 */
void setAds131m0xSink(uint32_t channel, SinkSource sink){
	//TODO: should we check to see if the specified sink - if it's a stream - has the same sample period as this moudule?
	if(channel < MAX_NUM_CHANNELS){
		m_channelConfigs[channel].sink = sink;
	}
}

SinkSource getAds131m0xSink(uint32_t channel){
	SinkSource result = NULL_SINK_SOURCE;
	if(channel < MAX_NUM_CHANNELS){
		result = m_channelConfigs[channel].sink;
	}
	return result;
}

float getAds131m0xGain(uint32_t channel){
	float result = NAN;
	if(channel < MAX_NUM_CHANNELS){
		result = m_channelConfigs[channel].gain;
	}
	return result;
}
float getAds131m0xOffset(uint32_t channel){
	float result = NAN;
	if(channel < MAX_NUM_CHANNELS){
		result = m_channelConfigs[channel].offset;
	}
	return result;
}
void setAds131m0xGain(uint32_t channel, float gain){
	if(channel < MAX_NUM_CHANNELS){
		m_channelConfigs[channel].gain = gain;
	}
}
void setAds131m0xOffset(uint32_t channel, float offset){
	if(channel < MAX_NUM_CHANNELS){
		m_channelConfigs[channel].offset = offset;
	}
}
/**
 * applies the specified adc reading to cause the ADC to now measure zero at this point
 */
void zeroAds131m0x(uint32_t channel, float currentValue){
	if(channel < MAX_NUM_CHANNELS){
		m_channelConfigs[channel].offset -= currentValue;
	}
}
/**
 * convenience function to set a bunch of params at the same time
 * @param channel the channel to configure
 * @param sink the sink to place the output result
 * @param gain the scale to apply to this channel
 * @param offset the offset to apply to this channel
 */
void setAds131m0xParams(uint32_t channel, SinkSource sink, float gain, float offset){
	if(channel < MAX_NUM_CHANNELS){
		m_channelConfigs[channel].sink = sink;
		m_channelConfigs[channel].gain = gain;
		m_channelConfigs[channel].offset = offset;
	}

}
/**
 * sets up a new state
 */
static void setState(State s){
	m_state = s;
	setupSpi(s);
	resetSlowTimer(&m_stateTimeoutTimer);
}
/**
 * swap bytes of a 32 bit word. This to fix the endian-ness of the received dat.
 */
static inline uint32_t swap32(uint8_t* in){
	uint32_t result = __REV(*(uint32_t*)in);
	return result;
}
