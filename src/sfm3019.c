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

#include "sfm3019.h"
#include "fastcodeUtil.h"
#include "math.h"


//*************************************************
//Defines
//*************************************************

#define POLYNOMIAL 0x131
#define FLOW_SCALE (1.0f/170.0f)
#define FLOW_OFFSET (-24576.0f)
#define TEMP_SCALE (1.0f/200.0f)
#define TEMP_OFFSET (0.0f)

#define I2C_ADDRESS (0x2e<<1)


#define READ_FLOW_COMMAND 0x1000
#define READ_TEMP_COMMAND 0x1001
#define RESET_COMMAND 0x06
#define READ_SN_COMMAND 0x31ae
#define READ_O2_COMMAND 0x3603
#define READ_AIR_COMMAND 0x3608
#define READ_MIX_COMMAND 0x3632
#define READ_MIX_COMMAND 0x3632
#define CONFIG_MIX_COMMAND 0xE17D
#define RESET_ADDRESS_COMMAND 0xe000
#define STOP_COMMAND 0x3ff9
#define READ_SCALE_COMMAND 0x3661


#define CONFIG_AV_COMMAND 0x366A

//#define EXPECTED_STATUS_WORD 0x0bff
#define EXPECTED_STATUS_WORD 0x17ff




//*************************************************
//Types
//*************************************************
typedef enum {
	IDLE,
	START_SAMPLING,
	SETUP_AVERAGING,
	STOP_SAMPLING,
	WAITING_FOR_STOP,
	SETUP_MIX,
	WAITING_FOR_MIX,
	READ_FLOW,
	READ_SN,
	WAITING_FOR_START,
	WAITING_FOR_AVERAGING,
	WAITING_FOR_FLOW,
	WAITING_FOR_SN,
	WAITING_FOR_POWER_CYCLE,
	READ_SCALE,
	WAITING_FOR_SCALE,
//	FAIL,
	SW_RESETTING,
} State;

//*************************************************
//Variables
//*************************************************
static State m_state = IDLE;
static uint32_t m_timeoutTimer;
static uint32_t m_flowTimeoutTimer;
static uint32_t m_superDuperTimeoutTimer;
static uint8_t m_txBytes[10];
static uint8_t m_rxBytes[10];
static TransactionStatus m_i2cStatus;
static I2cDev m_dev;
static float m_flowSlpm;
static float m_flowIntegral = 0;
static float m_temperatureCelsius;
static uint32_t m_serialNumber;
static uint32_t m_status;
static uint32_t m_sampleTimer;
static uint32_t m_sampleCounter = 0;
static uint32_t m_timeOfLastSample = 0;
static float m_resetIntegralTime = 0.3;
static float m_resetIntegralThreshold = 0.1;
static float m_o2Mix;
static PortPin m_notResetPin;




//*************************************************
//function prototypes
//*************************************************
static uint8_t calcCrc(uint8_t data[], uint8_t numBytes);
static bool checkCrc(uint8_t data[], uint8_t numBytes);
static void computeFlow(uint8_t* bs);
static void computeTemperature(uint8_t* bs);
static void computeStatus(uint8_t* bs);
static void setState(State s);
static uint8_t getMixPercent(void);

//*************************************************
//Code
//*************************************************


static uint8_t calcCrc(uint8_t data[], uint8_t numBytes){
	uint8_t bit;// bit mask
	uint8_t crc = 0xff;// calculated checksum
	uint8_t byteCtr;// byte counter


	// calculates 8-Bit checksum with given polynomial
	for(byteCtr = 0; byteCtr < numBytes; byteCtr++){
		crc ^= (data[byteCtr]);
		for(bit =8; bit >0; --bit){
			if(crc &0x80){
				crc = (crc <<1) ^ POLYNOMIAL;
			} else {
				crc = (crc <<1);
			}
		}
	}

	return crc;
}
/**
 * This code is copied and tweaked from Sensirion app note
 * Checks the specified number of bytes against the next byte
 * if they match then great, otherwise boo.
 */
static bool checkCrc(uint8_t data[], uint8_t numBytes){
	bool result = false;

	uint8_t crc = calcCrc(data, numBytes);



	// verify checksum
	if(crc == data[numBytes]){
		result = true;
	}
	return result;
}






/**
 * Call this repeatedly to measure flow. All flow percentages seem to work.
 * @param o2Mix the O2/air mixture. 1 means 100% O2, 0 means 0% O2
 */
void sfm3019SlowCode(I2cDev dev, uint32_t microsecondsBetweenSamples, float o2Mix, PortPin notResetPin){
	m_notResetPin = notResetPin;

	m_o2Mix = o2Mix;
	m_dev = dev;

	switch(m_state){

	default:
		setState(IDLE);
		break;
	case IDLE:
		//make sure power to the sensor is on
		setPinDirection(m_notResetPin, PIN_OUT);
		setPin(m_notResetPin, true);
		resetSlowTimer(&m_timeoutTimer);
		if(slowTimer(&m_sampleTimer, microsecondsBetweenSamples)){
//			setState(READ_SCALE);
			setState(READ_FLOW);
		}
		break;
	case WAITING_FOR_POWER_CYCLE:
		//don't do anything
		//just wait patiently until another timer trips and resets us
		break;
	case WAITING_FOR_STOP:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			setState(SW_RESETTING);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK ){
			setState(START_SAMPLING);
		} else if(m_i2cStatus == TRANSACTION_TX_DATA_NACK){
			setState(START_SAMPLING);

		}
		break;
	case WAITING_FOR_START:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			setState(IDLE);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
			setState(IDLE);
		} else if(m_i2cStatus == TRANSACTION_TX_DATA_NACK){
			setState(IDLE);
		}
		break;

	case WAITING_FOR_FLOW:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			setState(SW_RESETTING);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
			//reset timers seeings as we've got data
			resetSlowTimer(&m_flowTimeoutTimer);
			resetSlowTimer(&m_superDuperTimeoutTimer);
			//now process data
			computeFlow(&m_rxBytes[0]);
			computeTemperature(&m_rxBytes[3]);
			computeStatus(&m_rxBytes[6]);//note that this will setup next state

//			setPin(GPIO_C5_PIN, false);
			++m_sampleCounter;

		} else if(m_i2cStatus == TRANSACTION_TX_DATA_NACK){
			//this case is handled by the various timeout timers. I don't think there's more to do
		}
		break;
	case WAITING_FOR_SCALE:
			if(m_i2cStatus == TRANSACTION_TIMED_OUT){
				setState(SW_RESETTING);
			} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
				//reset timers seeings as we've got data
				resetSlowTimer(&m_flowTimeoutTimer);
				resetSlowTimer(&m_superDuperTimeoutTimer);
				//now process data
				computeFlow(&m_rxBytes[0]);
				computeTemperature(&m_rxBytes[3]);
				computeStatus(&m_rxBytes[6]);//note that this will setup next state


				++m_sampleCounter;

			} else if(m_i2cStatus == TRANSACTION_TX_DATA_NACK){
				//this case is handled by the various timeout timers. I don't think there's more to do
			}

			break;

	case SW_RESETTING:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			//this means we have not seen an ack or nack which I believe is impossible
			setState(IDLE);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
			setState(IDLE);
		} else if(m_i2cStatus == TRANSACTION_TX_DATA_NACK){
			//just go back to idle and assume another timeout will fix ti
			setState(IDLE);
		}
		break;
	case WAITING_FOR_AVERAGING:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			//this means we have not seen an ack or nack which I believe is impossible
			setState(SETUP_MIX);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
			setState(SETUP_MIX);
		} else if(m_i2cStatus == TRANSACTION_TX_DATA_NACK){
			setState(SETUP_MIX);
		}
		break;
	case WAITING_FOR_MIX:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			//this means we have not seen an ack or nack which I believe is impossible
			setState(IDLE);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
			setState(IDLE);
		} else if(m_i2cStatus == TRANSACTION_TX_DATA_NACK){
			setState(IDLE);
		}
		break;
	}
	if(slowTimer(&m_superDuperTimeoutTimer, 1000000)){
		//now we're really in trouble so cycle power
		setPin(m_notResetPin, false);
		setState(WAITING_FOR_POWER_CYCLE);
	} else if(slowTimer(&m_flowTimeoutTimer, 100000)){
		//if we haven't got a flow sample for a long while then maybe we didn't start sampling
		//but first stop sampling and so-on.
		setState(STOP_SAMPLING);
	} else if(slowTimer(&m_timeoutTimer, 10000)){
		//if we haven't had comms for a while then maybe we should reset altogether
		setState(SW_RESETTING);
	}
}

/**
 * sets up the new state, including triggering i2c reads etc.
 */
static void setState(State s){
	resetSlowTimer(&m_timeoutTimer);
	switch(s){
	default:
		s = IDLE;
		break;
	case IDLE:
		break;

	case WAITING_FOR_FLOW:
		break;
	case WAITING_FOR_POWER_CYCLE:
		//don't do anything and wait for other timer to rescue us
		break;
	case SETUP_MIX:
		m_txBytes[0] = (CONFIG_MIX_COMMAND >>8);
		m_txBytes[1] = (CONFIG_MIX_COMMAND & 0xff);
		m_txBytes[2] = getMixPercent();
		m_txBytes[3] = calcCrc(&m_txBytes[2], 1);
		if(i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 4, m_rxBytes, 0, &m_i2cStatus)){
			s = WAITING_FOR_MIX;
		}
		s = IDLE;
		break;
	case SETUP_AVERAGING:
		m_txBytes[0] = (CONFIG_AV_COMMAND >>8);
		m_txBytes[1] = (CONFIG_AV_COMMAND & 0xff);
		m_txBytes[2] = 0;
		m_txBytes[3] = calcCrc(&m_txBytes[2], 1);
		if(i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 4, m_rxBytes, 0, &m_i2cStatus)){
			s = WAITING_FOR_AVERAGING;
		}
		s = IDLE;
		break;
	case SW_RESETTING:
		//this reset command is one byte long and sent to address 0.
		m_txBytes[0] = (RESET_COMMAND);

		if(i2cQueue(m_dev, 0, m_txBytes, 1, m_rxBytes, 0, &m_i2cStatus)){


			s = IDLE;
		}
		s = IDLE;
		break;
	case START_SAMPLING:

		if(m_o2Mix <= 0.0f){
			//we're measuring air
			m_txBytes[0] = (READ_AIR_COMMAND >> 8);
			m_txBytes[1] = (READ_AIR_COMMAND & 0xff);
			if(i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 2, m_rxBytes, 0, &m_i2cStatus)){
				s = WAITING_FOR_START;
			}
		} else if(m_o2Mix >= 1.0f){
			//we're measuring O2
			m_txBytes[0] = (READ_O2_COMMAND >> 8);
			m_txBytes[1] = (READ_O2_COMMAND & 0xff);
			if(i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 2, m_rxBytes, 0, &m_i2cStatus)){
				s = WAITING_FOR_START;
			}
		} else {
			//we're measuring a mix
			m_txBytes[0] = (READ_MIX_COMMAND >> 8);
			m_txBytes[1] = (READ_MIX_COMMAND & 0xff);
			m_txBytes[2]  = 0;
			m_txBytes[3] = getMixPercent();
			m_txBytes[4] = calcCrc(&m_txBytes[2], 2);
			if(i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 5, m_rxBytes, 0, &m_i2cStatus)){
				s = WAITING_FOR_START;
			}
		}


		break;
	case STOP_SAMPLING:
		m_txBytes[0] = (STOP_COMMAND >>8);
		m_txBytes[1] = (STOP_COMMAND & 0xff);

		if(i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 2, m_rxBytes, 0, &m_i2cStatus)){
			s = WAITING_FOR_STOP;
		}
		break;
	case READ_FLOW:
		m_txBytes[0] = (RESET_ADDRESS_COMMAND >>8);
		m_txBytes[1] = (RESET_ADDRESS_COMMAND & 0xff);

		if(i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 2, m_rxBytes, 9, &m_i2cStatus)){
//			setPin(GPIO_C5_PIN, true);
			s = WAITING_FOR_FLOW;
		}
		break;

	case READ_SCALE:
		m_txBytes[0] = (READ_SCALE_COMMAND >>8);
		m_txBytes[1] = (READ_SCALE_COMMAND & 0xff);
		m_txBytes[2]  = 0;
		m_txBytes[3] = getMixPercent();
		m_txBytes[4] = calcCrc(&m_txBytes[2], 2);

		if(i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 5, m_rxBytes, 9, &m_i2cStatus)){

			s = WAITING_FOR_SCALE;
		}
		break;
	}
	m_state = s;

}



static void computeFlow(uint8_t* bs){
	static float timeWithNoFlow = 0;
	if(checkCrc(bs, 2)){
		//checksum passed!
		uint32_t temp = bs[0];
		temp <<= 8;
		temp |= bs[1];

		float result = (float)temp;
		//fix sign
		if(result > 32767){
			result -= 65536;
		}
		if(temp == 0xffff){
			result = NAN;
		} else {
			result -= FLOW_OFFSET;
			result *= FLOW_SCALE;
		}
		float lastFlow = m_flowSlpm;
		m_flowSlpm = result;
		uint32_t t = getTimeInMicroSeconds();
		float deltaT = compareTimeMicroSec(t, m_timeOfLastSample);
		m_timeOfLastSample = t;
		m_flowIntegral += (lastFlow + result)*(0.5/60)*deltaT;//result in litres

		if(result < m_resetIntegralThreshold && result > -m_resetIntegralThreshold){
			timeWithNoFlow += deltaT;
		} else {
			timeWithNoFlow = 0;
		}
		if(m_resetIntegralTime > 0 && timeWithNoFlow > m_resetIntegralTime){
			m_flowIntegral = 0;
		}


	}
}
/**
 * @return false if status is wrong.
 */
static void computeStatus(uint8_t* bs){
	if(checkCrc(bs, 2)){
		uint32_t temp = bs[0];
		temp <<= 8;
		temp |= bs[1];
		m_status = temp;
		//top nibble:
		//if sampling O2 returns 0b0000
		//if air returns 0b0001
		//if mix returns 0b0110
		//bit 11 is 1 if exponential smoothing mode
		//bit 10 is 1 if average util read is active, else fixed-n avering is active
		//bits 9:0 are all 1 if pure gas
		//check the type of read
		uint32_t mix = m_status & 0x3ff;
		uint32_t command = m_status & 0xf000;

		if(m_o2Mix <= 0.0f){
			//we're measuring air
			if(command != 0x1000){
				//the sensor does not think it's running air
				setState(STOP_SAMPLING);
			} else {
				setState(IDLE);
			}
		} else if(m_o2Mix >= 1.0f){
			//we're measuring O2
			if(command != 0x0000){
				//the sensor does not think it's running air
				setState(STOP_SAMPLING);
			} else {
				setState(IDLE);
			}
		} else {

			//we're measuring a gas mix
			if(command != 0x6000){
				//the sensor's not running in the right mode so stop and start again
				setState(STOP_SAMPLING);
			} else if(mix != getMixPercent()){
				setState(SETUP_AVERAGING);
			} else {
				setState(IDLE);
			}
		//check the gas mix
		}

	}


}


static void computeTemperature(uint8_t* bs){
	if(checkCrc(bs, 2)){
		//checksum passed!
		uint32_t temp = bs[0];
		temp <<= 8;
		temp |= bs[1];

		float result = (float)temp;
		result -= TEMP_OFFSET;
		result *= TEMP_SCALE;
		m_temperatureCelsius = result;


	}
}

/**
 * @return flow in SLPM
 */
float getSfm3019Flow(void){
	return m_flowSlpm;
}
/**
 * @return the temperature in Celsius
 */
float getSfm3019Temperature(void){
	return m_temperatureCelsius;
}
/**
 * @return the serial number
 */
float getSfm3019SerialNumber(void){
	return m_serialNumber;
}
/**
 * @return the absolute number of samples successfully taken
 */
uint32_t getSfm3019SampleCount(void){
	return m_sampleCounter;
}
/**
 * @return the raw status word of the sensor
 */
uint32_t getSfm3019Status(void){
	return m_status;
}
/**
 * @return integral of flow in litres since the last the last time the flow was essentially zero.
 */
float getSfm3019Volume(void){
	return m_flowIntegral;
}
/**
 * sets flow integral to reset when no flow detected for specified time
 * if the specified time is zero then it will reset immediately this function is called and then not until it is called again
 * @param time the time that the flow magnitude must be below the threshold to trigger a reset. If zero then this is disabled but will reset during this call.
 */
void sfm3019AutoResetIntegral(float time, float threshold){
	m_resetIntegralTime = time;
	m_resetIntegralThreshold = threshold;
	if(time == 0){
		m_flowIntegral = 0;
	}
}
static uint8_t getMixPercent(void){
	float m = m_o2Mix * 100;
	if(m > 100){
		m = 100;
	} else if(m < 0){
		m = 0;
	}
	return (uint8_t)m;

}
