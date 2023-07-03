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

#include "sfm3000.h"
#include "fastcodeUtil.h"
#include "math.h"

//*************************************************
//Defines
//*************************************************

#define POLYNOMIAL 0x131
#define FLOW_SCALE (1.0f/142.8f)
#define FLOW_OFFSET (32000.0f)
#define TEMP_SCALE (1.0f/1000.0f)
#define TEMP_OFFSET (0.0f)


#define I2C_ADDRESS (0x40<<1)


#define READ_FLOW_COMMAND 0x1000
#define READ_TEMP_COMMAND 0x1001
#define RESET_COMMAND 0x2000
#define READ_SN_COMMAND 0x31ae



//*************************************************
//Types
//*************************************************
typedef enum {
	IDLE,
	READ_TEMPERATURE,
	READ_FLOW,
	READ_SN,
	WAITING_FOR_TEMPERATURE,
	WAITING_FOR_FLOW,
	WAITING_FOR_SN,
	FAIL,
	SW_RESETTING,
} State;

//*************************************************
//Variables
//*************************************************
static State m_state = IDLE;
static uint32_t m_timeoutTimer;
static uint8_t m_txBytes[10];
static uint8_t m_rxBytes[10];
static TransactionStatus m_i2cStatus;
static I2cDev m_dev;
static float m_flowSlpm;
static float m_temperatureCelsius;
static uint32_t m_serialNumber;
static uint32_t m_sampleTimer;
static uint32_t m_sampleCounter = 0;



//*************************************************
//function prototypes
//*************************************************

static bool checkCrc(uint8_t data[], uint8_t nbrOfBytes);

static bool setupI2cRead(uint16_t command, uint8_t bytesToRead);
static void computeFlow(void);
static void computeTemperature(void);
static void computeSerialNumber(void);
static void setState(State s);

//*************************************************
//Code
//*************************************************

/**
 * This code is copied and tweaked from Sensirion app note
 * Checks the specified number of bytes against the next byte
 * if they match then great, otherwise boo.
 */
static bool checkCrc(uint8_t data[], uint8_t nbrOfBytes){
	bool result = false;
	uint8_t bit;// bit mask
	uint8_t crc = 0;// calculated checksum
	uint8_t byteCtr;// byte counter


	// calculates 8-Bit checksum with given polynomial
	for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++){
		crc ^= (data[byteCtr]);
		for(bit =8; bit >0; --bit){
			if(crc &0x80){
				crc = (crc <<1) ^ POLYNOMIAL;
			} else {
				crc = (crc <<1);
			}
		}
	}
	// verify checksum
	if(crc == data[nbrOfBytes]){
		result = true;
	}
	return result;
}






void sfm3000SlowCode(I2cDev dev, uint32_t microsecondsBetweenSamples){
	m_dev = dev;
	switch(m_state){

	default:
		setState(IDLE);
		break;
	case IDLE:
		resetSlowTimer(&m_timeoutTimer);
		if(slowTimer(&m_sampleTimer, microsecondsBetweenSamples)){
			setState(READ_FLOW);
		}
		break;
	case WAITING_FOR_TEMPERATURE:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			setState(SW_RESETTING);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
			//grab new measurement
			computeTemperature();
			setState(IDLE);				//TODO do something more clever than just going back to idle

		}
		break;
	case WAITING_FOR_SN:
			if(m_i2cStatus == TRANSACTION_TIMED_OUT){
				setState(SW_RESETTING);
			} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
				//grab new measurement
				computeSerialNumber();
				setState(READ_FLOW);				//TODO do something more clever than just going back to idle

			}
			break;
	case WAITING_FOR_FLOW:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			setState(SW_RESETTING);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
			//grab new measurement
			computeFlow();
			++m_sampleCounter;
			setState(READ_TEMPERATURE);
		}
		break;
	case FAIL:
		setState(IDLE);
		//TODO what to do here?
		break;
	case SW_RESETTING:
		if(m_i2cStatus == TRANSACTION_TIMED_OUT){
			//TODO what to do here?
			setState(IDLE);
		} else if(m_i2cStatus == TRANSACTION_COMPLETED_OK){
			//TODO what to do here?
			setState(IDLE);
		}
		break;
	}
	if(slowTimer(&m_timeoutTimer, 10000)){

		//TODO what's the right thing to do here?
		setState(IDLE);
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
	case WAITING_FOR_TEMPERATURE:
		break;
	case WAITING_FOR_FLOW:
		break;
	case FAIL:
		break;
	case SW_RESETTING:
		if(!setupI2cRead(RESET_COMMAND, 0)){

			//TODO what to do here if we can't even reset the bus?
		}
		s = IDLE;
		break;
	case READ_TEMPERATURE:
		if(setupI2cRead(READ_TEMP_COMMAND, 3)){
			s = WAITING_FOR_TEMPERATURE;
		}
		break;
	case READ_FLOW:
		if(setupI2cRead(READ_FLOW_COMMAND, 3)){
			s = WAITING_FOR_FLOW;
		}
		break;
	case READ_SN:
		if(setupI2cRead(READ_SN_COMMAND, 4)){
			s = WAITING_FOR_SN;
		}
		break;
	}
	m_state = s;
}

static bool setupI2cRead(uint16_t command, uint8_t bytesToRead){
	m_txBytes[0] = 0xff & (command >> 8);
	m_txBytes[1] = 0xff & command;
	return i2cQueue(m_dev, I2C_ADDRESS, m_txBytes, 2, m_rxBytes, bytesToRead, &m_i2cStatus);
}

static void computeFlow(void){
	if(checkCrc(m_rxBytes, 2)){
		//checksum passed!
		uint32_t temp = m_rxBytes[0];
		temp <<= 8;
		temp |= m_rxBytes[1];

		float result = (float)temp;

		if(temp == 0xffff){
			result = NAN;
		} else {
			result -= FLOW_OFFSET;
			result *= FLOW_SCALE;
		}
		m_flowSlpm = result;


	}
}
static void computeTemperature(void){
	if(checkCrc(m_rxBytes, 2)){
		//checksum passed!
		uint32_t temp = m_rxBytes[0];
		temp <<= 8;
		temp |= m_rxBytes[1];

		float result = (float)temp;
		result -= TEMP_OFFSET;
		result *= TEMP_SCALE;
		m_temperatureCelsius = result;


	}
}
static void computeSerialNumber(void){
//	if(checkCrc(m_rxBytes, 4)){
		//checksum passed!
		uint32_t temp = m_rxBytes[0];
		temp <<= 8;
		temp |= m_rxBytes[1];
		temp <<=8;
		temp |= m_rxBytes[2];
		temp <<=8;
		temp |= m_rxBytes[3];

		m_serialNumber = temp;


//	}
}
/**
 * @return flow in SLPM
 */
float getSfm3000Flow(void){
	return m_flowSlpm;
}
/**
 * @return the temperature in Celsius
 */
float getSfm3000Temperature(void){
	return m_temperatureCelsius;
}
/**
 * @return the serial number
 */
float getSfm3000SerialNumber(void){
	return m_serialNumber;
}
/**
 * @return the absolute number of samples successfully taken
 */
uint32_t getSfm3000SampleCount(void){
	return m_sampleCounter;
}
