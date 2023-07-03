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

#include <portExpanderManager.h>
#include <tca9534.h>
#include <fastcodeUtil.h>

//*************************************************
//Defines
//*************************************************

#define TOTAL_NUM 8
#define I2C_BASE_ADDRESS 0x38



//*************************************************
//Types
//*************************************************

//*************************************************
//Variables
//*************************************************

static IcType m_types[TOTAL_NUM];
static I2cDev m_buses[TOTAL_NUM];
static uint32_t m_addresses[TOTAL_NUM];
static uint8_t m_inputs[TOTAL_NUM];
static uint8_t m_directions[TOTAL_NUM];
static uint8_t m_outputs[TOTAL_NUM];
static bool m_enabled[TOTAL_NUM];
static uint32_t m_timer = 0;
static uint32_t m_synchronize = TOTAL_NUM;//this is essentially a semaphore to prevent setup of a channel that is in the process of being configured




//*************************************************
//function prototypes
//*************************************************

static uint32_t matchPortExpander(I2cDev bus, uint8_t address);


//*************************************************
//Code
//*************************************************

/**
 * find index of port expander configured for the specified I2C bus and I2C address.
 * If none found then returns the index of the next available spot.
 * If no spot available the returns 0xff
 */
static uint32_t matchPortExpander(I2cDev bus, uint8_t address){
	uint32_t n = 0xff;
	for(uint32_t i = 0; i < TOTAL_NUM; ++i){
		if(m_enabled[i] == true){
			if(m_buses[i] == bus && m_addresses[i] == address){
				n = i;
				break;
			}
		} else if(n == 0xff){
			n = i;
		}
	}
	return n;
}

void portExpanderInit(void){
	for(uint32_t i = 0; i < TOTAL_NUM; ++i){
		m_types     [i] = TCA9534;
		m_buses     [i] = 0;
		m_addresses [i] = 0;
		m_inputs    [i] = 0;
		m_directions[i] = 0;
		m_outputs   [i] = 0;
		m_enabled   [i] = false;
	}
}

/**
 * sets the configuration of the specified port expander
 * @param i the index of this port expander
 * @param bus the I2C bus that the expander is on. If this is I2CNULL_DEV then this channel will be disabled
 * @param address offset of the expander. This is equivalent to the address pin value
 * @param  type the type of expander. Valid types so far: TCA9534
 * @return the index of the expander if found, else the next available space. Returns 0xff if no space left.
 */
void setupPortExpander(uint32_t i, I2cDev bus, uint8_t address, IcType type){

	uint32_t n = i;//matchPortExpander(bus, address);

	if(n < TOTAL_NUM){//!= 0xff){
		m_synchronize = i;
		m_types     [n] = type;
		m_buses     [n] = bus;
		m_addresses [n] = address;
		m_enabled   [n] = true;
		m_synchronize = TOTAL_NUM;
	}

}
/**
 * indicates the number of enabled channels. This will also be the index of the first disabled channel
 */
void setNumChannels(uint32_t n){
	for(int i = n; i < TOTAL_NUM; ++i){
		m_enabled   [i] = false;
	}
}
/**
 * sets the configuration of the specified pin of the specified port expander
 * @param i the index of the port expander
 * @param pinMask a mask representing the pin to config.  E.g. if the pin number is 2 then the mask is 0b100.
 * @param config how to configure the pin
 */
void setupPortExpanderPin(uint32_t i, uint8_t pinMask, PinConfig config){

	if(m_enabled[i] == true){
		switch(config){
		case INPUT_LO:
			m_directions[i] |= pinMask;
			break;
		case INPUT_HI:
			m_directions[i] |= pinMask;
			break;
		case OUTPUT_CLEAR:
			m_directions[i] &= ~pinMask;
			m_outputs[i] &= ~pinMask;
			break;
		case OUTPUT_SET:
			m_directions[i] &= ~pinMask;
			m_outputs[i] |= pinMask;
			break;
		case OUTPUT_NO_CHANGE:
			m_directions[i] &= ~pinMask;
			break;
		}

	}
}
uint32_t getPortExpanderCount(void){
	return TOTAL_NUM;
}
/**
 * reconstructs the port config based on the current state of the port.
 * This will not necessarily reflect how the port was initially configured.
 * @param i the index of the specified port expander
 * @param pinMask a mask indicating the pin number in questions. E.g. if the pin number is 2 then the mask is 0b100.
 */
PinConfig getPortPinConfig(uint32_t i, uint32_t pinMask){
	PinConfig result = INPUT_LO;
	if(i >= 0 && i < TOTAL_NUM){

		uint8_t outs = m_outputs[i];
		uint8_t ins = m_inputs[i];
		uint8_t dirs = m_directions[i];

		if(dirs & pinMask){
			//this is an input
			if(ins & pinMask){
				result = INPUT_HI;
			} else {
				result = INPUT_LO;
			}
		} else {
			//this is an output
			if(outs & pinMask){
				result = OUTPUT_SET;
			} else {
				result = OUTPUT_CLEAR;
			}
		}
	}
	return result;
}
I2cDev getPortExpanderBus(uint32_t i){
	I2cDev result = 0;

	if(i >= 0 && i < TOTAL_NUM){
		result = m_buses[i];
	}
	return result;
}
uint32_t getPortExpanderAddress(uint32_t i){
	uint32_t result = 0;

	if(i >= 0 && i < TOTAL_NUM){
		result = m_addresses[i];
	}
	return result;
}
IcType getPortExpanderType(uint32_t i){
	IcType result = TCA9534;

	if(i >= 0 && i < TOTAL_NUM){
		result = m_types[i];
	}
	return result;
}
bool isPortExpanderEnabled(uint32_t i){
	bool result = false;
	if(i >= 0 && i < TOTAL_NUM){
		result = m_enabled[i];
	}
	return result;
}
void portExpanderSlowCode(void){
	if(slowTimer(&m_timer, 20000)){
		for(uint32_t i = 0; i < TOTAL_NUM; ++i){
			if(m_synchronize != i){//don't do this channel if it's in the middle of being configured.
				if(m_enabled[i]){
					switch(m_types[i]){
					case TCA9534:
						if(queueTca9534(m_buses[i], m_addresses[i], m_directions[i], m_outputs[i], &(m_inputs[i]))){
						} else {
							asm("nop");
						}


						break;
					default:
						break;
					}
				}
			}
		}
	}
	tca9534SlowCode();
}

