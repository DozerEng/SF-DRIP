/*
Copyright (c) 2019 STARFISH PRODUCT ENGINEERING INC.

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
#include "lp5018.h"
//#include <stdint.h>


//*************************************************
//defines
//*************************************************

#define LP5018_ADDRESS (0b01010000) //ADDR1, ADDR0 = GND, GND
//#define ADDRESS 0b0101001 //ADDR1, ADDR0 = GND, VCC
//#define ADDRESS 0b0101010 //ADDR1, ADDR0 = VCC, GND
//#define ADDRESS 0b0101011 //ADDR1, ADDR0 = VCC, VCC


#define REG_DEVICE_CONFIG0  0
#define REG_DEVICE_CONFIG1  1
#define REG_LED_CONFIG0  2
#define REG_BANK_BRIGHTNESS  3
#define REG_BANK_A_COLOR  4
#define REG_BANK_B_COLOR  5
#define REG_BANK_C_COLOR  6
#define REG_LED0_BRIGHTNESS  7
#define REG_LED1_BRIGHTNESS  8
#define REG_LED2_BRIGHTNESS  9
#define REG_LED3_BRIGHTNESS  10
#define REG_LED4_BRIGHTNESS  11
#define REG_LED5_BRIGHTNESS  12
#define REG_LED6_BRIGHTNESS  13 //only LP5024
#define REG_LED7_BRIGHTNESS  14 //only LP5024
#define REG_OUT0_COLOR  15
#define REG_OUT1_COLOR  16
#define REG_OUT2_COLOR  17
#define REG_OUT3_COLOR  18
#define REG_OUT4_COLOR  19
#define REG_OUT5_COLOR  20
#define REG_OUT6_COLOR  21
#define REG_OUT7_COLOR  22
#define REG_OUT8_COLOR  23
#define REG_OUT9_COLOR  24
#define REG_OUT10_COLOR  25
#define REG_OUT11_COLOR  26
#define REG_OUT12_COLOR  27
#define REG_OUT13_COLOR  28
#define REG_OUT14_COLOR  29
#define REG_OUT15_COLOR  30
#define REG_OUT16_COLOR  31
#define REG_OUT17_COLOR  32
#define REG_OUT18_COLOR  33 //only LP5024
#define REG_OUT19_COLOR  34 //only LP5024
#define REG_OUT20_COLOR  35 //only LP5024
#define REG_OUT21_COLOR  36 //only LP5024
#define REG_OUT22_COLOR  37 //only LP5024
#define REG_OUT23_COLOR  38 //only LP5024
#define REG_RESET  39

#define REG_TOTAL 40


#define LED_NUM (REG_OUT17_COLOR + 1 - REG_OUT0_COLOR)


#define BIT_CHIP_EN (1<<6) //DEVICE_CONFIG0 reg

#define BIT_LOG_SCALE_EN (1<<5) //DEVICE_CONFIG1 reg
#define BIT_POWER_SAVE_EN (1<<4) //DEVICE_CONFIG1 reg
#define BIT_AUTO_INCR_EN (1<<3) //DEVICE_CONFIG1 reg
#define BIT_PWM_DITHERING_EN (1<<2) //DEVICE_CONFIG1 reg
#define BIT_MAX_CURRENT_OPTION (1<<1) //DEVICE_CONFIG1 reg
#define BIT_LED_GLOBAL_OFF (1<<0) //DEVICE_CONFIG1 reg

#define BIT_LED7_BANK_EN (1<<7) //LED_CONFIG0 reg (only LP5024)
#define BIT_LED6_BANK_EN (1<<6) //LED_CONFIG0 reg (only LP5024)
#define BIT_LED5_BANK_EN (1<<5) //LED_CONFIG0 reg
#define BIT_LED4_BANK_EN (1<<4) //LED_CONFIG0 reg
#define BIT_LED3_BANK_EN (1<<3) //LED_CONFIG0 reg
#define BIT_LED2_BANK_EN (1<<2) //LED_CONFIG0 reg
#define BIT_LED1_BANK_EN (1<<1) //LED_CONFIG0 reg
#define BIT_LED0_BANK_EN (1<<0) //LED_CONFIG0 reg


//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************

static float m_ledValues[LED_NUM];

//an array of bytes to configure the led driver. It will be written from address 0 (REG_DEVICE_CONFIG0)
//the elements in the array correspond to the registers of the chip
static uint8_t i2cBytes[REG_TOTAL + 1];
static uint8_t* configBytes = i2cBytes + 1;
static TransactionStatus i2cStatus;
//*************************************************
//function prototypes
//*************************************************

static void setupConfigRegs(void);

//*************************************************
//code
//*************************************************


void lp5018Init(I2cDev dev){
	for(uint32_t i = 0; i < LED_NUM; ++i){
		m_ledValues[i] = 0.0;
	}
}


/**
 * Sets one of the LED values
 */
void setLp5018LValue(uint8_t i, float value){

	if(i < LED_NUM){
		m_ledValues[i] = value > 1.0f ? 1.0f : (value < 0.0f ? 0.0f : value);
	}




}
/**
 * This should be called at the required update rate for the LP5018
 * This triggers an I2C data dump to the chip
 * Probably only call every 100ms or slower.
 */
bool lp5018SlowCode(I2cDev dev){
	bool result = false;
	if(i2cStatus == TRANSACTION_COMPLETED_OK){
		result = true;
	}
	setupConfigRegs();
	i2cQueue(dev, LP5018_ADDRESS, i2cBytes, REG_TOTAL + 1, (uint8_t*)0, 0, &i2cStatus);
	return result;
}

void setupConfigRegs(void){
	for(int j = 0; j < LED_NUM/3; ++j){
		float max = 0;
		uint8_t idx = 0;
		for(int k = 0; k < 3; ++k){
			idx = j*3 + k;
			float v = m_ledValues[idx];
			if(max < v){
				max = v;
			}
		}
		for(int k = 0; k < 3; ++k){
			idx = j*3 + k;
			if(max > 0){
				configBytes[idx + REG_OUT0_COLOR] = (uint8_t)(m_ledValues[idx]/max*255);
			} else {
				configBytes[idx + REG_OUT0_COLOR] = 0;
			}
		}
		configBytes[REG_LED0_BRIGHTNESS + j] = (uint8_t)(max*255);

	}


	i2cBytes[0] = 0;
	configBytes[REG_DEVICE_CONFIG0] = BIT_CHIP_EN;
	configBytes[REG_DEVICE_CONFIG1] = BIT_AUTO_INCR_EN;
	configBytes[REG_LED_CONFIG0] = 0;
	configBytes[REG_BANK_BRIGHTNESS] = 0;
	configBytes[REG_BANK_A_COLOR] = 0;
	configBytes[REG_BANK_B_COLOR] = 0;
	configBytes[REG_BANK_C_COLOR] = 0;

}
