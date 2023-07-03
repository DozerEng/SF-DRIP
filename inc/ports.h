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
#ifndef PORTS_H_
#define PORTS_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_gpio.h"
#include "_processorGlobal.h"

//*************************************************
//Types
//*************************************************

typedef enum {

	GPIO_A0_PIN = 	0x00	,	//CN1000.A8
	GPIO_A1_PIN = 	0x01	,	//CN1000.A5
	GPIO_A2_PIN = 	0x02	,	//CN1000.A4
	GPIO_A3_PIN = 	0x03	,	//CN1000.A3
	GPIO_A4_PIN = 	0x04	,	//CN1000.A2, DAC1
	GPIO_A5_PIN = 	0x05	,	//CN1000.B2, DAC2
	GPIO_A6_PIN =	0x06	,	//CN1000.A14
	GPIO_A7_PIN = 	0x07	,	//CN1000.A13
	GPIO_A8_PIN = 	0x08	,	//CN1000.B15, I2C3
	GPIO_A9_PIN = 	0x09	,	//CN1000.B16, USART1
	GPIO_A10_PIN =	0x0a	,	//CN1000.B17, USART1
	GPIO_A11_PIN =	0x0b	,	//CN1000.B18, CN110.3
	GPIO_A12_PIN =	0x0c	,	//CN1000.B19, CN110.2
	GPIO_A13_PIN =	0x0d	,	//JTMS-SWDIO
	GPIO_A14_PIN =	0x0e	,	//JTCK-SWCLK
	GPIO_A15_PIN =	0x0f	,	//CN1000.B20, SPI3

	GPIO_B0_PIN =  	0x10	,	//CN1000.B5
	GPIO_B1_PIN =  	0x11	,	//CN1000.A7
	GPIO_B2_PIN =  	0x12	,	//CN1000.A6, LED100
	GPIO_B3_PIN =  	0x13	,	//CN1000.A15, SPI3
	GPIO_B4_PIN =  	0x14	,	//CN1000.A20, SPI3, I2C3
	GPIO_B5_PIN =  	0x15	,	//CN1000.A21, SPI3
	GPIO_B6_PIN = 	0x16	,	//CN1000.A22, IMU_INT
	GPIO_B7_PIN =  	0x17	,	//CN1000.B22
	GPIO_B8_PIN =  	0x18	,	//CN1000.A17, I2C1
	GPIO_B9_PIN =  	0x19	,	//CN1000.A16, I2C1
	GPIO_B10_PIN = 	0x1a	,	//CN1000.B6, LED101, I2C2
	GPIO_B11_PIN = 	0x1b	,	//N/A
	GPIO_B12_PIN = 	0x1c	,	//CN1000.B7, SPI2, I2C2
	GPIO_B13_PIN = 	0x1d	,	//CN1000.B8, SPI2
	GPIO_B14_PIN = 	0x1e	,	//CN1000.B9, SPI2
	GPIO_B15_PIN = 	0x1f	,	//CN1000.B10, SPI2

	GPIO_C0_PIN =  	0x20	,	//CN1000.A12
	GPIO_C1_PIN =  	0x21	,	//CN1000.A11
	GPIO_C2_PIN =  	0x22	,	//CN1000.A10
	GPIO_C3_PIN =  	0x23	,	//CN1000.A9, VHS_ADC
	GPIO_C4_PIN =  	0x24	,	//CN1000.B3, CN113.3
	GPIO_C5_PIN =  	0x25	,	//CN1000.B4, CN113.2
	GPIO_C6_PIN =   0x26	,	//CN1000.B11, TIM3
	GPIO_C7_PIN =  	0x27	,	//CN1000.B12, TIM3
	GPIO_C8_PIN =  	0x28	,	//CN1000.B13, TIM3
	GPIO_C9_PIN =  	0x29	,	//CN1000.B14, TIM3
	GPIO_C10_PIN = 	0x2a	,	//CN1000.B21, USART3
	GPIO_C11_PIN = 	0x2b	,	//CN1000.A18, USART3
	GPIO_C12_PIN = 	0x2c	,	//
	GPIO_C13_PIN = 	0x2d	,	//
	GPIO_C14_PIN = 	0x2e	, //
	GPIO_C15_PIN = 	0x2f	,	//

	GPIO_D0_PIN  =  0x30     ,//
	GPIO_D1_PIN  =  0x31     ,//
	GPIO_D2_PIN  =  0x32     ,//note that this pin will only be used as a diagnostic LED

	FIRST_PIN = GPIO_A0_PIN,
	LAST_PIN = GPIO_D2_PIN,
	NULL_PIN     =	0xff    ,
	GPIO_NULL_PIN = 0xff   ,
#if defined( SFDQ_HW_REV_X6)
//	GPIO_SPI3_NSS_PIN = GPIO_A15_PIN,
//	GPIO_SPI2_NSS_PIN = GPIO_B12_PIN,
	GPIO_LED100_PIN = GPIO_B2_PIN,
	GPIO_LED101_PIN = GPIO_D2_PIN,
//	GPIO_IMU_INT_PIN = GPIO_B6_PIN

#elif defined( SFDQ_HW_REV_X5)
//	GPIO_SPI3_NSS_PIN = GPIO_A15_PIN,
//	GPIO_SPI2_NSS_PIN = GPIO_B12_PIN,
	GPIO_LED100_PIN = GPIO_B2_PIN,
	GPIO_LED101_PIN_2 = GPIO_B10_PIN,
	GPIO_LED101_PIN = GPIO_D2_PIN,
//	GPIO_IMU_INT_PIN = GPIO_B6_PIN

#elif defined( SFDQ_HW_REV_X3)
//	GPIO_SPI3_NSS_PIN = GPIO_A15_PIN,
//	GPIO_SPI2_NSS_PIN = GPIO_B12_PIN,
	GPIO_LED100_PIN = GPIO_A7_PIN,
	GPIO_LED101_PIN = GPIO_A6_PIN,
//	GPIO_IMU_INT_PIN = GPIO_B6_PIN,

	HB_EN_1_PIN = GPIO_C9_PIN,
	HB_EN_2_PIN = GPIO_C14_PIN,
	HB_EN_3_PIN = GPIO_C15_PIN,
	HB_EN_4_PIN = GPIO_C13_PIN
#endif



} PortPin;

typedef enum {
	PIN_IN,
	PIN_OUT_LO,
	PIN_OUT_HI,
	PIN_AN,
	PIN_AF1,
	PIN_AF2,
	PIN_OUT

} PinDir;

typedef enum {
	PULL_NONE = 0,
	PULL_UP = 1,
	PULL_DOWN = 2,
	PULL_RESERVED = 3
} PullUpDownType;

//*************************************************
//defines
//*************************************************


//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************
void setPin(PortPin pin, bool onOff);
bool isPinInputSet(PortPin pin);
void togglePin(PortPin pin);
bool isPinOutputSet(PortPin pin);
bool isPinChangeAllowed(PortPin pin);
bool isPinDir(PortPin pin, PinDir dir);
PinDir getPinDirection(PortPin pin);
void setPinDirection(PortPin pin, PinDir mode);
//void setPinPullup(PortPin pin, bool enable);
/**
 * sets the specified pin to alternate function mode and sets the AF register to the specified value.
 * Note that unless the comm protocol pin direction is set to NO_EFFECT, this will be overwritten.
 * @param pin the pin to set
 * @param the alternate function desired. See the STM32F446 Reference Manual for details
 */
void setPinRawAf(PortPin pin, uint8_t af);
void initPins();
void setPinPullup(PortPin pin, PullUpDownType t);
PullUpDownType getPinPullup(PortPin pin);


#endif /* PORTS_H_ */
