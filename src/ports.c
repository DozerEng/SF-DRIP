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
//#include "_processorGlobal.h"

#include "ports.h"
#include <stdbool.h>
#include <stdint.h>
//#include "stm32f4xx_gpio.h"


//********************************************************
//Defines
//********************************************************

#define PINTYPE_OC ((uint8_t)0b10000000)
#define PINTYPE_PP ((uint8_t)0b00000000)
#define PINTYPE_OCMASK ((uint8_t)0b01111111)

#define GPIO_AF_I2C1_OC (GPIO_AF_I2C1 + PINTYPE_OC)
#define GPIO_AF_I2C2_OC (GPIO_AF_I2C2 + PINTYPE_OC)
#define GPIO_AF_I2C3_OC (GPIO_AF_I2C3 + PINTYPE_OC)

//********************************************************
//TypeDefs
//********************************************************

typedef struct {
	GPIO_TypeDef* port;
	uint16_t pinSource;
	uint16_t pinMask;
	PinDir defaultDir;
	uint8_t af1;//this is the alternate function as well as the top bit being open collector or not
	uint8_t af2;//ditto
	GPIOOType_TypeDef oType;
	bool changeAllowed;
} GpioStruct;

static const GpioStruct gpios[] = {


//the following pin defs are for the X5 PCB rev



//   port,       pinSource,          pinMask,       defaultDir,  af1,       ,     af2     , oType       ,  changeAllowed

	{ GPIOA,	 GPIO_PinSource0,	 GPIO_Pin_0,	PIN_IN,	GPIO_AF_TIM2	, 0, GPIO_OType_PP,  true},	//GPIO_A0_PIN =  0,	//CN1000.A8
	{ GPIOA,	 GPIO_PinSource1,	 GPIO_Pin_1,	PIN_IN,	GPIO_AF_TIM2	, 0, GPIO_OType_PP,  true},	//GPIO_A1_PIN =  1,	//CN1000.A5
	{ GPIOA,	 GPIO_PinSource2,	 GPIO_Pin_2,	PIN_IN,	GPIO_AF_TIM2	, GPIO_AF_USART2, GPIO_OType_PP,  true},	//GPIO_A2_PIN =  2,	//CN1000.A4
	{ GPIOA,	 GPIO_PinSource3,	 GPIO_Pin_3,	PIN_IN,	GPIO_AF_TIM2	, GPIO_AF_USART2, GPIO_OType_PP,  true},	//GPIO_A3_PIN =  3,	//CN1000.A3
	{ GPIOA,	 GPIO_PinSource4,	 GPIO_Pin_4,	PIN_AN,	GPIO_AF_SPI1	,            0, GPIO_OType_PP,  true},	//GPIO_A4_PIN =  4,	//CN1000.A2, DAC1
	{ GPIOA,	 GPIO_PinSource5,	 GPIO_Pin_5,	PIN_AN,	GPIO_AF_SPI1	,            0, GPIO_OType_PP,  true},	//GPIO_A5_PIN =  5,	//CN1000.B2, DAC2
	{ GPIOA,	 GPIO_PinSource6,	 GPIO_Pin_6,	PIN_IN,	GPIO_AF_SPI1	, GPIO_AF_TIM3, GPIO_OType_PP,  true},	//GPIO_A6_PIN =  6,	//CN1000.A14
	{ GPIOA,	 GPIO_PinSource7,	 GPIO_Pin_7,	PIN_IN,	GPIO_AF_SPI1	, GPIO_AF_TIM3, GPIO_OType_PP,  true},	//GPIO_A7_PIN =  7,	//CN1000.A13
	{ GPIOA,	 GPIO_PinSource8,	 GPIO_Pin_8,	PIN_IN,	0				, GPIO_AF_I2C3_OC, GPIO_OType_PP,  true},	//GPIO_A8_PIN =  8,	//CN1000.B15, I2C3
	{ GPIOA,	 GPIO_PinSource9,	 GPIO_Pin_9,	PIN_AF1,	GPIO_AF_USART1	,            0, GPIO_OType_PP, false},	//GPIO_A9_PIN =  9,	//CN1000.B16, USART1
	{ GPIOA,	GPIO_PinSource10,	GPIO_Pin_10,	PIN_AF1,	GPIO_AF_USART1	,            0, GPIO_OType_PP, false},	//GPIO_A10_PIN = 10,	//CN1000.B17, USART1
//	{ GPIOA,	GPIO_PinSource11,	GPIO_Pin_11,	PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_A11_PIN = 11,	//CN1000.B18, CN110.3
	{ GPIOA,	GPIO_PinSource11,	GPIO_Pin_11,	PIN_OUT,	0				,            0, GPIO_OType_PP,  true},	//GPIO_A11_PIN = 11,	//CN1000.B18, CN110.3
	{ GPIOA,	GPIO_PinSource12,	GPIO_Pin_12,	PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_A12_PIN = 12,	//CN1000.B19, CN110.2
	{ GPIOA,	GPIO_PinSource13,	GPIO_Pin_13,	PIN_AF1,	GPIO_AF_SWJ		,            0, GPIO_OType_PP,  false},	//GPIO_A13_PIN = 13,	//JTMS-SWDIO - if this gets changed it breaks debugging
	{ GPIOA,	GPIO_PinSource14,	GPIO_Pin_14,	PIN_AF1,	GPIO_AF_SWJ		,            0, GPIO_OType_PP,  false},	//GPIO_A14_PIN = 14,	//JTCK-SWCLK - if this gets changed it breaks debugging
	{ GPIOA,	GPIO_PinSource15,	GPIO_Pin_15,	PIN_IN, 	0    	,            0, GPIO_OType_PP,  true},	//GPIO_A15_PIN = 15,	//CN1000.B20,


	{ GPIOB,	GPIO_PinSource0,	GPIO_Pin_0,		PIN_IN,	0				, GPIO_AF_TIM3, GPIO_OType_PP,  true},	//GPIO_B0_PIN =  16,	//CN1000.B5
	{ GPIOB,	GPIO_PinSource1,	GPIO_Pin_1,		PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_B1_PIN =  17,	//CN1000.A7
	{ GPIOB,	GPIO_PinSource2,	GPIO_Pin_2,		PIN_OUT,	0				,            0, GPIO_OType_PP,  true},	//GPIO_B2_PIN =  18,	//CN1000.A6, LED100
	{ GPIOB,	GPIO_PinSource3,	GPIO_Pin_3,		PIN_IN,	GPIO_AF_SPI3  	,            0, GPIO_OType_PP,  true},	//GPIO_B3_PIN =  19,	//CN1000.A15, SPI3
	{ GPIOB,	GPIO_PinSource4,	GPIO_Pin_4,		PIN_IN,	GPIO_AF_SPI3    , GPIO_AF_I2C3_OC, GPIO_OType_PP,  true},	//GPIO_B4_PIN =  20,	//CN1000.A20, SPI3, I2C3
	{ GPIOB,	GPIO_PinSource5,	GPIO_Pin_5,		PIN_IN,	GPIO_AF_SPI3  	,            0, GPIO_OType_PP,  true},	//GPIO_B5_PIN =  21,	//CN1000.A21, SPI3
	{ GPIOB,	GPIO_PinSource6,	GPIO_Pin_6,		PIN_IN,	GPIO_AF_I2C1_OC		,            0, GPIO_OType_PP,  true},	//GPIO_B6_PIN =  22,	//CN1000.A22, IMU_INT
	{ GPIOB,	GPIO_PinSource7,	GPIO_Pin_7,		PIN_IN,	GPIO_AF_I2C1_OC		,            0, GPIO_OType_PP,  true},	//GPIO_B7_PIN =  23,	//CN1000.B22
	{ GPIOB,	GPIO_PinSource8,	GPIO_Pin_8,		PIN_AF1,	GPIO_AF_I2C1_OC	,            0, GPIO_OType_OD,  true},	//GPIO_B8_PIN =  24,	//CN1000.A17, I2C1 SCL
	{ GPIOB,	GPIO_PinSource9,	GPIO_Pin_9,		PIN_AF1,	GPIO_AF_I2C1_OC	,            0, GPIO_OType_OD,  true},	//GPIO_B9_PIN =  25,	//CN1000.A16, I2C1 SDA
	{ GPIOB,	GPIO_PinSource10,	GPIO_Pin_10,	PIN_OUT,	GPIO_AF_I2C2_OC    ,            0, GPIO_OType_PP,  true},	//GPIO_B10_PIN = 26,	//CN1000.B6, LED101, I2C2
	{ GPIOB,	GPIO_PinSource11,	GPIO_Pin_11,	PIN_IN,	0				,            0, GPIO_OType_PP, false},	//GPIO_B11_PIN = 27,	//N/A
	{ GPIOB,	GPIO_PinSource12,	GPIO_Pin_12,	PIN_OUT,	0    ,            0, GPIO_OType_PP,  true},	//GPIO_B12_PIN = 28,	//CN1000.B7, SPI2, I2C2
	{ GPIOB,	GPIO_PinSource13,	GPIO_Pin_13,	PIN_AF1,	GPIO_AF_SPI2    ,            0, GPIO_OType_PP,  true},	//GPIO_B13_PIN = 29,	//CN1000.B8, SPI2
	{ GPIOB,	GPIO_PinSource14,	GPIO_Pin_14,	PIN_AF1,	GPIO_AF_SPI2    ,            0, GPIO_OType_PP,  true},	//GPIO_B14_PIN = 30,	//CN1000.B9, SPI2
	{ GPIOB,	GPIO_PinSource15,	GPIO_Pin_15,	PIN_AF1,	GPIO_AF_SPI2    ,            0, GPIO_OType_PP,  true},	//GPIO_B15_PIN = 31,	//CN1000.B10, SPI2

	{ GPIOC,	GPIO_PinSource0,	GPIO_Pin_0,		PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_C0_PIN =   32,	//CN1000.A12
	{ GPIOC,	GPIO_PinSource1,	GPIO_Pin_1,		PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_C1_PIN =   33,	//CN1000.A11
	{ GPIOC,	GPIO_PinSource2,	GPIO_Pin_2,		PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_C2_PIN =   34,	//CN1000.A10
	{ GPIOC,	GPIO_PinSource3,	GPIO_Pin_3,		PIN_AN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_C3_PIN =   35,	//CN1000.A9, VHS_ADC
	{ GPIOC,	GPIO_PinSource4,	GPIO_Pin_4,		PIN_IN,	GPIO_AF_SPI1	,            0, GPIO_OType_PP,  true},	//GPIO_C4_PIN =   36,	//CN1000.B3, CN113.3
	{ GPIOC,	GPIO_PinSource5,	GPIO_Pin_5,		PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_C5_PIN =   37,	//CN1000.B4, CN113.2
	{ GPIOC,	GPIO_PinSource6,	GPIO_Pin_6,		PIN_AF1,	GPIO_AF_TIM3	,  GPIO_AF_USART6, GPIO_OType_PP, false},	//GPIO_C6_PIN =   38,	//CN1000.B11, TIM3
	{ GPIOC,	GPIO_PinSource7,	GPIO_Pin_7,		PIN_AF1,	GPIO_AF_TIM3	,  GPIO_AF_USART6, GPIO_OType_PP, false},	//GPIO_C7_PIN =   39,	//CN1000.B12, TIM3
	{ GPIOC,	GPIO_PinSource8,	GPIO_Pin_8,		PIN_AF1,	GPIO_AF_TIM3	,            0, GPIO_OType_PP, false},	//GPIO_C8_PIN =   40,	//CN1000.B13, TIM3
	{ GPIOC,	GPIO_PinSource9,	GPIO_Pin_9,		PIN_AF1,	GPIO_AF_TIM3,  GPIO_AF_I2C3_OC, GPIO_OType_PP, false},	//GPIO_C9_PIN =   41,	//CN1000.B14, TIM3
	{ GPIOC,	GPIO_PinSource10,	GPIO_Pin_10,	PIN_AF1,	GPIO_AF_USART3	, GPIO_AF_SPI3, GPIO_OType_PP,  true},	//GPIO_C10_PIN =  42,	//CN1000.B21, USART3
	{ GPIOC,	GPIO_PinSource11,	GPIO_Pin_11,	PIN_AF1,	GPIO_AF_USART3	, GPIO_AF_SPI3, GPIO_OType_PP,  true},	//GPIO_C11_PIN =  43,	//CN1000.A18, USART3
	{ GPIOC,	GPIO_PinSource12,	GPIO_Pin_12,	PIN_IN,	GPIO_AF_I2C2_OC    ,  GPIO_AF_SPI3, GPIO_OType_PP,  true},	//GPIO_C12_PIN =  44,	//
	{ GPIOC,	GPIO_PinSource13,	GPIO_Pin_13,	PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_C13_PIN =  45,	//
	{ GPIOC,	GPIO_PinSource14,	GPIO_Pin_14,	PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_C14_PIN =  46, //
	{ GPIOC,	GPIO_PinSource15,	GPIO_Pin_15,	PIN_IN,	0				,            0, GPIO_OType_PP,  true},	//GPIO_C15_PIN =  47,	//

	{ GPIOD,	GPIO_PinSource0,	GPIO_Pin_0,		 PIN_IN, 0				,            0, GPIO_OType_PP,  true},	//GPIO_D0_PIN =  48,	//
	{ GPIOD,	GPIO_PinSource1,	GPIO_Pin_1,		 PIN_IN, 0				,            0, GPIO_OType_PP,  true},	//GPIO_D1_PIN =  49,	//
	{ GPIOD,	GPIO_PinSource2,	GPIO_Pin_2,		PIN_OUT, 0				,            0, GPIO_OType_PP,  true},	//GPIO_D2_PIN =  50,	//

//the following pin defs are for the X2,X3 revs
//	{ GPIOA,	 GPIO_PinSource0,	 GPIO_Pin_0,	PIN_IN	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A0_PIN =  0,	//CN1000.A8
//	{ GPIOA,	 GPIO_PinSource1,	 GPIO_Pin_1,	PIN_IN	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A1_PIN =  1,	//CN1000.A5
//	{ GPIOA,	 GPIO_PinSource2,	 GPIO_Pin_2,	PIN_IN	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A2_PIN =  2,	//CN1000.A4
//	{ GPIOA,	 GPIO_PinSource3,	 GPIO_Pin_3,	PIN_IN	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A3_PIN =  3,	//CN1000.A3
//	{ GPIOA,	 GPIO_PinSource4,	 GPIO_Pin_4,	PIN_AN	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A4_PIN =  4,	//CN1000.A2, DAC1
//	{ GPIOA,	 GPIO_PinSource5,	 GPIO_Pin_5,	PIN_AN	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A5_PIN =  5,	//CN1000.B2, DAC2
//	{ GPIOA,	 GPIO_PinSource6,	 GPIO_Pin_6,	PIN_OUT	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A6_PIN =  6,	//CN1000.A14, LED101
//	{ GPIOA,	 GPIO_PinSource7,	 GPIO_Pin_7,	PIN_OUT	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A7_PIN =  7,	//CN1000.A13, LED100
//	{ GPIOA,	 GPIO_PinSource8,	 GPIO_Pin_8,	PIN_IN	, 0					, GPIO_AF_I2C3	, GPIO_OType_PP	, true},	//GPIO_A8_PIN =  8,	//CN1000.B15, I2C3
//	{ GPIOA,	 GPIO_PinSource9,	 GPIO_Pin_9,	PIN_AF1	, GPIO_AF_USART1	, 0				, GPIO_OType_PP	, true},	//GPIO_A9_PIN =  9,	//CN1000.B16, USART1
//	{ GPIOA,	GPIO_PinSource10,	GPIO_Pin_10,	PIN_AF1	, GPIO_AF_USART1	, 0				, GPIO_OType_PP	, true},	//GPIO_A10_PIN = 10,	//CN1000.B17, USART1
//	{ GPIOA,	GPIO_PinSource11,	GPIO_Pin_11,	PIN_IN	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A11_PIN = 11,	//CN1000.B18, CN110.3
//	{ GPIOA,	GPIO_PinSource12,	GPIO_Pin_12,	PIN_IN	, 0					, 0				, GPIO_OType_PP	, true},	//GPIO_A12_PIN = 12,	//CN1000.B19, CN110.2
//	{ GPIOA,	GPIO_PinSource13,	GPIO_Pin_13,	PIN_AF1	, GPIO_AF_SWJ		, 0				, GPIO_OType_PP	, false},	//GPIO_A13_PIN = 13,	//JTMS-SWDIO - if this gets changed it breaks debugging
//	{ GPIOA,	GPIO_PinSource14,	GPIO_Pin_14,	PIN_AF1	, GPIO_AF_SWJ		, 0				, GPIO_OType_PP	, false},	//GPIO_A14_PIN = 14,	//JTCK-SWCLK - if this gets changed it breaks debugging
//	{ GPIOA,	GPIO_PinSource15,	GPIO_Pin_15,	PIN_OUT	, 0		    		, 0				, GPIO_OType_PP	, true},	//GPIO_A15_PIN = 15,	//CN1000.B20, SPI3 CS this is just an output
//
//
//	{ GPIOB,	GPIO_PinSource0,	GPIO_Pin_0,		PIN_IN	,	0				, 0				, GPIO_OType_PP	,  true},	//GPIO_B0_PIN =  16,	//CN1000.B5
//	{ GPIOB,	GPIO_PinSource1,	GPIO_Pin_1,		PIN_AF1	, GPIO_AF_TIM3		, 0				, GPIO_OType_PP	,  false},	//GPIO_B1_PIN =  17,	//CN1000.A7, Half-bridge PWM
//	{ GPIOB,	GPIO_PinSource2,	GPIO_Pin_2,		PIN_AF1	, GPIO_AF_TIM2		, 0				, GPIO_OType_PP	,  false},	//GPIO_B2_PIN =  18,	//CN1000.A6, Half-bridge PWM
//	{ GPIOB,	GPIO_PinSource3,	GPIO_Pin_3,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_B3_PIN =  19,	//CN1000.A15, SPI3
//	{ GPIOB,	GPIO_PinSource4,	GPIO_Pin_4,		PIN_AF1	, GPIO_AF_TIM3		, 0				, GPIO_OType_PP	,  false},	//GPIO_B4_PIN =  20,	//CN1000.A20, , Half-bridge PWM
//	{ GPIOB,	GPIO_PinSource5,	GPIO_Pin_5,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_B5_PIN =  21,	//CN1000.A21, SPI3
//	{ GPIOB,	GPIO_PinSource6,	GPIO_Pin_6,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_B6_PIN =  22,	//CN1000.A22, IMU_INT
//	{ GPIOB,	GPIO_PinSource7,	GPIO_Pin_7,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_B7_PIN =  23,	//CN1000.B22
//	{ GPIOB,	GPIO_PinSource8,	GPIO_Pin_8,		PIN_AF1	, GPIO_AF_I2C1		, 0				, GPIO_OType_PP	,  true},	//GPIO_B8_PIN =  24,	//CN1000.A17, I2C1
//	{ GPIOB,	GPIO_PinSource9,	GPIO_Pin_9,		PIN_AF1	, GPIO_AF_I2C1		, 0				, GPIO_OType_OD	,  true},	//GPIO_B9_PIN =  25,	//CN1000.A16, I2C1
//	{ GPIOB,	GPIO_PinSource10,	GPIO_Pin_10,	PIN_IN	, GPIO_AF_I2C2    	, 0				, GPIO_OType_PP	,  true},	//GPIO_B10_PIN = 26,	//CN1000.B6, I2C2
//	{ GPIOB,	GPIO_PinSource11,	GPIO_Pin_11,	PIN_IN	, 0					, 0				, GPIO_OType_PP	, false},	//GPIO_B11_PIN = 27,	//N/A
//	{ GPIOB,	GPIO_PinSource12,	GPIO_Pin_12,	PIN_OUT	, 0			    	, 0				, GPIO_OType_PP	,  true},	//GPIO_B12_PIN = 28,	//CN1000.B7, SPI2, I2C2, SPI2 CS only needs to be output
//	{ GPIOB,	GPIO_PinSource13,	GPIO_Pin_13,	PIN_AF1	, GPIO_AF_SPI2    	, 0				, GPIO_OType_PP	,  true},	//GPIO_B13_PIN = 29,	//CN1000.B8, SPI2
//	{ GPIOB,	GPIO_PinSource14,	GPIO_Pin_14,	PIN_AF1	, GPIO_AF_SPI2    	, 0				, GPIO_OType_PP	,  true},	//GPIO_B14_PIN = 30,	//CN1000.B9, SPI2
//	{ GPIOB,	GPIO_PinSource15,	GPIO_Pin_15,	PIN_AF1	, GPIO_AF_SPI2    	, 0				, GPIO_OType_PP	,  true},	//GPIO_B15_PIN = 31,	//CN1000.B10, SPI2
//
//	{ GPIOC,	GPIO_PinSource0,	GPIO_Pin_0,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_C0_PIN =   32,	//CN1000.A12
//	{ GPIOC,	GPIO_PinSource1,	GPIO_Pin_1,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_C1_PIN =   33,	//CN1000.A11
//	{ GPIOC,	GPIO_PinSource2,	GPIO_Pin_2,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_C2_PIN =   34,	//CN1000.A10
//	{ GPIOC,	GPIO_PinSource3,	GPIO_Pin_3,		PIN_AN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_C3_PIN =   35,	//CN1000.A9, VHS_ADC
//	{ GPIOC,	GPIO_PinSource4,	GPIO_Pin_4,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_C4_PIN =   36,	//CN1000.B3, CN113.3
//	{ GPIOC,	GPIO_PinSource5,	GPIO_Pin_5,		PIN_IN	, 0					, 0				, GPIO_OType_PP	,  true},	//GPIO_C5_PIN =   37,	//CN1000.B4, CN113.2
//	{ GPIOC,	GPIO_PinSource6,	GPIO_Pin_6,		PIN_AF1	, GPIO_AF_USART6	, 0				, GPIO_OType_PP	, false},	//GPIO_C6_PIN =   38,	//CN1000.B11, USART6 main comms for this board rev
//	{ GPIOC,	GPIO_PinSource7,	GPIO_Pin_7,		PIN_AF1	, GPIO_AF_USART6	, 0				, GPIO_OType_PP	, false},	//GPIO_C7_PIN =   39,	//CN1000.B12, USART6 main comms for this board rev
//	{ GPIOC,	GPIO_PinSource8,	GPIO_Pin_8,		PIN_AF1	, GPIO_AF_TIM3		, 0				, GPIO_OType_PP	, false},	//GPIO_C8_PIN =   40,	//CN1000.B13, , Half-bridge PWM
//	{ GPIOC,	GPIO_PinSource9,	GPIO_Pin_9,		PIN_OUT	, 0					, 0				, GPIO_OType_PP	, false},	//GPIO_C9_PIN =   41,	//CN1000.B14, Half-bridge ZC_EN
//	{ GPIOC,	GPIO_PinSource10,	GPIO_Pin_10,	PIN_AF1	, GPIO_AF_SPI3	    , 0				, GPIO_OType_PP	,  true},	//GPIO_C10_PIN =  42,	//CN1000.B21
//	{ GPIOC,	GPIO_PinSource11,	GPIO_Pin_11,	PIN_AF1	, GPIO_AF_SPI3		, 0				, GPIO_OType_PP	,  true},	//GPIO_C11_PIN =  43,	//CN1000.A18
//	{ GPIOC,	GPIO_PinSource12,	GPIO_Pin_12,	PIN_AF1	, GPIO_AF_SPI3	    , 0				, GPIO_OType_PP	,  true},	//GPIO_C12_PIN =  44,	//
//	{ GPIOC,	GPIO_PinSource13,	GPIO_Pin_13,	PIN_OUT	, 0					, 0				, GPIO_OType_PP	,  false},	//GPIO_C13_PIN =  45,	//Half-bridge ZC_EN
//	{ GPIOC,	GPIO_PinSource14,	GPIO_Pin_14,	PIN_OUT	, 0					, 0				, GPIO_OType_PP	,  false},	//GPIO_C14_PIN =  46, //Half-bridge ZC_EN
//	{ GPIOC,	GPIO_PinSource15,	GPIO_Pin_15,	PIN_OUT	, 0					, 0				, GPIO_OType_PP	,  false},	//GPIO_C15_PIN =  47,	//Half-bridge ZC_EN


};




//*************************************************
//function prototypes
//*************************************************
static void initPin(const GpioStruct* gpio, PinDir dir, GPIOOType_TypeDef oType);
static GPIOMode_TypeDef getPinMode(PinDir dir);
static void configAf(PortPin pin, PinDir dir);
static void setPinOpenDrain(const GpioStruct* gpio, bool enable);




//*************************************************
//local variables
//*************************************************
PinDir m_pinDirs[LAST_PIN + 1];




//********************************************************
//Code
//********************************************************

void setPin(PortPin pin, bool onOff){
	if(pin == NULL_PIN || pin > LAST_PIN){
		return;
	}
	if(onOff){
		GPIO_SetBits( gpios[pin].port, gpios[pin].pinMask);
	} else {
		GPIO_ResetBits( gpios[pin].port, gpios[pin].pinMask);
	}
}

void togglePin(PortPin pin){
	if(pin == NULL_PIN || pin > LAST_PIN){
		return;
	}
	//assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

	uint32_t reset_pins = gpios[pin].port->ODR   & gpios[pin].pinMask;
	uint32_t   set_pins = (~reset_pins) & gpios[pin].pinMask;

	gpios[pin].port->BSRRH |= reset_pins;
	gpios[pin].port->BSRRL |=   set_pins;
}
bool isPinInputSet(PortPin pin){
	if(pin == NULL_PIN || pin > LAST_PIN){
		return false;
	}
	GpioStruct gpio = gpios[pin];
	return (GPIO_ReadInputData(gpio.port) & gpio.pinMask) != 0;
}
bool isPinOutputSet(PortPin pin){
	if(pin == NULL_PIN || pin > LAST_PIN){
		return false;
	}
	GpioStruct gpio = gpios[pin];
	return (GPIO_ReadOutputData(gpio.port) & gpio.pinMask) != 0;
}


bool isPinChangeAllowed(PortPin pin){
	if(pin == NULL_PIN || pin > LAST_PIN){
		return false;
	}

	return gpios[pin].changeAllowed;
}

bool isPinDir(PortPin pin, PinDir dir){
	if(pin == NULL_PIN || pin > LAST_PIN){
		return false;
	}

	return getPinDirection(pin) == dir;
}

void setPinPullup(PortPin pin, PullUpDownType t){
	if(t != PULL_NONE && t != PULL_UP && t != PULL_DOWN){
		return;
	}
	if(pin != NULL_PIN && pin < LAST_PIN){


		const GpioStruct* gpio = &(gpios[pin]);
		uint32_t pu = gpio->port->PUPDR;

		uint32_t temp = pu ^ (t << (gpio->pinSource * 2));

		temp &= (0b11 << (gpio->pinSource * 2));

		if(temp != 0){
			pu ^= temp;
			gpio->port->PUPDR = pu;
		}




	}
}
PullUpDownType getPinPullup(PortPin pin){
	PullUpDownType result = PULL_NONE;
	if(pin != NULL_PIN && pin < LAST_PIN){

		const GpioStruct* gpio = &(gpios[pin]);


		uint32_t pu = gpio->port->PUPDR;
		pu >>= (gpio->pinSource * 2);
		pu &= 0b11;

		result = (PullUpDownType)pu;
//		switch(pu){
//		case 0:
//			//no pullup
//			result = PULL_NONE;
//			break;
//		case 1:
//			result = PULL_UP;
//			break;
//		case 2:
//			result = PULL_DOWN;
//			break;
//		case 3:
//			result = PULL_RESERVED;
//			break;
//		}


	}
	return result;
}
/**
 * set the direction of this pin. Only change it if it's allowed for the pin
 */
void setPinDirection(PortPin pin, PinDir dir){
	//don't do anything unless it needs to change
	const GpioStruct* gpio = &(gpios[pin]);

	PinDir d = dir;
//	if(!gpio->changeAllowed){
//		d = gpio->defaultDir;
//	}

	if(m_pinDirs[pin] != d){



		m_pinDirs[pin] = dir;


		uint32_t moder = gpio->port->MODER;
		uint32_t temp = moder ^ (getPinMode(dir) << (gpio->pinSource * 2));
		temp &= 0b11 << (gpio->pinSource * 2);


		if(temp != 0){
			moder ^= temp;
			gpio->port->MODER = moder;
		}

	}

	switch(d){
	case PIN_IN:
	case PIN_OUT_LO:
	case PIN_OUT_HI:
	case PIN_OUT:
	case PIN_AN:
		break;
	case PIN_AF1:
	case PIN_AF2:
		configAf(pin, dir);
		break;


	}
}

static void setPinOpenDrain(const GpioStruct* gpio, bool enable){
	//don't do anything unless it needs to change

	uint32_t mask = gpio->pinMask;

	uint32_t temp = enable ? mask : 0;

	temp ^= gpio->port->OTYPER;

	temp &= mask;

	if(temp != 0){
		gpio->port->OTYPER ^= mask;
	}



}
PinDir getPinDirection(PortPin pin){
	return m_pinDirs[pin];


}



static GPIOMode_TypeDef getPinMode(PinDir dir){
	GPIOMode_TypeDef m = GPIO_Mode_IN;
	switch(dir){
	case PIN_IN:
		m =  GPIO_Mode_IN;
		break;
	case PIN_OUT_HI:
	case PIN_OUT_LO:
	case PIN_OUT:
		m = GPIO_Mode_OUT;
		break;
	case PIN_AN:
		m = GPIO_Mode_AN;
		break;
	case PIN_AF1:
	case PIN_AF2:
		m = GPIO_Mode_AF;
		break;
	}
	return m;

}
void initPins(){



	for(PortPin i = FIRST_PIN; i <= LAST_PIN; ++i){


		const GpioStruct* g = &(gpios[i]);

		PinDir dir = g->defaultDir;
		m_pinDirs[i] = dir;
		GPIOOType_TypeDef oType = g->oType;

		initPin(g, dir, oType);
		configAf(i, dir);

	}
}

static void initPin(const GpioStruct* gpio, PinDir dir, GPIOOType_TypeDef oType){
	GPIO_TypeDef* port = gpio->port;
	uint32_t pinMask = gpio->pinMask;

	GPIOMode_TypeDef mode = getPinMode(dir);
//	GPIOOType_TypeDef oType = GPIO_OType_PP;

	GPIO_InitTypeDef itd;
	GPIO_StructInit(&itd);
	itd.GPIO_Mode = mode;
	itd.GPIO_Pin = pinMask;
	itd.GPIO_PuPd = GPIO_PuPd_NOPULL;
	itd.GPIO_Speed = GPIO_Speed_2MHz;
	itd.GPIO_OType = oType;

	GPIO_Init(port, &itd);
}

static void configAf(PortPin pin, PinDir dir){

//	if(!gpio.changeAllowed){
//		return;
//	}
	const GpioStruct* gpio = &(gpios[pin]);


	//only do stuff if pin is configured as alternate function
	switch(dir){
	case PIN_IN:
	case PIN_OUT:
	case PIN_OUT_LO:
	case PIN_OUT_HI:
	case PIN_AN:
		setPinOpenDrain(gpio, false);
		break;
	case PIN_AF1:


		if(PINTYPE_OC & gpio->af1){
			//it should be open collector
			setPinOpenDrain(gpio, true);
			setPinPullup(pin, PULL_UP);
		} else {
			setPinOpenDrain(gpio, false);
			setPinPullup(pin, PULL_NONE);
		}
		setPinRawAf(pin, (PINTYPE_OCMASK & gpio->af1));


		break;
	case PIN_AF2:
		if(PINTYPE_OC & gpio->af2){
			//it should be open collector
			setPinOpenDrain(gpio, true);
			setPinPullup(pin, PULL_UP);
		} else {
			setPinOpenDrain(gpio, false);
			setPinPullup(pin, PULL_NONE);
		}
		setPinRawAf(pin, (PINTYPE_OCMASK & gpio->af2));


		break;
	}

}


/**
 * sets the specified pin to alternate function mode and sets the AF register to the specified value.
 * Note that unless the comm protocol pin direction is set to NO_EFFECT, this will be overwritten.
 * @param pin the pin to set
 * @param the alternate function desired. See the STM32F446 Reference Manual for details
 */
void setPinRawAf(PortPin pin, uint8_t af){
	const GpioStruct* gpio = &(gpios[pin]);
	//set pin to AF mode
	uint32_t moder = gpio->port->MODER;
	uint32_t temp = moder ^ (GPIO_Mode_AF << (gpio->pinSource * 2));
	temp &= 0b11 << (gpio->pinSource * 2);


	if(temp != 0){
		moder ^= temp;
		gpio->port->MODER = moder;
	}

	//first check the state of the pin
	uint32_t afNow = gpio->port->AFR[gpio->pinSource >> 0x03];
	afNow >>= (gpio->pinSource & 0b111) * 4;
	afNow &= 0b1111;
	if(af != (uint8_t)afNow){
		//now set which AF to use because it's not right
		GPIO_PinAFConfig(gpio->port, gpio->pinSource, (0b1111 & af));
	}



}



