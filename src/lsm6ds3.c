/*
Copyright (c) 2018 STARFISH PRODUCT ENGINEERING INC.

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


#include <i2c.h>
#include "lsm6ds3.h"
#include "fastcodeUtil.h"


//********************************************************
//Defines
//********************************************************
#define PI 3.1415926535897932384626433832795028841
#define G 9.80665

#define ADDRESS 0b11010100

#define FUNC_CFG_ACCESS_REG 0x01 //don't think I need to write this
#define SENSOR_SYNC_TIME_FRAME_REG 0x04 //probably don't need this either
#define FIFO_CTRL1_REG 0x06 //assume for now I don't need fifo
#define FIFO_CTRL2_REG 0x07
#define FIFO_CTRL3_REG 0x08
#define FIFO_CTRL4_REG 0x09
#define FIFO_CTRL5_REG 0x0a
#define CTRL1_XL_REG 0x10 //set to 0b01010000
#define CTRL2_G_REG 0x11 //set to 0b01010000
#define CTRL3_C_REG 0x12 //set to 0b00000100 little endian
#define CTRL4_C_REG 0x13 //set to 0b0000000 or don't bother
#define CTRL5_C_REG 0x14 //set to 0x0 or don't bother
#define CTRL6_C_REG 0x15 //set to 0x0 or don't bother
#define CTRL7_G_REG 0x16 //set to 0x0 or don't bother
#define CTRL8_XL_REG 0x17 //not sure yet
#define CTRL9_XL_REG 0x18
#define CTRL10_C_REG 0x19
#define STATUS_REG_REG 0x1e
#define OUT_TEMP_L_REG 0x20
#define OUT_TEMP_H_REG 0x21
#define OUTX_L_G_REG 0x22
#define OUTX_H_G_REG 0x23
#define OUTY_L_G_REG 0x24
#define OUTY_H_G_REG 0x25
#define OUTZ_L_G_REG 0x26
#define OUTZ_H_G_REG 0x27
#define OUTX_L_XL_REG 0x28
#define OUTX_H_XL_REG 0x29
#define OUTY_L_XL_REG 0x2a
#define OUTY_H_XL_REG 0x2b
#define OUTZ_L_XL_REG 0x2c
#define OUTZ_H_XL_REG 0x2d

#define _REG 0x
#define _REG 0x



#define G_MAG_FULL_SCALE 2.0
#define DEGPS_MAG_FULL_SCALE 125.0f
#define DEG_C_OFFSET 25.0f
#define DEG_C_PER_BIT 62.5e-3f


#define MPSS_PER_BIT (G_MAG_FULL_SCALE * (G * 2.0 / 65535.0))
#define RAD_PER_SECOND_PER_BIT (DEGPS_MAG_FULL_SCALE * (PI / 180.0 * 2.0 / 65535.0))

#define NUM_BYTES_TO_READ 16
//********************************************************

//********************************************************
//variables
//********************************************************

TransactionStatus configStatus;
TransactionStatus readStatus;

static uint8_t readData[NUM_BYTES_TO_READ];
static uint8_t configTxData[] = {
		0x10,//select address 0x10
		0b0100000,//set ctrl1_xl reg
		0b0101000,//set ctrl2_G reg
		0b00000100//set ctrl3_c reg
};
static uint8_t readTxData[] = {0x1e};
static uint32_t m_timer = 0;

static uint8_t status = 0;
static float temperature = 0;
static float accX = 0;
static float accY = 0;
static float accZ = 0;
static float gyroX = 0;
static float gyroY = 0;
static float gyroZ = 0;
//********************************************************



//********************************************************
//Function Prototypes
//********************************************************

//********************************************************






void lsm6ds3SlowCode(float seconds){
	if(readStatus == TRANSACTION_COMPLETED_OK){
		readStatus = NO_TRANSACTION_YET;
		int i = 0;
		int16_t t = 0;
		//STATUS_REG_REG
		status = readData[i++];//0x1e
		//OUT_TEMP_L_REG
		//OUT_TEMP_H_REG
		t = readData[i++];;//0x1f dump the reserved byte


		t = readData[i++];//0x20
		t |= readData[i++] << 8;//0x21
		temperature = ((float)t)*DEG_C_PER_BIT + DEG_C_OFFSET;
		//OUTX_L_G_REG
		//OUTX_H_G_REG
		t = readData[i++];//0x22
		t |= readData[i++] << 8;//0x23
		gyroX = ((float)t)*RAD_PER_SECOND_PER_BIT;
		//OUTY_L_G_REG
		//OUTY_H_G_REG
		t = readData[i++];
		t |= readData[i++] << 8;
		gyroY = ((float)t)*RAD_PER_SECOND_PER_BIT;
		//OUTZ_L_G_REG
		//OUTZ_H_G_REG
		t = readData[i++];
		t |= readData[i++] << 8;
		gyroZ = ((float)t)*RAD_PER_SECOND_PER_BIT;
		//OUTX_L_XL_REG
		//OUTX_H_XL_REG
		t = readData[i++];
		t |= readData[i++] << 8;
		accX = ((float)t)*MPSS_PER_BIT;
		//OUTY_L_XL_REG
		//OUTY_H_XL_REG
		t = readData[i++];
		t |= readData[i++] << 8;
		accY = ((float)t)*MPSS_PER_BIT;
		//OUTZ_L_XL_REG
		//OUTZ_H_XL_REG
		t = readData[i++];
		t |= readData[i++] << 8;
		accZ = ((float)t)*MPSS_PER_BIT;

	}
	uint32_t t = (uint32_t)(seconds*SECONDS);

	if(slowTimer(&m_timer, t)){
		i2cQueue(I2C1_DEV, ADDRESS, configTxData, 4, (uint8_t*)0, 0, &configStatus);
		i2cQueue(I2C1_DEV, ADDRESS, readTxData, 1, readData,  NUM_BYTES_TO_READ, &readStatus);
	}
}
float getLsm6ds3AccX(void){
	return accX;
}
float getLsm6ds3AccY(void){
	return accY;
}
float getLsm6ds3AccZ(void){
	return accZ;
}
float getLsm6ds3GyroX(void){
	return gyroX;
}
float getLsm6ds3GyroY(void){
	return gyroY;
}
float getLsm6ds3GyroZ(void){
	return gyroZ;
}
