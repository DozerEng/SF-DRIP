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
//includes
//*************************************************

#include "bq25703A.h"
#include "fastcodeUtil.h"


//*************************************************
//defines
//*************************************************
#define I2C_ADDRESS 0xd6 //it could also be 0xd6 if it's shifted up, or 0x6b or perhaps 0b1101101X where X is read/write

//I2C register addresses
#define CHARGE_OPTION_0_L     0x00
#define CHARGE_OPTION_0_H     0x01
#define CHARGE_CURRENT_L      0x02
#define CHARGE_CURRENT_H      0x03
#define MAX_CHARGE_VOLTAGE_L  0x04
#define MAX_CHARGE_VOLTAGE_H  0x05
#define OTG_VOLTAGE_L         0x06
#define OTG_VOLTAGE_H         0x07
#define OTG_CURRENT_L         0x08
#define OTG_CURRENT_H         0x09
#define INPUT_VOLTAGE_L       0x0a
#define INPUT_VOLTAGE_H       0x0b
#define MIN_SYSTEM_VOLTAGE_L  0x0c
#define MIN_SYSTEM_VOLTAGE_H  0x0d
#define IIN_HOST_L            0x0e
#define IIN_HOST_H            0x0f

#define CHARGER_STATUS_L      0x20
#define CHARGER_STATUS_H      0x21
#define PROCHOT_STATUS_L      0x22
#define PROCHOT_STATUS_H      0x23
#define IIN_DPM_L             0x24
#define IIN_DPM_H             0x25
#define ADC_VBUS_PSYS_L       0x26
#define ADC_VBUS_PSYS_H       0x27
#define ADC_IBAT_L            0x28
#define ADC_IBAT_H            0x29
#define ADC_IINCM_PIN_L       0x2a
#define ADC_IINCM_PIN_H       0x2b
#define ADC_VSYS_BAT_L        0x2c
#define ADC_VSYS_BAT_H        0x2d
#define MANUFACTURE_ID        0x2e
#define DEVICE_ID             0x2f

#define CHARGE_OPTION_1_L     0x30
#define CHARGE_OPTION_1_H     0x31
#define CHARGE_OPTION_2_L     0x32
#define CHARGE_OPTION_2_H     0x33
#define CHARGE_OPTION_3_L     0x34
#define CHARGE_OPTION_3_H     0x35
#define PROCHOT_OPTION_0_L    0x36
#define PROCHOT_OPTION_0_H    0x37
#define PROCHOT_OPTION_1_L    0x38
#define PROCHOT_OPTION_1_H    0x39
#define ADC_OPTION_L          0x3a
#define ADC_OPTION_H          0x3b


#define EN_LWPWR_MASK         (1<<7) //OPTION_0_H
#define EN_LDO_MASK           (1<<2) //OPTION_0_L
#define EN_OTG_MASK           (1<<4) //OPTION_3_H
#define EN_ICO_MODE_MASK      (1<3)  //OPTION_3_H


#define RPSYS (30.0e3f) //psys resistance
#define PSYS_W_PER_V (33.333333333f) //this is 1/1e-6V/W/30k

//*************************************************
//Types
//*************************************************

typedef enum {
	IDLE_STATE,//nothing happening. Waiting for time to initate comms to chip
	CONFIG_1_STATE,//waiting for completion of I2C transaction sending config 1 stuff
	CONFIG_2_STATE,//waiting for completion of I2C transaction sending config 2 stuff
	READ_VALS_STATE,//waiting for completion of I2C transaction reading values from IC
	READ_SETTINGS_STATE,//waiting for completion of I2C transaction reading the 0x01 - 0x0f registers
	WRITE_CURRENT_STATE,//a debug state that only writes the charge current
	CLEAR_OVER_VOLTAGE_ERROR_STATE,
	ERROR_STATE,
} I2CState;

typedef enum {
	PRECHARGE_STATE,
	CONSTANT_CURRENT_STATE,
	CONSTANT_VOLTAGE_STATE,
	CHARING_COMPLETE_STATE

} ChargeState;

//*************************************************
//Variables
//*************************************************
static uint8_t txData[20];
static uint8_t readData[16];
static uint8_t readSettingsData[64];
static uint8_t readSettingsData2[6];

static TransactionStatus i2cStatus;
static I2cDev i2cDev = I2C1_DEV;
static float chargeVoltageSetpoint = 16.8f;
static float chargeCurrentSetpoint = 1.0f;
static float otgVoltageLimit = 12.0f;
static float otgCurrentLimit = 2.0f;
static float inputCurrentLimit = 5.0f;
static float inputVoltageLimit = 11.0f;//when the input sags below this limit then it enters digital power management mode
static float systemVoltageLimit = 12.00f;//minimum battery voltage that the system will get (I think that's what it means)
static bool lowPowerEnable = false;
static bool ldoEnable = false;
static bool otgEnable = false;
static bool icoModeEnable = false;
static I2CState state = IDLE_STATE;
//added volatile so these variables would not get optimized out.
//May be able to remove this later when I actually do something with these values
volatile static uint8_t chargerStatusH;
volatile static uint8_t chargerStatusL;
volatile static uint8_t procHotStatusH;
volatile static float inputCurrentDpmActual;
volatile static float systemPowerActual;
volatile static float inputVoltageActual;
volatile static float chargeCurrentActual;
volatile static float dischargeCurrentActual;
volatile static float inputCurrentActual;
volatile static float cmpInVoltageActual;
volatile static float systemVoltageActual;
volatile static float batteryVoltageActual;
volatile static uint8_t mfgId;
volatile static uint8_t deviceId;
static uint32_t pollingTimerReg = 0;
volatile static bool lastQueueOk = true;





//*************************************************
//function prototypes
//*************************************************

static void setupConfig1(void);
static void setupConfig2(void);
static void setupRead(void);
static void setupReadSettings(void);
static void setState(I2CState s);
static void decodeVals(uint8_t* readData);

static bool isIcoDone(void);
static bool isInPrecharge(void);
static bool isInOtg(void);
static bool isAcOverVoltage(void);
static bool isBatteryOverCurrent(void);
static bool isAcOverCurrent(void);
static bool isSysOverVoltageProtectionOn(void);
static bool isOtgOverCurrentProtectionOn(void);
static bool isOtgOverVoltageProtectionOn(void);





//*************************************************
//code
//*************************************************


void bq25703ASlowCode(I2cDev dev){
	static bool startup = true;
	i2cDev = dev;
	switch(state){
	case IDLE_STATE:
		if(slowTimer(&pollingTimerReg, startup ? 1*SECONDS : 1*SECONDS)){//500*MILLISECONDS)){//1*SECONDS)){
			if(startup){
				setState(CONFIG_1_STATE);
			} else {
				setState(CONFIG_1_STATE);
				//setState(READ_SETTINGS_STATE);
			}
			startup = false;
//			setState(WRITE_CURRENT_STATE);
		}
		break;
	case WRITE_CURRENT_STATE:
		if(i2cStatus == TRANSACTION_COMPLETED_OK){
			//cascade into next state
//			setState(CONFIG_1_STATE);
			setState(READ_SETTINGS_STATE);
		} else if(i2cStatus != TRANSACTION_IN_PROGRESS && i2cStatus != TRANSACTION_QUEUED){
			setState(IDLE_STATE);
		}
		break;
	case CONFIG_1_STATE:
		if(i2cStatus == TRANSACTION_COMPLETED_OK){
			//cascade into next state
			setState(CONFIG_2_STATE);
		} else if(i2cStatus != TRANSACTION_IN_PROGRESS && i2cStatus != TRANSACTION_QUEUED){
			setState(IDLE_STATE);
		}
		break;
	case CONFIG_2_STATE:
		if(i2cStatus == TRANSACTION_COMPLETED_OK){
			//cascade into next state
//			setState(READ_VALS_STATE);
			setState(READ_SETTINGS_STATE);
		} else if(i2cStatus != TRANSACTION_IN_PROGRESS && i2cStatus != TRANSACTION_QUEUED){
			setState(IDLE_STATE);
		}

		break;
	case READ_SETTINGS_STATE:
		if(i2cStatus == TRANSACTION_COMPLETED_OK){
			decodeVals((uint8_t*)(&readSettingsData[0x20]));
			if(isSysOverVoltageProtectionOn()){
				setState(CLEAR_OVER_VOLTAGE_ERROR_STATE);
			} else {
				//cascade into next state
				setState(IDLE_STATE);
			}


		} else if(i2cStatus != TRANSACTION_IN_PROGRESS && i2cStatus != TRANSACTION_QUEUED){
			setState(IDLE_STATE);
		}
		break;
	case READ_VALS_STATE:
		if(i2cStatus == TRANSACTION_COMPLETED_OK){
			decodeVals((uint8_t*)readData);
			//cascade into next state
//			setState(IDLE_STATE);
			setState(READ_SETTINGS_STATE);
		} else if(i2cStatus != TRANSACTION_IN_PROGRESS && i2cStatus != TRANSACTION_QUEUED){
			setState(IDLE_STATE);
		}
		break;
	case CLEAR_OVER_VOLTAGE_ERROR_STATE:
		if(i2cStatus == TRANSACTION_COMPLETED_OK){
			setState(IDLE_STATE);
		} else if(i2cStatus != TRANSACTION_IN_PROGRESS && i2cStatus != TRANSACTION_QUEUED){
			setState(IDLE_STATE);
		}

		break;
	case ERROR_STATE:
		break;
	}

}

static void setState(I2CState s){
	uint32_t temp;
	switch(state){
	case IDLE_STATE:
		break;
	case WRITE_CURRENT_STATE:
		txData[0] = CHARGE_CURRENT_L; //address of first byte to write




		//charge current
		temp = (uint32_t)((chargeCurrentSetpoint)*15.625f);//this converts it to int with 64mA resolution
		temp &= 0x7f;
		temp <<= 6;//left shift according to datasheet
		txData[1] = (uint8_t)((temp >> 0) & 0xff); //CHARGE_CURRENT_L
		txData[2] = (uint8_t)((temp >> 8) & 0xff); //CHARGE_CURRENT_H
		lastQueueOk = i2cQueue(i2cDev, I2C_ADDRESS, txData, 3, (uint8_t*)readSettingsData2, 6, &i2cStatus);
		break;
	case CONFIG_1_STATE:
		setupConfig1();
		break;
	case CONFIG_2_STATE:
		setupConfig2();
		break;
	case READ_VALS_STATE:
		setupRead();
		break;
	case CLEAR_OVER_VOLTAGE_ERROR_STATE:
		txData[0] = CHARGER_STATUS_L; //address of first byte to write
		txData[1] = 0;//low byte
		txData[2] = 0;//high byte
		lastQueueOk = i2cQueue(i2cDev, I2C_ADDRESS, txData, 3, (uint8_t*)0, 0, &i2cStatus);
		break;
	case READ_SETTINGS_STATE:
		setupReadSettings();
		break;
	case ERROR_STATE:
		break;
	}
	state = s;
}

/**
 * sends the first batch of configuration data to the chip
 */
static void setupConfig1(void){
	txData[0] = CHARGE_OPTION_0_L; //address of first byte to write
	txData[1] = 0b00001110 | (ldoEnable ? EN_LDO_MASK : 0); //CHARGE_OPTION_0_L
	txData[2] = 0b00000110; //CHARGE_OPTION_0_H 5 second WDT

	uint32_t temp;

	//charge current
	temp = (uint32_t)((chargeCurrentSetpoint)*15.625f);//this converts it to int with 64mA resolution
	temp &= 0x7f;
	temp <<= 6;//left shift according to datasheet
	txData[3] = (uint8_t)((temp >> 0) & 0xff); //CHARGE_CURRENT_L
	txData[4] = (uint8_t)((temp >> 8) & 0xff); //CHARGE_CURRENT_H

	temp = (uint32_t)((chargeVoltageSetpoint)*62.5f);//this converts it to int with 16mV resolution
	temp &= 0x7ff;
	temp <<= 4;//shift left according to what registers expect
	txData[5] = (uint8_t)((temp >> 0) & 0xff); //MAX_CHARGE_VOLTAGE_L
	txData[6] = (uint8_t)((temp >> 8) & 0xff); //MAX_CHARGE_VOLTAGE_H

	temp = (uint32_t)((otgVoltageLimit - 4.480)*15.625);//this converts it to int with 64mV resolution
	temp &= 0xff;
	temp <<= 6;
	txData[7] = (uint8_t)((temp >> 0) & 0xff); //OTG_VOLTAGE_L
	txData[8] = (uint8_t)((temp >> 8) & 0xff); //OTG_VOLTAGE_H

	temp = (uint32_t)(otgCurrentLimit*20);//this converts it to int with 64mV of OTG voltage
	temp &= 0x7f;
	temp <<= 8;
	txData[9] =  (uint8_t)((temp >> 0) & 0xff); //OTG_CURRENT_L
	txData[10] = (uint8_t)((temp >> 8) & 0xff); //OTG_CURRENT_H

	temp = (uint32_t)((inputVoltageLimit - 3.2)*15.625);//this converts to int with 64mV resolution
	temp &= 0xff;
	temp <<= 6;
	txData[11] = (uint8_t)((temp >> 0) & 0xff); //INPUT_VOLTAGE_L
	txData[12] = (uint8_t)((temp >> 8) & 0xff); //INPUT_VOLTAGE_H


	temp = (uint32_t)((systemVoltageLimit)*3.90625);//this converts to int with 256mV resolution
	temp &= 0x3f;
	temp <<= 8;
	txData[13] = (uint8_t)((temp >> 0) & 0xff); //MIN_SYSTEM_VOLTAGE_L
	txData[14] = (uint8_t)((temp >> 8) & 0xff); //MIN_SYSTEM_VOLTAGE_H

	temp = (uint32_t)((inputCurrentLimit - 50e-3)*20);//this converst to int with 50mA resolution
	temp &= 0x7f;
	temp <<= 8;
	txData[15] = (uint8_t)((temp >> 0) & 0xff); //IIN_HOST_L
	txData[16] = (uint8_t)((temp >> 8) & 0xff);  //IIN_HOST_H

	lastQueueOk = i2cQueue(i2cDev, I2C_ADDRESS, txData, 17, (uint8_t*)0, 0, &i2cStatus);
}
/**
 * sends the second batch of configuration data to the chip
 */
static void setupConfig2(void){
	uint32_t i = 0;
	txData[i++] = CHARGE_OPTION_1_L; //address of first byte to write
	txData[i++] = 0b00000000; //CHARGE_OPTION_1_L
	txData[i++] = 0b00000010; //CHARGE_OPTION_1_H
	txData[i++] = 0b00110101; //CHARGE_OPTION_2_L
	txData[i++] = 0b11000000; //CHARGE_OPTION_2_H
	txData[i++] = 0b00000000; //CHARGE_OPTION_3_L
	txData[i++] = 0b00000000 | (otgEnable ? EN_OTG_MASK : 0) | (icoModeEnable ? EN_ICO_MODE_MASK : 0); //CHARGE_OPTION_3_H
	txData[i++] = 0b01010100; //PROCHOT_OPTION_0_L
	txData[i++] = 0b01100110; //PROCHOT_OPTION_0_H address 0x37
	txData[i++] = 0b01111111; //PROCHOT_OPTION_1_L
	txData[i++] = 0b00101001; //PROCHOT_OPTION_1_H 5A warning threshold
	txData[i++] = 0b11111111; //ADC_OPTION_L
	txData[i++] = 0b10100000;  //ADC_OPTION_H
	lastQueueOk = i2cQueue(i2cDev, I2C_ADDRESS, txData, i, (uint8_t*)0, 0, &i2cStatus);
}
/**
 * triggers a read from the main status regs on the chip
 */
static void setupRead(void){
	txData[0] = CHARGER_STATUS_L; //address of first byte to write
	lastQueueOk = i2cQueue(i2cDev, I2C_ADDRESS, txData, 1, (uint8_t*)readData, 16, &i2cStatus);
}
/**
 * triggers a read from the settings regs 0x00 to 0x0f
 */
static void setupReadSettings(void){
	txData[0] = CHARGE_OPTION_0_L; //address of first byte to write
	lastQueueOk = i2cQueue(i2cDev, I2C_ADDRESS, txData, 1, (uint8_t*)readSettingsData, 64, &i2cStatus);
}


//void bq25703AInit(I2cDev dev){
//	i2cDev = dev;
//}



static void decodeVals(uint8_t* readData){
	chargerStatusL = (uint8_t)(readData[0x0]);
	chargerStatusH = (uint8_t)(readData[0x1]);
	procHotStatusH = (uint8_t)(readData[0x2]);
	//byte 3 has no content
	//byte 4 is not used
	inputCurrentDpmActual = (float)(readData[0x5] & 0x7f)*50e-3 + 50e-3;
	systemPowerActual = (float)(readData[0x6])*12e-3*PSYS_W_PER_V;
	inputVoltageActual = (float)(readData[0x7])*64e-3 + 3.2;
	dischargeCurrentActual = (float)(readData[0x8] & 0x7f)*256e-3;
	chargeCurrentActual = (float)(readData[0x9] & 0x7f)*64e-3;
	cmpInVoltageActual = (float)(readData[0xa])*12e-3;
	inputCurrentActual = (float)(readData[0xb])*50e-3;
	batteryVoltageActual = (float)(readData[0xc])*64e-3 + 2.88f;
	systemVoltageActual = (float)(readData[0xd])*64e-3 + 2.88f;
	mfgId = readData[0xe];
	deviceId = readData[0xf];


}

bool isInputPresent(void){
	return (chargerStatusH & (1<<7)) != 0;
}
static bool isIcoDone(void){
	return (chargerStatusH & (1<<6)) != 0;
}
static bool isInPrecharge(void){
	return (chargerStatusH & (1<<1)) != 0;
}
static bool isInOtg(void){
	return (chargerStatusH & (1<<0)) != 0;
}
static bool isAcOverVoltage(void){
	return (chargerStatusL & (1<<7)) != 0;
}
static bool isBatteryOverCurrent(void){
	return (chargerStatusL & (1<<6)) != 0;
}
static bool isAcOverCurrent(void){
	return (chargerStatusL & (1<<5)) != 0;
}
static bool isSysOverVoltageProtectionOn(void){
	return (chargerStatusL & (1<<4)) != 0;
}
static bool isOtgOverVoltageProtectionOn(void){
	return (chargerStatusL & (1<<1)) != 0;
}
static bool isOtgOverCurrentProtectionOn(void){
	return (chargerStatusL & (1<<0)) != 0;
}


void setCharge(float cellVolts, float amps, float ampsLimit){
	chargeVoltageSetpoint = cellVolts;
	chargeCurrentSetpoint = amps;

}
void setOtg(float volts, float ampLimit){

}
bool isChargeComplete(void){
	return false;
}
float getBatteryCurrent(void){
	float result = chargeCurrentActual > 0 ? chargeCurrentActual : -dischargeCurrentActual;
	return result;
}
float getBatteryVoltage(void){
	return batteryVoltageActual;
}
float getInputVoltage(void){
	return inputVoltageActual;
}
float getSystemVoltage(void){
	return systemVoltageActual;
}

bool isCharging(void){
	return (chargerStatusH & 0b00001110) != 0;
}
uint32_t getChargeStatusBits(void){
	uint32_t result = 0;
	result |= procHotStatusH << 16;
	result |= chargerStatusH << 8;
	result |= chargerStatusL;

	return result;

}
