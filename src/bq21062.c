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
//this module handles I2C control of a bq21062 battery charge manager IC


//*************************************************
//includes
//*************************************************
#include "bq21062.h"
#include <i2c.h>
#include <fastcodeUtil.h>

//*************************************************
//defines
//*************************************************

#define I2C_ADDRESS 0xD6 //this is the address shifted up by 2


#define  REG_STAT0           0x0    //Charger Status 0                         READ ONLY
#define  REG_STAT1           0x1    //Charger Status 1                         READ ONLY
#define  REG_STAT2           0x2    //Status 2                                 READ ONLY
#define  REG_FLAG0           0x3    //Charger Flags 0                          READ ONLY - CLEAR ON READ
#define  REG_FLAG1           0x4    //Charger Flags 1                          READ ONLY - CLEAR ON READ
#define  REG_FLAG2           0x5    //Flags 2                                  READ ONLY - CLEAR ON READ
#define  REG_FLAG3           0x6    //Timer Flags                              READ ONLY - CLEAR ON READ
#define  REG_MASK0           0x7    //Interrupt Masks 0                        READ & WRITE
#define  REG_MASK1           0x8    //Interrupt Masks 1                        READ & WRITE
#define  REG_MASK2           0x9    //Interrupt Masks 2                        READ & WRITE
#define  REG_MASK3           0xA    //Interrupt Masks 3                        READ & WRITE
//note missing regs here 0xb - 0x11

#define  REG_VBAT_CTRL       0x12   //Battery Voltage Control                  READ & WRITE
#define  REG_ICHG_CTRL       0x13   //Fast Charge Current Control              READ & WRITE
#define  REG_PCHRGCTRL       0x14   //Pre-Charge Current Control               READ & WRITE
#define  REG_TERMCTRL        0x15   //Termination Current Control              READ & WRITE
#define  REG_BUVLO           0x16   //Battery UVLO and Current Limit Control   READ & WRITE
#define  REG_CHARGERCTRL0    0x17   //Charger Control 0                        READ & WRITE
#define  REG_CHARGERCTRL1    0x18   //Charger Control 1                        READ & WRITE
#define  REG_ILIMCTRL        0x19   //Input Corrent Limit Control              READ & WRITE
//note missing regs here 0x1a - 0x1c

#define  REG_LDOCTRL         0x1D   //LDO Control                              READ & WRITE
//note missing regs here 0x1e - 0x2f

#define  REG_MRCTRL          0x30   //MR Control                               READ & WRITE
//note missing regs here

#define  REG_ICCTRL0         0x35   //IC Control 0                             READ & WRITE
#define  REG_ICCTRL1         0x36   //IC Control 1                             READ & WRITE
#define  REG_ICCTRL2         0x37   //IC Control 2                             READ & WRITE
//note missing regs here

#define  REG_TS_READ         0x40   //TS Read                                  READ & WRITE
//note missing regs here

#define  REG_TS_FAULT_MEAS   0x58   //TS Fault Measurement                     READ & WRITE
//note missing regs here

#define  REG_TS_FASTCHGCTRL  0x61   //TS Charge Control                        READ & WRITE
#define  REG_TS_COLD         0x62   //TS Cold Threshold                        READ & WRITE
#define  REG_TS_COOL         0x63   //TS Cool Threshold                        READ & WRITE
#define  REG_TS_WARM         0x64   //TS Warm Threshold                        READ & WRITE
#define  REG_TS_HOT          0x65   //TS Hot Threshold                         READ & WRITE
//note missing regs here

#define  REG_DEVICE_ID       0x6F   //Device ID                                READ ONLY




#define PRE_CHARGE_CURRENT (10e-3f)
//*************************************************
//Types
//*************************************************
typedef enum {
	IDLE_STATE,
	READING_REGS_STATE,

} State;

typedef enum {
	WRITE_REG,
	READ_REG
} TypeOfTransaction;

typedef struct {
	uint8_t regAddress;
	TypeOfTransaction transactionType;
	uint8_t* regPointer;
} RegisterConfig;




//*************************************************
//Variables
//*************************************************
static uint8_t regStat0;
static uint8_t regStat1;
static uint8_t regStat2;
static uint8_t regFlag0;
static uint8_t regFlag1;
static uint8_t regFlag2;
static uint8_t regFlag3;
static uint8_t regMask0            = 0b00000000;
static uint8_t regMask1            = 0b00000000;
static uint8_t regMask2            = 0b00000000;
static uint8_t regMask3            = 0b00000000;
static uint8_t regVbatCtrl         = 0b00111100; //sets to 4.2V this happens to be the default
static uint8_t regIchgCtrl         =          0; //sets charge current. value & 1.25mA is the setpoint
static uint8_t regPchrgCtrl        = 0b00001000; //this will pre-charge at 10mA. 1.25mA * bottom 5 bits
static uint8_t regTermCtrl         = 0b00000000;
static uint8_t regBuvLo            = 0b00000000;
static uint8_t regChargerCtrl0     = 0b00000000;
static uint8_t regChargerCtrl1     = 0b00000000;
static uint8_t regIlimCtrl         = 0b00000000;
static uint8_t regLdoCtrl          = 0b00000000;
static uint8_t regMrCtrl           = 0b00000000;
static uint8_t regIcCtrl0          = 0b00000000;
static uint8_t regIcCtrl1          = 0b00000000;
static uint8_t regIcCtrl2          = 0b00000000;
static uint8_t regTsRead           = 0b00000000;
static uint8_t regTsFaultMeas      = 0b00000000;
static uint8_t regTsFastChgCtrl    = 0b00000000;
static uint8_t regTsCold           = 0b00000000;
static uint8_t regTsCool           = 0b00000000;
static uint8_t regTsWarm           = 0b00000000;
static uint8_t regTsHot            = 0b00000000;
static uint8_t regDeviceId         = 0b00000000;

static RegisterConfig regConfigs[] = {
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_STAT1         , READ_REG,  &regStat1           },
		{REG_STAT2         , READ_REG,  &regStat2           },
		{REG_FLAG0         , READ_REG,  &regFlag0           },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_FLAG1         , READ_REG,  &regFlag1           },
		{REG_FLAG2         , READ_REG,  &regFlag2           },
		{REG_FLAG3         , READ_REG,  &regFlag3           },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_MASK0         , WRITE_REG ,  &regMask0         },
		{REG_MASK1         , WRITE_REG ,  &regMask1         },
		{REG_MASK2         , WRITE_REG ,  &regMask2         },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_MASK3         , WRITE_REG ,  &regMask3         },
		{REG_VBAT_CTRL     , WRITE_REG ,  &regVbatCtrl      },
		{REG_PCHRGCTRL     , WRITE_REG ,  &regPchrgCtrl     },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_TERMCTRL      , WRITE_REG ,  &regTermCtrl      },
		{REG_BUVLO         , WRITE_REG ,  &regBuvLo         },
		{REG_CHARGERCTRL0  , WRITE_REG ,  &regChargerCtrl0  },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_CHARGERCTRL1  , WRITE_REG ,  &regChargerCtrl1  },
		{REG_ILIMCTRL      , WRITE_REG ,  &regIlimCtrl      },
		{REG_LDOCTRL       , WRITE_REG ,  &regLdoCtrl       },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_MRCTRL        , WRITE_REG ,  &regMrCtrl        },
		{REG_ICCTRL0       , WRITE_REG ,  &regIcCtrl0       },
		{REG_ICCTRL1       , WRITE_REG ,  &regIcCtrl1       },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_ICCTRL2       , WRITE_REG ,  &regIcCtrl2       },
		{REG_TS_READ       , WRITE_REG ,  &regTsRead        },
		{REG_TS_FAULT_MEAS , WRITE_REG ,  &regTsFaultMeas   },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_TS_FASTCHGCTRL, WRITE_REG ,  &regTsFastChgCtrl },
		{REG_TS_COLD       , WRITE_REG ,  &regTsCold        },
		{REG_TS_COOL       , WRITE_REG ,  &regTsCool        },
		{REG_ICHG_CTRL     , WRITE_REG ,  &regIchgCtrl      },//scatter duplicates of this reg write around so it happens more frequently
		{REG_STAT0         , READ_REG,  &regStat0           },//ditto
		{REG_TS_WARM       , WRITE_REG ,  &regTsWarm        },
		{REG_TS_HOT        , WRITE_REG ,  &regTsHot         },
		{REG_DEVICE_ID     , READ_REG,  &regDeviceId        },

};

static uint32_t transactionTimer = 0;
static uint32_t registerIndex = 0;

static I2cDev m_dev = I2CNULL_DEV;
static uint8_t m_txData[2];
static TransactionStatus m_status = NULL_TRANSACTION;
static uint32_t m_badCommsCount = 0;
static bool m_updated = false;

//*************************************************
//function prototypes
//*************************************************

static uint32_t float2Int(float v);
/**
 * this will check all the read in values to see what's going on
 */
static void checkRegs(void);

//*************************************************
//code
//*************************************************

/**
 * must be run periodically to ensure proper update and control of charger chip
 */
void bq20162SlowCode(void){
	//only do stuff if this is configured
	if(m_dev != I2CNULL_DEV){
		if(slowTimer(&transactionTimer, 50000)){
			//do this every 100ms

			//first bounds check the index
			if(registerIndex >= (sizeof(regConfigs) / sizeof(regConfigs[0]))){//the sizeof stuff should return the number of elements in the array
				registerIndex = 0;
				checkRegs();
			}

			//get info on the register we're about to deal with
			RegisterConfig* rc = &(regConfigs[registerIndex]);

			switch(m_status){
			case NULL_TRANSACTION:
			case NO_TRANSACTION_YET:
			case TRANSACTION_QUEUED:
			case TRANSACTION_IN_PROGRESS:
			case TRANSACTION_COMPLETED_OK:
				break;
			case TRANSACTION_TIMED_OUT:
			case TRANSACTION_QUEUE_FULL:
			case TRANSACTION_TX_ADDRESS_NACK:
			case TRANSACTION_RX_ADDRESS_NACK:
			case TRANSACTION_TX_DATA_NACK:
			case TRANSACTION_RX_DATA_NACK:
				++m_badCommsCount;
				break;
			}
			//queue the next transaction if we're able

			if(rc->transactionType == WRITE_REG){
				m_txData[0] = rc->regAddress;
				m_txData[1] = *(rc->regPointer);

				i2cQueue(m_dev, I2C_ADDRESS, m_txData, 2, 0, 0, &m_status);
			} else {
				m_txData[0] = rc->regAddress;

				i2cQueue(m_dev, I2C_ADDRESS, m_txData, 1, rc->regPointer, 1, &m_status);
			}
			++registerIndex;
		}


	}

}

/**
 * must be called at startup to init this module
 */
void bq20162Init(){
	//TODO: add code if necessary

}



/**
 * indicates if the battery is being charged
 */
bool isBq21062Charging(void){
	bool powerGood = (regStat0 & (1<<0)) != 0;

	return powerGood & !isBq21062FullyCharged();
	}
/**
 * indicates whether the battery is in a full charge state or not
 */
bool isBq21062FullyCharged(void){
	return (regStat0 & (1 << 5)) != 0;
}

void bq20162Config(I2cDev i, float chargeCurrentAmps, float batteryVoltage){


	m_dev = i;
	float icharge = chargeCurrentAmps > 500e-3 ? 500e-3 : chargeCurrentAmps;
	regIchgCtrl         = 0b11111111 & (uint8_t)float2Int(icharge * (1.0f/2.5e-3));//peak current
	regVbatCtrl         = (uint8_t)float2Int((batteryVoltage - 3.6f)*(1.0f/10e-3));//battery voltage
	regPchrgCtrl        = 0b10000000 | (uint8_t)float2Int(PRE_CHARGE_CURRENT *(1.0f/2.5e-3));//precharge current
	regTermCtrl         = 0b00010100;//this means 10% of charge current
	regBuvLo            = 0b00000011;//3V fast charge thresh, 1200mA overcurrent thresh, 2.8V undervoltage lockout
	regChargerCtrl0     = 0b10110000;//thermistor function enabled, JEITA control mode, 200mV recharge thresh, watchdog disabled, 2X safety timer disabled, 3 hour safety timer
	regChargerCtrl1     = 0b00111000;//VINDPM enabled, 4.5V, 80C current foldback threshold
	regIlimCtrl         = 0b00000111;//600mA input current limit
	regLdoCtrl          = 0b00110010;//LDO disabled. This is reset value.
	regMrCtrl           = 0b00101110;//This is reset value.
	regIcCtrl0          = 0b00100000;//This is reset value.
	regIcCtrl1          = 0b00000000;//This is reset value.
	regIcCtrl2          = 0b00010010;//this is mostly the reset value except: PG hiZ, watchdog enabled
	regTsRead           = 0b10000010;//this is mostly default except: TS checked each second
	regTsFaultMeas      = 0b00000100;//enable TS fault measurement
	regTsFastChgCtrl    = 0b00110100;//reset value
	regTsCold           = 0b01111100;//reset value
	regTsCool           = 0b01101101;//reset value
	regTsWarm           = 0b00111000;//reset value
	regTsHot            = 0b00111000;//reset value

}


static uint32_t float2Int(float v){
	float result = v;

	if(result < 0.0f){
		result = 0.0f;
	}

	result += 0.5f;//this will cause it to round.

	return (uint32_t)result;
}
/**
 * this will check all the read in values to see what's going on
 */
static void checkRegs(void){
	m_updated = true;
	//TODO: check stuff

}

/**
 * indicates if the data of this chip has updated since last query
 * This will clear its state after being queries so an immediately following call will always return false
 */
bool isBq20162Updated(void){
	bool result = m_updated;
	if(result){
		m_updated = false;
	}
	return result;
}
