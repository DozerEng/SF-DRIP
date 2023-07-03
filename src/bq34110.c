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


//*************************************************
//defines
//*************************************************

#define I2C_ADDRESS 0xAA //this is the address shifted up by 2

#define CNTL_REG_L                   0x00
#define CNTL_REG_H                   0x01
#define TEMP_REG_L                  0x06
#define TEMP_REG_H                  0x07
#define VOLT_REG_L                  0x08
#define VOLT_REG_H                  0x09
#define BSTAT_REG_L                 0x0a
#define BSTAT_REG_H                 0x0b
#define CURRENT_REG_L               0x0c
#define CURRENT_REG_H               0x0d
#define REMAINING_CAPACITY_REG_L    0x10
#define REMAINING_CAPACITY_REG_H    0x11
#define FULL_CHARGE_CAPACITY_REG_L  0x12
#define FULL_CHARGE_CAPACITY_REG_H  0x13
#define AVERAGE_CURRENT_REG_L       0x14
#define AVERAGE_CURRENT_REG_H       0x15
#define TIME_TO_EMPTY_REG_L         0x16
#define TIME_TO_EMPTY_REG_H         0x17
#define TIME_TO_FULL_REG_L          0x18
#define TIME_TO_FULL_REG_H          0x19
#define ACC_CHARGE_REG_L            0x1a
#define ACC_CHARGE_REG_H            0x1b
#define ACC_TIME_REG_L              0x1c
#define ACC_TIME_REG_H              0x1d
#define LAST_ACC_CHARGE_REG_L       0x1e
#define LAST_ACC_CHARGE_REG_H       0x1f
#define LAST_ACC_TIME_REG_L         0x20
#define LAST_ACC_TIME_REG_H         0x21
#define AVG_POWER_REG_L             0x24
#define AVG_POWER_REG_H             0x25
#define INT_TEMP_REG_L              0x28
#define INT_TEMP_REG_H              0x29
#define CYLCE_COUNT_REG_L           0x2a
#define CYLCE_COUNT_REG_H           0x2b
#define RSOC_REG_L                  0x2c
#define RSOC_REG_H                  0x2d
#define SOH_REG_L                   0x2e
#define SOH_REG_H                   0x2f
#define CV_REG_L                    0x30
#define CV_REG_H                    0x31
#define CC_REG_L                    0x32
#define CC_REG_H                    0x33
#define BLT_DISCHARGE_SET_REG_L     0x34
#define BLT_DISCHARGE_SET_REG_H     0x35
#define BLT_CHARGE_SET_REG_L        0x36
#define BLT_CHARGE_SET_REG_H        0x37
#define STATUS_REG_L                0x3a
#define STATUS_REG_H                0x3b
#define DESIGN_CAPACITY_REG_L       0x3c
#define DESIGN_CAPACITY_REG_H       0x3d
#define MAC_REG_L                   0x3e
#define MAC_REG_H                   0x3f
#define MAC_DATA_REG                0x40
#define MAC_DATA_SUM_REG            0x60
#define MAC_DATA_LEN_REG            0x61
#define EOS_LEARN_STATUS_REG_L      0x64
#define EOS_LEARN_STATUS_REG_H      0x65
#define EOS_SAFETY_STATUS_REG_L     0x66
#define EOS_SAFETY_STATUS_REG_H     0x67
#define EOS_STATUS_REG              0x68
#define ANALOG_COUNT_REG            0x79
#define RAW_CURRENT_REG_L           0x7a
#define RAW_CURRENT_REG_H           0x7b
#define RAW_VOLTAGE_REG_L           0x7c
#define RAW_VOLTAGE_REG_H           0x7d
#define RAW_INT_TEMP_REG_L          0x7e
#define RAW_INT_TEMP_REG_H          0x7f
#define RAW_EXT_TEMP_REG_L          0x80
#define RAW_EXT_TEMP_REG_H          0x81


//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************


//*************************************************
//code
//*************************************************

