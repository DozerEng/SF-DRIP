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

#ifndef INC_BQ25703A_H_
#define INC_BQ25703A_H_

#include "stdint.h"
#include "stdbool.h"
#include "i2c.h"



//void bq25703AInit(I2cDev dev);
void bq25703ASlowCode(I2cDev dev);

/**
 * Sets the charger either in lower power mode or "performance" mode.
 * @param e if true then chip set to low power mode
 */
void setLowPowerMode(bool e);


void setCharge(float cellVolts, float amps, float ampsLimit);
void setOtg(float volts, float ampLimit);
bool isChargeComplete(void);
/**
 * @return the current in (positive) or out (negative) of the battery
 */
float getBatteryCurrent(void);
float getBatteryVoltage(void);
float getInputVoltage(void);
float getSystemVoltage(void);
//bool isAcConnected(void);

bool isInputPresent(void);
bool isCharging(void);
/**
 * @return byte packed: procHotStatus, chargeStatusH, chargeStatusL
 */
uint32_t getChargeStatusBits(void);

#endif /* INC_BQ25703A_H_ */
