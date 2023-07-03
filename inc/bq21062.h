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

#ifndef INC_BQ21062_H_
#define INC_BQ21062_H_

#include <i2c.h>
#include <stdbool.h>


/**
 * must be run periodically to ensure proper update and control of charger chip
 */
void bq20162SlowCode();

/**
 * must be called at startup to init this module
 */
void bq20162Init();



void bq20162Config(I2cDev i, float chargeCurrentAmps, float batteryVoltage);

/**
 * indicates if the data of this chip has updated since last query
 * This will clear its state after being queries so an immediately following call will always return false
 */
bool isBq20162Updated(void);

/**
 * indicates if the battery is being charged
 */
bool isBq21062Charging(void);

/**
 * indicates whether the battery is in a full charge state or not
 */
bool isBq21062FullyCharged(void);

#endif /* INC_BQ21062_H_ */
