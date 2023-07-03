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

#ifndef INC_THERMISTORS_H_
#define INC_THERMISTORS_H_

#include <stdint.h>

/**
 * must be called before using this module.
 * This computes the lookup tables
 */
void initThermistors(void);

/**
 * Sets up to compute the temperature based on a modified form the the Steinhart-Hart equation.
 * 1/T = c0 + c1*ln(Rt/R0) + c2*ln(Rt/R0)^2 + c3*ln(Rt/R0)^3
 * @param index - the temperature channel that's being configured. Can be from 0 to 7
 * @param adcChannel - the adc channel that will be used as an input
 * @param C0 - the offset coefficient of the Steinhart-Hart equation
 * @param C1 - the linear coefficient of the S-H equation
 * @param C2 - the quadratic coefficient of the S-H equation
 * @param C3 - the cubic coefficient of the S-H equation
 */
void setupThermistor(uint8_t index, uint8_t adcChannel, float c0, float c1, float c2, float c3);
/**
 * @return the temperature of the specified channel in Celsius
 */
float getThermistorTemperature(uint8_t index);
uint32_t getThermistorCount(void);
#endif /* INC_THERMISTORS_H_ */
