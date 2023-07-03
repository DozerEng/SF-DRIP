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

#ifndef SIGGEN_H_
#define SIGGEN_H_

#include <stdint.h>
#include <stdbool.h>
#include "powerOutputs.h"
#include "temperatureControl.h"

typedef enum {
	OFF_SIG = 0,
	DC_SIG = 1,
	SINE_SIG = 2,
	SQUARE_SIG = 3,
	TRIANGLE_SIG = 4,
	STEPPER_SIG = 5,
	COMPLEMENT_SIG = 6,
	THERMAL_SIG = 7,
	BRUSHLESS_MOTOR_SIG = 8,
	FEEDBACK_SIG = 9,
} SigType;

void sigGenConfig(OutputChannel chan, SigType type, float freqHz, float ampFrac);
void setSigGenType(OutputChannel chan, SigType type);
SigType getSigGenType(OutputChannel chan);
void sigGenInit();
void sigGenFastCode();
TempControlState* getTempControlState(OutputChannel chan);
float* getSigGenOutputs();
SigType lookupType(uint32_t i);
//bool isSigGenEnabled(void);
//void sigGenDisable(void);
void sigGenTemperatureControl(OutputChannel chan, uint32_t thermistorChan, float setpoint, float pCoeff, float iCoeff, float dCoeff, float maxPwm);
/**
 * Computes a triangle wave signal. A single cycle spans 2*pi radians.
 * @param radians the input phase
 * @return the resulting value spanning 0.0f to 1.0f
 */
float getTriangleWave(float radians);
/**
 * Computes a square wave signal. A single cycle spans 2*pi radians.
 * @param radians the input phase
 * @return the resulting value spanning 0.0f to 1.0f
 */
float getSquareWave(float radians);
/**
 * Computes a sine wave signal. A single cycle spans 2*pi radians.
 * This is based on a 360 element lookup table
 * @param radians the input phase
 * @return the resulting value spanning 0.0f to 1.0f
 */
float getSineWave(float radians);
#endif /* SIGGEN_H_ */
