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

#include <math.h>
#include "temperatureControl.h"






/**
 * calcs the new setpoints for the fan and the heater
 * implements PI control of the form:
 * A + B/s
 * this can be written:
 * (A*s + B)/s
 * and
 * A*(s + B/A)/s
 * which leads to coefficients of:
 * gain = A
 * tc = B/A
 *
 * so
 * A = gain
 * B = tc*gain
 *
 *
 */
void calcPIDOutput(TempControlState* state, float temperature, float deltaTime){
	state->deltaTime = deltaTime;
	float deriv = (temperature - state->temperature)/deltaTime;

	deriv -= state->derivative;
	deriv *= deltaTime;
	deriv /= state->derivativeFilterTc;
	state->derivative += deriv;

	state->temperature = temperature;
	float error = state->setpoint - temperature;
	if(!finite(error)){
		error = 0;
	}
	float a = state->pCoeff;
	float b = state->iCoeff;
//	float c = state->dCoeff;
	float max = state->maxPwm;

	//note that the integral includes the gain

	float integral = state->integral;
	integral += b*deltaTime*error;

	//zero the integral if the gain term is zero
	if(b == 0){
		integral = 0.0;
	}
	//ensure integral is finite. If not, zero it
	if(!finite(integral)){
		integral = 0.0f;
	}


	float out = a*error + integral;

	//if we're within bounds then keep the result, otherwise fix it
	//ensure integral only grows as bit as it can with the output within bounds
	if(out > max){
		integral -= out - max;
		out = max;
	} else if(out < 0.0){
		integral -= out;
		out = 0.0;
	}


	state->integral = integral;



	state->heaterPwm = out;
	//now compute fan
	out = 0.2f - out;
	out *= 5;

	state->fanPwm = out;
}



