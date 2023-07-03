/*
Copyright (c) 2017-2021 STARFISH PRODUCT ENGINEERING INC.

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

#include "filter.h"
//#include "float.h"

/**
 * filters the incoming value
 * @param newValue the new value to filter
 * @param filterReg - the register to hold the state of this filter
 * @param constant - nominally the reciprocol of the product of the time constant and the sample rate. This will always be < 1.0
 */
float lowPassFilter(float newValue, float* filterReg, float constant){
	float nv = newValue;
	//ensure input is not NaN
	if(nv != nv){
		nv = 0;
	}
	float fv = *filterReg;
	fv *= (1.0 - constant);
	fv += nv*constant;
	if(fv != fv){
		//this is NaN or some other non-finite number
		//TODO: decide if this is the right course of action
		fv = 0.0;
	}
	*filterReg = fv;
	return fv;
}
