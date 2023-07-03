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

//*************************************************
//includes
//*************************************************
#include "sigGen.h"
#include <math.h>
#include <thermistors.h>
#include "fastcodeUtil.h"
//#include "thermistors.h"
#include "adcManager.h"
#include "feedbackControl.h"
#include "brushlessMotor.h"
#include "stepperMotor.h"



//*************************************************
//defines
//*************************************************


#define LOOKUP_SIZE 360
#define PI 3.1415926535897932384626433832795028841

//*************************************************
//function prototypes
//*************************************************
static float lookup(SigType type, float phase, uint32_t index);
//*************************************************

//*************************************************
//Types
//*************************************************


//*************************************************
//local variables
//*************************************************



static const float m_sinLookup[LOOKUP_SIZE] = {
		0.500, 0.509, 0.517, 0.526, 0.535, 0.544, 0.552, 0.561, 0.570, 0.578,
		0.587, 0.595, 0.604, 0.612, 0.621, 0.629, 0.638, 0.646, 0.655, 0.663,
		0.671, 0.679, 0.687, 0.695, 0.703, 0.711, 0.719, 0.727, 0.735, 0.742,
		0.750, 0.758, 0.765, 0.772, 0.780, 0.787, 0.794, 0.801, 0.808, 0.815,
		0.821, 0.828, 0.835, 0.841, 0.847, 0.854, 0.860, 0.866, 0.872, 0.877,
		0.883, 0.889, 0.894, 0.899, 0.905, 0.910, 0.915, 0.919, 0.924, 0.929,
		0.933, 0.937, 0.941, 0.946, 0.949, 0.953, 0.957, 0.960, 0.964, 0.967,
		0.970, 0.973, 0.976, 0.978, 0.981, 0.983, 0.985, 0.987, 0.989, 0.991,
		0.992, 0.994, 0.995, 0.996, 0.997, 0.998, 0.999, 0.999, 1.000, 1.000,
		1.000, 1.000, 1.000, 0.999, 0.999, 0.998, 0.997, 0.996, 0.995, 0.994,
		0.992, 0.991, 0.989, 0.987, 0.985, 0.983, 0.981, 0.978, 0.976, 0.973,
		0.970, 0.967, 0.964, 0.960, 0.957, 0.953, 0.949, 0.946, 0.941, 0.937,
		0.933, 0.929, 0.924, 0.919, 0.915, 0.910, 0.905, 0.899, 0.894, 0.889,
		0.883, 0.877, 0.872, 0.866, 0.860, 0.854, 0.847, 0.841, 0.835, 0.828,
		0.821, 0.815, 0.808, 0.801, 0.794, 0.787, 0.780, 0.772, 0.765, 0.758,
		0.750, 0.742, 0.735, 0.727, 0.719, 0.711, 0.703, 0.695, 0.687, 0.679,
		0.671, 0.663, 0.655, 0.646, 0.638, 0.629, 0.621, 0.612, 0.604, 0.595,
		0.587, 0.578, 0.570, 0.561, 0.552, 0.544, 0.535, 0.526, 0.517, 0.509,
		0.500, 0.491, 0.483, 0.474, 0.465, 0.456, 0.448, 0.439, 0.430, 0.422,
		0.413, 0.405, 0.396, 0.388, 0.379, 0.371, 0.362, 0.354, 0.345, 0.337,
		0.329, 0.321, 0.313, 0.305, 0.297, 0.289, 0.281, 0.273, 0.265, 0.258,
		0.250, 0.242, 0.235, 0.228, 0.220, 0.213, 0.206, 0.199, 0.192, 0.185,
		0.179, 0.172, 0.165, 0.159, 0.153, 0.146, 0.140, 0.134, 0.128, 0.123,
		0.117, 0.111, 0.106, 0.101, 0.095, 0.090, 0.085, 0.081, 0.076, 0.071,
		0.067, 0.063, 0.059, 0.054, 0.051, 0.047, 0.043, 0.040, 0.036, 0.033,
		0.030, 0.027, 0.024, 0.022, 0.019, 0.017, 0.015, 0.013, 0.011, 0.009,
		0.008, 0.006, 0.005, 0.004, 0.003, 0.002, 0.001, 0.001, 0.000, 0.000,
		0.000, 0.000, 0.000, 0.001, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006,
		0.008, 0.009, 0.011, 0.013, 0.015, 0.017, 0.019, 0.022, 0.024, 0.027,
		0.030, 0.033, 0.036, 0.040, 0.043, 0.047, 0.051, 0.054, 0.059, 0.063,
		0.067, 0.071, 0.076, 0.081, 0.085, 0.090, 0.095, 0.101, 0.106, 0.111,
		0.117, 0.123, 0.128, 0.134, 0.140, 0.146, 0.153, 0.159, 0.165, 0.172,
		0.179, 0.185, 0.192, 0.199, 0.206, 0.213, 0.220, 0.228, 0.235, 0.242,
		0.250, 0.258, 0.265, 0.273, 0.281, 0.289, 0.297, 0.305, 0.313, 0.321,
		0.329, 0.337, 0.345, 0.354, 0.362, 0.371, 0.379, 0.388, 0.396, 0.405,
		0.413, 0.422, 0.430, 0.439, 0.448, 0.456, 0.465, 0.474, 0.483, 0.491
};
//static float m_triangleLookup[LOOKUP_SIZE];
//static float m_squareLookup[LOOKUP_SIZE];
static bool m_enabled = false;

static TempControlState m_tempState[4] = {
		{0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0},
};



static float m_outputs[] = {0, 0, 0, 0};
static SigType m_sigTypes[] = {OFF_SIG, OFF_SIG, OFF_SIG, OFF_SIG};
static float m_amplitudes[] = {0, 0, 0, 0};
static float m_frequencyHz[] = {0, 0, 0, 0};
static float m_phase[] = {0, 0, 0, 0};


//*************************************************



void sigGenConfig(OutputChannel chan, SigType type, float freqHz, float ampFrac){
	uint32_t i = chan;
	if(i >= 0 && i < 4){
		m_amplitudes[i] =  ampFrac > 0.99f ? 0.99f : ampFrac;
		m_frequencyHz[i] = freqHz;
		m_sigTypes[i] = type;
	}
	m_enabled = true;
}
void setSigGenType(OutputChannel chan, SigType type){
	if(chan >= 0 && chan < 4){
		m_sigTypes[chan] = type;
		float v = 0;
		switch(type){
		case OFF_SIG:
		case DC_SIG:
		case SINE_SIG:
		case SQUARE_SIG:
		case TRIANGLE_SIG:
		case COMPLEMENT_SIG:
			v = 0;
			break;
		case THERMAL_SIG:
		case STEPPER_SIG:
		case BRUSHLESS_MOTOR_SIG:
		case FEEDBACK_SIG:
			v = 1.0f;
			break;

		}
		m_amplitudes[chan] = v;
	}
}
SigType getSigGenType(OutputChannel chan){
	SigType result = OFF_SIG;

	if(chan >= 0 && chan < 4){
		result = m_sigTypes[chan];

	}
	return result;
}
void sigGenInit(){
	//populate the microstepping lookup table
//		for(int i = 0; i < LOOKUP_SIZE; ++i){
//			float si = i;
//			si /= LOOKUP_SIZE;
//			float ti = si * 2;
//			si *= PI*2;
//			m_sinLookup[i] = 0.5f*sinf(si) + 0.5f;
//			m_squareLookup[i] = si > PI ? 1.0f : 0.0f;
//			m_triangleLookup[i] = ti < 1.0f ? ti : 2.0f - ti;
//		}
}

//bool isSigGenEnabled(void){
//	return m_enabled;
//}
//void sigGenDisable(void){
//	m_enabled = false;
//}

static float lookup(SigType type, float phase, uint32_t index){
	float result;
	float p = phase;


//	uint32_t pi = (uint32_t)(p*LOOKUP_SIZE);

	switch(type){
	default:
	case OFF_SIG:
		result = 0.0;
		break;
	case STEPPER_SIG:
		result = getStepperPwmOutputs(index);
		break;
	case DC_SIG:
		result = 1.0;
		break;
	case SINE_SIG:
//		result = m_sinLookup[pi];
		result = getSineWave(p*M_TWOPI);
		break;
	case SQUARE_SIG:
//		result = m_squareLookup[pi];
		result = getSquareWave(p*M_TWOPI);
		break;
	case TRIANGLE_SIG:
//		result = m_triangleLookup[pi];
		result = getTriangleWave(p*M_TWOPI);
		break;
	case COMPLEMENT_SIG:
		result = 0;
		break;
	case THERMAL_SIG:
		;//this empty line is here to stop a compiler error
		uint32_t adcNum = m_tempState[index].thermistorNum;
		if(adcNum < 0){
			result = 0.0;
		} else {

			float adc = getAdcValue(adcNum);
			float tNow = getThermistorTemperature(adc);
			calcPIDOutput(&(m_tempState[index]), tNow, getSecondsPerBranch());
			result = m_tempState[index].heaterPwm;
//			m_outputs[index] = result;
		}
		break;
	case BRUSHLESS_MOTOR_SIG:
		result = getBrushlessPhaseValue(index);
		break;
	case FEEDBACK_SIG:
		result = getOutputForPWM(index);
		break;
	}

	return result;
}
TempControlState* getTempControlState(OutputChannel chan){

	TempControlState* result = 0;
	if(chan >= 0 && chan < 4){
		result = &(m_tempState[chan]);
	}
	return result;
}
void sigGenFastCode(){
	//first compute normal values
	uint32_t j = 0;
	for(uint32_t i = 0; i < 4; ++i){
		//first compute new phase
		m_phase[i] += getSecondsPerBranch()*m_frequencyHz[i];
		m_phase[i] -= floorf(m_phase[i]);
		if(m_sigTypes[i] == COMPLEMENT_SIG){
			switch(i){
			case 0:
				j = 1;//
				break;
			case 1:
				j = 0;
				break;
			case 2:
				j = 3;
				break;
			case 3:
				j = 2;
				break;
			}
			float v = lookup(m_sigTypes[j], m_phase[j], j) * m_amplitudes[j];
			m_outputs[i] = 1.0f - v;
		} else {
			m_outputs[i] = lookup(m_sigTypes[i], m_phase[i], i) * m_amplitudes[i];
		}

		powerOutputSetpoint(i, m_outputs[i]);
	}
}
float* getSigGenOutputs(){
	return m_outputs;
}

SigType lookupType(uint32_t i){
	SigType result = OFF_SIG;
	if(i >= OFF_SIG && i <= THERMAL_SIG){
		result = (SigType)i;
	}
	return result;
}


void sigGenTemperatureControl(OutputChannel chan, uint32_t thermistorChan, float setpoint, float pCoeff, float iCoeff, float dCoeff, float maxPwm){
	if(chan >= 0 && chan < 4){
		m_sigTypes[chan] = THERMAL_SIG;

		m_amplitudes[chan] = 1.0;

		m_tempState[chan].setpoint = setpoint;
		m_tempState[chan].thermistorNum = thermistorChan;
		m_tempState[chan].pCoeff = pCoeff;
		m_tempState[chan].iCoeff = iCoeff;
		m_tempState[chan].dCoeff = dCoeff;
		if(maxPwm > 1.0){
			maxPwm = 1.0;
		} else if(maxPwm < 0.0){
			maxPwm = 0.0;
		}
		m_tempState[chan].maxPwm = maxPwm;

		//tempState[chan];
	}
}

/**
 * Computes a triangle wave signal. A single cycle spans 2*pi radians.
 * @param radians the input phase
 * @return the resulting value spanning 0.0f to 1.0f
 */
float getTriangleWave(float radians){
	float fi = radians*(1.0f/M_TWOPI);
	fi -= floorf(fi);

	float result = fi < 0.5f ? 2.0f*fi : 2.0f - 2.0f*fi;
	return result;
}
/**
 * Computes a square wave signal. A single cycle spans 2*pi radians.
 * @param radians the input phase
 * @return the resulting value spanning 0.0f to 1.0f
 */
float getSquareWave(float radians){
	float fi = radians*(1.0f/M_TWOPI);
	fi -= floorf(fi);


	float result = fi > 0.5f ? 1.0f : 0.0f;
	return result;
}
/**
 * Computes a sine wave signal. A single cycle spans 2*pi radians.
 * This is based on a 360 element lookup table
 * @param radians the input phase
 * @return the resulting value spanning 0.0f to 1.0f
 */
float getSineWave(float radians){
	float fi = radians*(LOOKUP_SIZE/M_TWOPI);
	uint32_t i = floorf(fi);
	if(i >= LOOKUP_SIZE){
		i = 0;
	}
	float result = m_sinLookup[i];
	return result;
}


