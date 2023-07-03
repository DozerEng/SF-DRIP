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
#include "stepperMotor.h"
#include <math.h>
#include <ports.h>
#include <sigGen.h>


//*************************************************
//defines
//*************************************************

//*************************************************
//Types
//*************************************************


typedef struct {
	StepperType type;
	float distancePerStep;
	float stepPhasePerDistance;
	float homePosition;
	float eotPosition;
	PortPin homePin;
	PortPin eotPin;
	PortPin outputPin1;
	PortPin outputPin2;
	bool invertHomePin;
	bool invertEotPin;
	float pwmFraction;
	float phaseOffset;
	float currentPosition;
	float currentVelocity;
	float currentPhase;//current phase, expressed as a value from 0 to 1 over a complete phase cycle
	bool zeroing;
	bool zero;
	StepperOnIdle onIdle;
	int32_t stepBacklog;
	uint32_t iPhase;
	uint32_t iOldPhase;



} StepperChannel;



//*************************************************
//Variables
//*************************************************
static StepperChannel m_steppers[STEPPER_CHANNELS];
static int32_t  m_pwmChannelIndex;

//*************************************************
//function prototypes
//*************************************************
static void computeStepperOutputsInternal(StepperChannel* sc, float position, float velocity);


//*************************************************
//code
//*************************************************

/**
 * configures a stepper channel.
 * Note that a step is considered an increment of 90 degrees of the winding phase cycle.
 * @param index can be up to STEPPER_CHANNELS - 1
 * @param type the type of this channel. Only one channel can be type STEPPER_PWM
 * @param distancePerStep the distance (in arbitrary units) that this channel will travel per single step of the motor
 *
 *
 */
void configStepper(uint32_t index, StepperType type, float distancePerStep, float pwmFraction){
	if(index >= STEPPER_CHANNELS){
		return;
	}
	StepperChannel* sc = &(m_steppers[index]);
	sc->type = type;
	sc->distancePerStep = distancePerStep;
	sc->stepPhasePerDistance = 0.25/distancePerStep;//precompute this
	sc->pwmFraction = pwmFraction;

	if(type == STEPPER_PWM){
		m_pwmChannelIndex = index;

	}

}

/**
 * Configures the output pins for this channel.
 * This only applies to STEPPER_IO_AB and STEPPER_IO_STEP_DIR stepper types
 */
void configStepperOutputPins(uint32_t index, PortPin pin1, PortPin pin2){
	if(index >= STEPPER_CHANNELS){
		return;
	}
	m_steppers[index].outputPin1 = pin1;
	m_steppers[index].outputPin2 = pin2;
}
bool isStepperConfigured(uint32_t index){
	if(index >= STEPPER_CHANNELS){
		return false;
	}
	return m_steppers[index].type != STEPPER_UNCONFIG;
}
void stepperFastCode(void){

}

static void computeStepperOutputsInternal(StepperChannel* sc, float position, float velocity){
	if(sc->zero){
		sc->zero = false;
		//if zeroing then grab current phase
		float currentPhase = sc->currentPosition * sc->stepPhasePerDistance;
		currentPhase += sc->phaseOffset;

		float newPhase = position * sc->stepPhasePerDistance;

		float deltaPhase = currentPhase - newPhase;

		//convert to value from 0 to 1, not including 1
		deltaPhase -= floorf(deltaPhase);
		sc->phaseOffset = deltaPhase;
		sc->currentPosition = position;
	} else if(sc->zeroing){
		//don't do anything here.
	} else {
		sc->currentPosition = position;
	}
	sc->currentVelocity = velocity;

	//compute new phase. note this spans 0 to 1 over one full phase cycle (0 to 360 degrees of phase)
	float oldPhase = sc->currentPhase;
	float phase = sc->currentPosition;
	phase *= sc->stepPhasePerDistance;
	phase += sc->phaseOffset;
	phase -= floorf(phase);
	sc->currentPhase = phase;


	//compute outputs if necessary
	uint32_t ip = (uint32_t)(phase*4);//this will be an int from 0 to 3
	uint32_t iop = (uint32_t)(oldPhase*4);//this will be an int from 0 to 3

	sc->iPhase = ip;
	sc->iOldPhase = iop;

	switch(sc->type){
	case STEPPER_UNCONFIG:
	case STEPPER_PWM:
		break;
	case STEPPER_IO_AB:

		switch(ip){
		default:
		case 0://this is 0 degrees of phase
			setPin(sc->outputPin1, false);//this is the sin output
			setPin(sc->outputPin2, true);//this is the cos output
			break;
		case 1://this is 90 degrees of phase
			setPin(sc->outputPin1, true);//this is the sin output
			setPin(sc->outputPin2, true);//this is the cos output
			break;
		case 2://this is 180 degrees of phase
			setPin(sc->outputPin1, true);//this is the sin output
			setPin(sc->outputPin2, false);//this is the cos output
			break;
		case 3://this is 270 degrees of phase
			setPin(sc->outputPin1, false);//this is the sin output
			setPin(sc->outputPin2, false);//this is the cos output
			break;
		}
		break;
	case STEPPER_IO_STEP_DIR:
		//first compute the number of steps to move. Allow for values in the range +/-2
		;
		int32_t deltaPhase = ((int32_t)ip) - ((int32_t)iop);
		switch(deltaPhase){
		case -3:
			sc->stepBacklog += 1;//this value can only happen when making the transition from 3 to 0, which is a wrap around of the phase
			break;
		case -2:
			sc->stepBacklog += -2;
			break;
		case -1:
			sc->stepBacklog += -1;
			break;
		case 0:
			break;
		case 1:
			sc->stepBacklog += 1;
			break;
		case 2:
			sc->stepBacklog += 2;
			break;
		case 3:
			sc->stepBacklog += -1;//this value can only happen when making the transition from 0 to 3, which is a wrap around of the phase
			break;
		}

		//the backlog has now been updated. It will be positive if we need to step up in value and negative if we need to step down in value

		//now execute a step if necessary step.
		//first grab the state of the step and dir pins as this is necessary to figure out what to do next
		PortPin stepPin = sc->outputPin1;
		PortPin dirPin = sc->outputPin2;
		bool dirPinVal = isPinOutputSet(dirPin);
		bool stepPinVal = isPinOutputSet(stepPin);

		if(stepPinVal){
			setPin(stepPin, false);//the step pin is high so all we can do is clear it
			//we'll worry about chipping away at the backlog next time
		} else if((dirPinVal && sc->stepBacklog > 0) || (!dirPinVal && sc->stepBacklog < 0)){
			//the direction is correct so initiate a step
			setPin(stepPin, true);
			//now reduce the backlog
			if(sc->stepBacklog > 0){
				sc->stepBacklog -= 1;
			} else {
				sc->stepBacklog += 1;
			}


		} else if(sc->stepBacklog == 0){
			//there's no need to do anything because there's no backlog
		} else {
			//by deduction, the direction pin is the wrong value so set it correctly to the desired direction
			if(sc->stepBacklog > 0){
				setPin(dirPin, true);
			} else {
				setPin(dirPin, false);
			}
		}

		break;
	}

}

void computeStepperOutputsFromVelocity(uint32_t index, float velocity, float deltaTime){
	if(index >= STEPPER_CHANNELS){
		return;
	}
	StepperChannel* sc = &(m_steppers[index]);


	computeStepperOutputsInternal(sc, sc->currentPosition + velocity*deltaTime, velocity);



}
void computeStepperOutputs(uint32_t index, float position, float deltaTime){
	if(index >= STEPPER_CHANNELS){
		return;
	}

	StepperChannel* sc = &(m_steppers[index]);

	computeStepperOutputsInternal(sc, position, (position - sc->currentPosition)/deltaTime);




}
/**
 * @return the specified PWM output. 0 if PWM stepper not configured.
 */
float getStepperPwmOutputs(uint32_t outputIndex){
	if(m_pwmChannelIndex < 0){
		return 0.0;
	}
	StepperChannel* sc = &(m_steppers[m_pwmChannelIndex]);

	float phase = sc->currentPhase;

	switch(outputIndex){
	default:
	case 0:
		break;
	case 1:
		phase += 0.5f;
		break;
	case 2:
		phase += 0.25f;
		break;
	case 3:
		phase += 0.75f;
		break;

	}
	phase -= floorf(phase);
	phase *= M_PI*2.0f;
	float result = getSineWave(phase)*sc->pwmFraction;
	if(getStepperState(m_pwmChannelIndex) == STEPPER_STATE_IDLE){
		switch(sc->onIdle){
		case ON_IDLE_FULL_DRIVE:
			break;
		case ON_IDLE_HALF_DRIVE:
			result *= 0.5;
			break;
		case ON_IDLE_QUARTER_DRIVE:
			result *= 0.25;
			break;
		case ON_IDLE_NO_DRIVE:
			result = 0;
			break;
		}
	}
	return result;
}
void initSteppers(void){
	m_pwmChannelIndex = -1;
//	float phase = 0;
//	float scale = M_TWOPI/SIN_LOOKUP_SIZE;
//	for(uint32_t i = 0; i < SIN_LOOKUP_SIZE; ++i){
//		phase = i*scale;
//		m_sinLookup[i] = 0.495*sinf(phase)+0.505;
//	}


	for(uint32_t i = 0; i < STEPPER_CHANNELS; ++i){
		StepperChannel* s = &(m_steppers[i]);
		s->distancePerStep = 0;
		s->eotPin = NULL_PIN;
		s->eotPosition = 1;
		s->homePin = NULL_PIN;
		s->homePosition = 0;
		s->invertEotPin = false;
		s->invertHomePin = false;
		s->outputPin1 = NULL_PIN;
		s->outputPin2 = NULL_PIN;
		s->pwmFraction = 0;
		s->phaseOffset =0;
		s->type = STEPPER_UNCONFIG;
		s->currentPosition = 0;
		s->zeroing = false;
		s->zero = false;
		s->onIdle = ON_IDLE_HALF_DRIVE;//ON_IDLE_FULL_DRIVE;
	}

}
StepperState getStepperState(uint32_t index){
	if(index >= STEPPER_CHANNELS){
		return false;
	}
	StepperChannel* sc = &(m_steppers[index]);
	StepperState result = STEPPER_STATE_IDLE;
	if(sc->type == STEPPER_UNCONFIG){
		result = STEPPER_STATE_UNCONFIGURED;
	} else if(sc->zeroing){
		result = STEPPER_STATE_ZEROING;
	} else {
		if(getTrajectoryQueueLength(index) > 0){
			result = STEPPER_STATE_MOVING;
		}
	}

	return result;
}


void setStepperOnIdle(uint32_t index, StepperOnIdle soi){
	if(index >= STEPPER_CHANNELS){
		return;
	}
	StepperChannel* sc = &(m_steppers[index]);
	sc->onIdle = soi;
}
void setStepperZeroing(uint32_t index, bool z){
	if(index >= STEPPER_CHANNELS){
		return;
	}
	StepperChannel* sc = &(m_steppers[index]);
	if(!z && sc->zeroing){
		sc->zero = true;
	} else {
	}
	sc->zeroing = z;
}
float getStepperPosition(uint32_t index){
	float result = NAN;
	if(index < STEPPER_CHANNELS){
		StepperChannel* sc = &(m_steppers[index]);
		result = sc->currentPosition;
	}
	return result;
}
float getStepperVelocity(uint32_t index){
	float result = NAN;
	if(index < STEPPER_CHANNELS){
		StepperChannel* sc = &(m_steppers[index]);
		result = sc->currentVelocity;
	}
	return result;
}
bool isStepperIdle(uint32_t index){
	return getStepperState(m_pwmChannelIndex) == STEPPER_STATE_IDLE;
}

