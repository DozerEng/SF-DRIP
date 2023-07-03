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
//this allows any GPIO pin to be used as a low frequency PWM

//*************************************************
//includes
//*************************************************
#include "softPwm.h"
#include <math.h>
#include "fastcodeUtil.h"

//*************************************************
//defines
//*************************************************
//#define SOFT_SECONDS_PER_FASTCODE SECONDS_PER_FASTLOOP
#define SOFT_PWM_COUNT 16
//*************************************************
//Types
//*************************************************
typedef enum {
	SOFT_UNCONFIG = 0,
	SOFT_PWM = 1,
	SOFT_ONE_SHOT = 2,
	SOFT_FOLLOWER = 3,
} SoftPwmType;

typedef struct {
	PortPin output;
	PortPin trigger;
	SoftPwmType type;
	float periodOrTimeOrGain;
	float compareTime;
	uint32_t channelToFollow;
	bool invertTrigger;
	float timeCounter;


} SoftPwm;

//*************************************************
//Variables
//*************************************************

static SoftPwm m_softPwms[SOFT_PWM_COUNT];

//*************************************************
//function prototypes
//*************************************************


//*************************************************
//code
//*************************************************

/**
 * configures one PWM channel.
 * This will likely be called by sfdqPackets because of communication to set up a PWM channel.
 * It may also be called by application specific code
 * @param channel - indicates which channel to setup
 * @param output the pin to output the PWM signal on
 * @param the input trigger pin for One-shot mode
 */
void configSoftPwm(uint32_t channel, PortPin output){
	if(channel >= SOFT_PWM_COUNT){
		return;
	}
	SoftPwm* s = &(m_softPwms[channel]);

	s->output = output;

}

/**
 * this sets the duty cycle of a particular PWM channel.
 * It may also do other stuff that we haven't thought of yet.
 */
void updateSoftPwm(uint32_t channel, float dutyCycle){
	if(channel >= SOFT_PWM_COUNT){
		return;
	}
	SoftPwm* s = &(m_softPwms[channel]);
	switch(s->type){
	case SOFT_UNCONFIG:
		break;
	case SOFT_PWM:
		s->compareTime = dutyCycle * s->periodOrTimeOrGain;

		//do a quick scan for followers and if so, set them up
		for(uint32_t i = 0; i < SOFT_PWM_COUNT; ++i){
			SoftPwm* sf = &(m_softPwms[i]);

			if(sf->type == SOFT_FOLLOWER && sf->channelToFollow == channel){
				sf->compareTime = s->compareTime * sf->periodOrTimeOrGain;
			}


		}
		break;
	case SOFT_ONE_SHOT:
		break;
	case SOFT_FOLLOWER:
		break;
	}
}
void deconfigureSoftPwm(uint32_t channel){
	if(channel >= SOFT_PWM_COUNT){
		return;
	}
	SoftPwm* s = &(m_softPwms[channel]);
	s->type = SOFT_UNCONFIG;
}
void setupSoftPwmAsOneShot(uint32_t channel, float time, PortPin trigger, bool fallingNotRising){
	if(channel >= SOFT_PWM_COUNT){
		return;
	}
	SoftPwm* s = &(m_softPwms[channel]);
	s->type = SOFT_ONE_SHOT;
	s->periodOrTimeOrGain = time;
	s->trigger = trigger;
	s->invertTrigger = fallingNotRising;
}
void setupSoftPwmAsFollower(uint32_t channel, uint32_t channelToFollow, float gain){
	if(channel >= SOFT_PWM_COUNT){
		return;
	}
	SoftPwm* s = &(m_softPwms[channel]);
	s->type = SOFT_FOLLOWER;
	s->channelToFollow = channelToFollow;
	s->periodOrTimeOrGain = gain;



}
void setupSoftPwmAsPwm(uint32_t channel, float frequencyHz){
	if(channel >= SOFT_PWM_COUNT){
		return;
	}
	SoftPwm* s = &(m_softPwms[channel]);
	s->type = SOFT_PWM;
	s->periodOrTimeOrGain = 1.0f/frequencyHz;
}
/**
 * @return the number of soft PWM channels
 */
uint32_t getSoftPwmCount(void){
	return SOFT_PWM_COUNT;
}
/**
 * The setpoint is the duty cycle when in PWM mode
 * it is time when in one shot mode
 * it is duty cycle when in follower mode
 * @return the setpoint of the specified channel.
 */
float getSoftPwmSetpoint(uint32_t channel){
	float result = NAN;
	if(channel >= SOFT_PWM_COUNT){
		return NAN;
	}
	SoftPwm* s = &(m_softPwms[channel]);
	switch(s->type){
	case SOFT_UNCONFIG:
		break;
	case SOFT_FOLLOWER:
	case SOFT_PWM:
		result = s->compareTime / s->periodOrTimeOrGain;
		break;
	case SOFT_ONE_SHOT:
		result = s->periodOrTimeOrGain;
		break;
	}
	return result;
}

bool isSoftPwmConfigured(uint32_t channel){
	bool result = false;
	if(channel < SOFT_PWM_COUNT){
		SoftPwm* s = &(m_softPwms[channel]);
		if(s->type != SOFT_UNCONFIG){
			result = true;
		}
	}
	return result;
}

void softPwmFastCode(void){
	for(uint32_t channel = 0; channel < SOFT_PWM_COUNT; ++channel){
		SoftPwm* s = &(m_softPwms[channel]);
		//first make sure the counter has not somehow become non-finite.
		if(!finitef(s->timeCounter)){
			s->timeCounter = 0;
		}
		switch(s->type){
		case SOFT_UNCONFIG:
			//nothing to do here
			break;
		case SOFT_PWM:
			//will cycle the counter and set the output at zero, reset it at the capture value
			s->timeCounter += getSecondsPerFastLoop();
			if(s->timeCounter > (s->periodOrTimeOrGain * 2)){
				s->timeCounter = 0;
			} else if(s->timeCounter >= s->periodOrTimeOrGain){
				s->timeCounter -= s->periodOrTimeOrGain;
				if(s->compareTime > 0){
					setPin(s->output, true);
				}
			} else if(s->timeCounter >= s->compareTime){
				setPin(s->output, false);
			}

			break;
		case SOFT_ONE_SHOT:
			//this will check the trigger pin

			if(isPinInputSet(s->trigger) || s->timeCounter > 0){

				if(s->timeCounter >= s->periodOrTimeOrGain){
					setPin(s->output, false);
				} else {
					s->timeCounter += getSecondsPerFastLoop();
					setPin(s->output, true);
				}
			}
			break;
		case SOFT_FOLLOWER:
			//do followers later after all the other channels have been computed
			break;
		}
	}
	//now do the followers now that the followed channels have been computed
	for(uint32_t channel = 0; channel < SOFT_PWM_COUNT; ++channel){
		SoftPwm* s = &(m_softPwms[channel]);

		//this will use the same counter as the channel it is following but apply its own gain

		if(s->type == SOFT_FOLLOWER){
			SoftPwm* sf = &(m_softPwms[s->channelToFollow]);
			if(s->channelToFollow < SOFT_PWM_COUNT && sf->type == SOFT_PWM){
				setPin(s->output, sf->timeCounter < s->compareTime);
			} else {
				setPin(s->output, false);
			}
		}
	}

}

bool hasOneShotFired(uint32_t channel) {
	bool result = false;
	if(channel < SOFT_PWM_COUNT){
		SoftPwm* s = &(m_softPwms[channel]);
		if(s->timeCounter >= s->periodOrTimeOrGain){
			result = true;
		}
	}
	return result;
}
bool isOneShotReset(uint32_t channel){
	bool result = false;
	if(channel < SOFT_PWM_COUNT){
		SoftPwm* s = &(m_softPwms[channel]);
		if(s->timeCounter == 0){
			result = true;
		}
	}
	return result;
}


void resetOneShot(uint32_t channel) {
	if(channel < SOFT_PWM_COUNT){
		SoftPwm* s = &(m_softPwms[channel]);
		setPin(s->output, false);
		s->timeCounter = 0;
	}
}

