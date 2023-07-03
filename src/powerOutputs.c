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
#include <math.h>
#include "stm32f4xx_tim.h"
#include "pwm.h"
#include "powerOutputs.h"
#include "fastcodeUtil.h"


//********************************************************
//Defines
//********************************************************









//*************************************************
//Types
//*************************************************

typedef struct {
	float rawOutput;
	float setpoint;
	PwmTimer timer;
	PwmChannel channel;
	float maxSlewPerBranch;
} PowerOutput;


//********************************************************
//Variable Definitions
//********************************************************

static PowerOutput m_powerOutputs[OUTPUT_CHANNEL_NUM] = {
		{0, 0, 0, PWM_CH1, 0.05f},
		{0, 0, 0, PWM_CH2, 0.05f},
		{0, 0, 0, PWM_CH3, 0.05f},
		{0, 0, 0, PWM_CH4, 0.05f},
};




//********************************************************



//********************************************************
//Function Prototypes
//********************************************************


static float slew(float delta, float slewPerBranch);


//********************************************************


void powerOutputFastCode(void){











	for(uint32_t i = 0; i < OUTPUT_CHANNEL_NUM; ++i){

		PowerOutput* p = &(m_powerOutputs[i]);
		//only setup if the timer has been setup
		if(p->timer != 0){
			//slew outputs
			p->rawOutput += slew(p->setpoint - p->rawOutput, p->maxSlewPerBranch);
			//ensure value is not invalid
			if(!finitef(p->rawOutput)){
				p->rawOutput = 0.0f;
			}
			//set the outputs
			setPwm(p->timer, p->channel, p->rawOutput);
		}
	}


}


void powerOutputSetpoint(OutputChannel channel, float value){
	if(channel >= OUTPUT_CHANNEL_NUM){
		return;
	}
	float v = value;
	if(v < 0){
		v = 0;
	} else if(v >= 0.95){
		v = 0.95;
	}
	PowerOutput* p = &(m_powerOutputs[channel]);
	p->setpoint = v;
}


void powerOutputInit(OutputChannel channel, PwmTimer timer, PwmChannel timerChannel){
	if(channel < OUTPUT_CHANNEL_NUM){
		PowerOutput* p = &(m_powerOutputs[channel]);
		p->timer = timer;
		p->channel = timerChannel;

		if(timer == PWM_TIMER8 || timer == PWM_TIMER1){
			pwmInit(timer, 200, 1);//100 gives 303kHz with tim8
		} else {
			pwmInit(timer, 200, 0);//100 gives 225kHz with tim3
		}

		setPwm(timer, timerChannel, 0.0f);
		initPwm(timer, timerChannel, 0.0);


	}
}


static float slew(float delta, float slewPerBranch){
	float result = delta;
	if(delta > slewPerBranch){
		result = slewPerBranch;
	} else if(delta < -slewPerBranch){
		result = -slewPerBranch;
	}
	return result;
}

void setMaxOutputSlew(OutputChannel channel, float fractionPerSecond){
	if(channel < OUTPUT_CHANNEL_NUM){
		PowerOutput* p = &(m_powerOutputs[channel]);

		p->maxSlewPerBranch = fractionPerSecond*getSecondsPerBranch();
	}
}
