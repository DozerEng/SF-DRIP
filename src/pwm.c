
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
#include "pwm.h"
#include "stm32f4xx_tim.h"
#include <math.h>






//********************************************************
//Variable Definitions
//********************************************************
//********************************************************



//********************************************************
//Function Prototypes
//********************************************************
TIM_TypeDef* getTimer(PwmTimer timer){
	TIM_TypeDef* result;
	switch(timer){
	case PWM_TIMER1:
		result = TIM1;
		break;
	case PWM_TIMER2:
		result = TIM2;
		break;
	case PWM_TIMER3:
		result = TIM3;
		break;
	case PWM_TIMER4:
		result = TIM4;
		break;
	case PWM_TIMER5:
		result = TIM5;
		break;
	case PWM_TIMER6:
		result = TIM6;
		break;
	case PWM_TIMER7:
		result = TIM7;
		break;
	case PWM_TIMER8:
		result = TIM8;
		break;
	case PWM_TIMER9:
		result = TIM9;
		break;
	case PWM_TIMER10:
		result = TIM10;
		break;
	case PWM_TIMER11:
		result = TIM11;
		break;
	case PWM_TIMER12:
		result = TIM12;
		break;
	case PWM_TIMER13:
		result = TIM13;
		break;
	case PWM_TIMER14:
		result = TIM14;
		break;
	default:
		result = 0;
		break;

	}
	return result;
}
//********************************************************




/**
 * initialize the pwm driver
 */
void pwmInit(PwmTimer tim, uint32_t period, uint16_t prescaler){
	TIM_TypeDef* timer = getTimer(tim);
	TIM_TimeBaseInitTypeDef tbitd;
	tbitd.TIM_ClockDivision = TIM_CKD_DIV1;
	tbitd.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	tbitd.TIM_Period = period;
	tbitd.TIM_Prescaler = prescaler;
	tbitd.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit((timer), &tbitd);
	TIM_InternalClockConfig((timer));
	TIM_Cmd((timer), ENABLE);



}

void initPwm(PwmTimer tim, PwmChannel c, float duty){
	TIM_TypeDef* timer = getTimer(tim);
	TIM_OCInitTypeDef oitd;
	oitd.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oitd.TIM_OCMode = TIM_OCMode_PWM1;
	oitd.TIM_OCNIdleState = 0;
	oitd.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	oitd.TIM_OCPolarity = TIM_OCPolarity_High;
	oitd.TIM_OutputNState = TIM_OutputNState_Disable;
	oitd.TIM_OutputState = TIM_OutputState_Enable;


	uint32_t period = timer->ARR;
	uint32_t pulse = (uint32_t)(((float)period)*duty);

	//check if it's inf or NaN
	if(pulse > period || !isfinite(duty)){
		pulse = 0;
	}

	if(pulse == period){
		pulse = period - 1;
	}



	oitd.TIM_Pulse = pulse;

	switch(c){
	case PWM_CH1:
		TIM_SetCompare1(timer, pulse);
		TIM_OC1Init(timer, &oitd);
		break;
	case PWM_CH2:
		TIM_SetCompare2(timer, pulse);
		TIM_OC2Init(timer, &oitd);
		break;
	case PWM_CH3:
		TIM_SetCompare3(timer, pulse);
		TIM_OC3Init(timer, &oitd);
		break;
	case PWM_CH4:
		TIM_SetCompare4(timer, pulse);
		TIM_OC4Init(timer, &oitd);
		break;

	}

	TIM_CtrlPWMOutputs(timer, ENABLE);



}


void setPwm(PwmTimer tim, PwmChannel c, float duty){
	TIM_TypeDef* timer = getTimer(tim);
	uint32_t period = timer->ARR;
	uint32_t pulse = (uint32_t)(((float)period)*duty);

	//check if it's inf or NaN
	if(pulse > period){
		pulse = period - 1;
	} else if(!isfinite(duty)){
		pulse = 0;
	}





	switch(c){
	case PWM_CH1:
		timer->CCR1 = pulse;
//		TIM_SetCompare1(timer, pulse);

		break;
	case PWM_CH2:
		timer->CCR2 = pulse;
//		TIM_SetCompare2(timer, pulse);

		break;
	case PWM_CH3:
		timer->CCR3 = pulse;
//		TIM_SetCompare3(timer, pulse);

		break;
	case PWM_CH4:
		timer->CCR4 = pulse;
//		TIM_SetCompare4(timer, pulse);

		break;

	}





}



