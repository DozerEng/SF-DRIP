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

#ifndef PWM_H_
#define PWM_H_


#include <stdbool.h>
#include "stm32f4xx_tim.h"
//#include "_processorGlobal.h"



/**
 * initialize the spi driver
 */

typedef enum {
	PWM_CH1,
	PWM_CH2,
	PWM_CH3,
	PWM_CH4,
} PwmChannel;
typedef enum {
	PWM_TIMER1,
	PWM_TIMER2,
	PWM_TIMER3,
	PWM_TIMER4,
	PWM_TIMER5,
	PWM_TIMER6,
	PWM_TIMER7,
	PWM_TIMER8,
	PWM_TIMER9,
	PWM_TIMER10,
	PWM_TIMER11,
	PWM_TIMER12,
	PWM_TIMER13,
	PWM_TIMER14,

} PwmTimer;
/**
 * 1000, 333 gives 100Hz
 */
void pwmInit(PwmTimer timer, uint32_t period, uint16_t prescaler);
void initPwm(PwmTimer timer, PwmChannel c, float duty);

void setPwm(PwmTimer timer, PwmChannel c, float duty);



#endif /* PWM_H_ */
