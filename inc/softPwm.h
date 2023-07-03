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



#ifndef INC_SOFTPWM_H_
#define INC_SOFTPWM_H_


//*************************************************
//includes
//*************************************************
#include "ports.h"

//*************************************************
//defines
//*************************************************

//*************************************************
//Types
//*************************************************



//*************************************************
//function prototypes
//*************************************************

/**
 * configures one PWM channel.
 * This will likely be called by sfdqPackets because of communication to set up a PWM channel.
 * It may also be called by application specific code
 * @param channel - indicates which channel to setup
 * @param output the pin to output the PWM signal on
 * @param the input trigger pin for One-shot mode
 */
void configSoftPwm(uint32_t channel, PortPin output);

/**
 * this sets the duty cycle of a particular PWM channel.
 * It may also do other stuff that we haven't thought of yet.
 */
void updateSoftPwm(uint32_t channel, float dutyCycle);
void setupSoftPwmAsOneShot(uint32_t channel, float time,  PortPin trigger, bool fallingNotRising);
void setupSoftPwmAsFollower(uint32_t channel, uint32_t channelToFollow, float gain);
void setupSoftPwmAsPwm(uint32_t channel, float frequencyHz);
void deconfigureSoftPwm(uint32_t channel);
/**
 * @return the number of soft PWM channels
 */
uint32_t getSoftPwmCount(void);
/**
 * The setpoint is the duty cycle when in PWM mode
 * it is time when in one shot mode
 * it is duty cycle when in follower mode
 * @return the setpoint of the specified channel.
 */
float getSoftPwmSetpoint(uint32_t channel);
/**
 * indicates if the specified channel is setup
 */
bool isSoftPwmConfigured(uint32_t channel);

bool hasOneShotFired(uint32_t channel);
bool isOneShotReset(uint32_t channel);

void resetOneShot(uint32_t channel);

void softPwmFastCode(void);
#endif /* INC_SOFTPWM_H_ */



