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
#ifndef INC_STEPPERMOTOR_H_
#define INC_STEPPERMOTOR_H_



//*************************************************
//includes
//*************************************************
#include <stdint.h>
#include <stdbool.h>
#include "ports.h"
#include "feedbackControl.h"


//*************************************************
//defines
//*************************************************
#define STEPPER_CHANNELS NUM_FEEDBACK_CHANNELS

//*************************************************
//Types
//*************************************************
typedef enum {
	STEPPER_UNCONFIG,
	STEPPER_PWM,
	STEPPER_IO_AB,
	STEPPER_IO_STEP_DIR
} StepperType;

typedef enum {
	STEPPER_STATE_UNCONFIGURED = 0,
	STEPPER_STATE_IDLE = 2,
	STEPPER_STATE_MOVING = 3,
	STEPPER_STATE_ZEROING = 8,
	STEPPER_STATE_EOT = 9,
	STEPPER_STATE_HOME = 10

} StepperState;

typedef enum {
	ON_IDLE_FULL_DRIVE = 0,
	ON_IDLE_HALF_DRIVE = 1,
	ON_IDLE_QUARTER_DRIVE = 2,
	ON_IDLE_NO_DRIVE = 3,
} StepperOnIdle;

//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
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
void configStepper(uint32_t index, StepperType type, float distancePerStep, float pwmFraction);

/**
 * Configures the output pins for this channel.
 * This only applies to STEPPER_IO_AB and STEPPER_IO_STEP_DIR stepper types
 */
void configStepperOutputPins(uint32_t index, PortPin pin1, PortPin pin2);

bool isStepperConfigured(uint32_t index);
void stepperFastCode(void);
void computeStepperOutputs(uint32_t index, float position, float deltaTime);
void computeStepperOutputsFromVelocity(uint32_t index, float velocity, float deltaTime);
/**
 * @return the specified PWM output. 0 if PWM stepper not configured.
 */
float getStepperPwmOutputs(uint32_t outputIndex);
void initSteppers(void);
StepperState getStepperState(uint32_t index);
//uint32_t getStepperChannelCount(void);
void setStepperOnIdle(uint32_t index, StepperOnIdle soi);
/**
 * When zeroing, a channel will grab the current setpoint and set its current position to it
 * @param index the stepper channel to set zeroing
 * @param if true then the specified channel is zeroing.
 */
void setStepperZeroing(uint32_t index, bool z);
float getStepperPosition(uint32_t index);
float getStepperVelocity(uint32_t index);
bool isStepperIdle(uint32_t index);

#endif /* INC_STEPPERMOTOR_H_ */
