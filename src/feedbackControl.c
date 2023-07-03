/*
Copyright (c) 2019 STARFISH PRODUCT ENGINEERING INC.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "feedbackControl.h"
#include <math.h>
#include <stdbool.h>

#include <thermistors.h>
#include "adcManager.h"
#include "encoder.h"
#include "ad7172.h"
#include "dac.h"
#include "timeSync.h"
#include "brushlessMotor.h"
#include "sfdqPackets.h"
#include "portExpanderManager.h"
#include "stepperMotor.h"
#include "softPwm.h"
#include "appData.h"
#include "queue.h"
#include <streamingManager.h>
//#include "ports.h"



//*************************************************
//defines
//*************************************************

#define TRAJECTORY_QUEUE_SIZE 100
#define EPSILON (1e-7f)

//*************************************************
//Types
//*************************************************
// a trajectory is considered one transition from the current place and speed to a new place and speed, applying the specified acceleration
typedef enum {
     FB_QUEUED,
     FB_STARTING,
     FB_GOING,
     FB_DONE,
} TrajectoryState;

typedef struct {
     float position;//the target destination
     float velocity;// the target speed that we will have when we reach the destination
     uint32_t finishTime;//the time the trajectory is supposed to end
     TrajectoryState state;
} Trajectory;

typedef struct {
	FeedbackInputType inputType;
	uint8_t inputIndex;
	FeedbackOutputType outputType;
	uint8_t outputIndex;
	FeedbackCircularUnits units;
	bool useTrajectories;

	float actual;//the actual location of the system
	float trajectoryPosition;//the present position of the ongoing trajectory
	float trajectoryVelocity;//the present velocity of the ongoing trajectory
	uint32_t trajectoryEndTime;//the time that the trajectory will end
	float trajectoryTime;//the time of the current trajectory or the end of the last if there is no new one
	float trajectoryC0;
	float trajectoryC1;
	float trajectoryC2;
	float trajectoryC3;

	//    uint32_t finishTime;//targetTime for current trajectory

	float deltaTime;
	uint32_t adcNum;

	float x0;//this is the latest error input to the controller
	float x1;//this is the last error input
	float x2;//this is the last last error input
	float y0;//this is the output of the controller
	float y1;//the is the output of the controller last sample time
	float y2;//this is the output of the controller last last sample time
	float d1;//coeff for the z^-1 term of the y size of the z transform
	float d2;//coeff for the z^-2 term of the y side
	float c0;//coeff for the z^0 term of the x side
	float c1;//coeff for the z^-1 term of the x side
	float c2;//coeff for the z^-2 term of the x side
	Trajectory trajectories[TRAJECTORY_QUEUE_SIZE];
	uint32_t trajQueueFront;//index where trajectories are pulled off the queue. If this equals head then there is no trajectory in queue
	uint32_t trajQueueBack;//index where trajectories are added to queue
	uint32_t trajectoryLastEndTime;//last trajectory added end time
	float minOutput;
	float maxOutput;
	float minInput;
	float maxInput;
	float input;
	float arbitrationMax;
	/**
	* if zero then not limited. If -1 then limited in neg direction. If +1 then limited in pos direction
	*/
	int32_t inputLimited;
	PortPin minTravelLimitPin;
	PortPin maxTravelLimitPin;
	bool invertMinLimitPin;
	bool invertMaxLimitPin;
	bool passThrough;
	bool invertOutput;
	SinkSource inputSink;
	SinkSource outputSink;
	SinkSource errorSink;
	SinkSource setpointSink;



} FeedbackControlState;



//*************************************************
//Variables
//*************************************************


static FeedbackControlState m_states[NUM_FEEDBACK_CHANNELS];


//*************************************************
//function prototypes
//*************************************************

static void computeNextState(FeedbackControlState *state);

static void setOutput(uint32_t index);
static Trajectory* getCurrentTrajectory(FeedbackControlState *state);
static bool nextTrajectory(FeedbackControlState *state);
static void processTrajectory(FeedbackControlState *state);
static Trajectory* getLastAddedTrajectory(FeedbackControlState *state);
static void clip(FeedbackControlState* s);
static float forceFinite(float f);
static void testForBreak(uint32_t);
static float inLimits(FeedbackControlState *s, float output);
static bool isOn(FeedbackControlState *state);
static bool isMinTravelPinSet(FeedbackControlState *s);
static bool isMaxTravelPinSet(FeedbackControlState *s);
/**
 * wraps the input to keep it within circular bounds, if necessary
 */
static float wrap(float x, FeedbackCircularUnits units);
///**
// * computes the distance in circular units (deg, rad) from x0 to x1
// * @return negative if x1 is
// */
//static float circDistance(float x0, float x1, FeedbackCircularUnits units);

//*************************************************
//code
//*************************************************


/**
  * initialize module. Must be called at startup
  */
void initFeedback(void){
     for(int i = 0; i < NUM_FEEDBACK_CHANNELS; ++i){
    	 FeedbackControlState* s = &(m_states[i]);
         s->inputType = FB_INPUT_NULL;
         s->outputType = FB_OUTPUT_NULL;
         s->trajQueueFront = 0;
         s->trajQueueBack = 0;
         s->useTrajectories = true;
         s->inputLimited = 0;
         s->minTravelLimitPin = NULL_PIN;
         s->maxTravelLimitPin = NULL_PIN;
         s->invertMaxLimitPin = false;
         s->invertMinLimitPin = false;
         s->passThrough = false;
         s->units = FB_LINEAR;
         s->invertOutput = false;
     }
}
/**
 * sets up one feedback channel
 * output is computed using a biquad filter of the form:
 * y[n] = c0*x[n] + c1*x[n - 1] + c2*x[n - 2] - d1*y[n - 1] - d2*y[n - 2]
 *
 * sets up one feedback channel
 * @param inType - the type of input that will be used
 * @param inIndex - the index of the input channel
 * @param outType - the type of output that will be used. Usually this will be HI_POWER_OUTPUT
 * @param outIndex - the index of the output
 * @param minOut - the minimum allowed output value
 * @param maxOut - the maximum allowed output value
 * @param minIn - the minimum allowable input value. Exceeding this will cut the output
 * @param maxIn - the minimum allowable input value. Exceeding this will cut the output
 *
 */
void setupFeedback(uint8_t index, FeedbackInputType inType, uint8_t inIndex, FeedbackOutputType outType, uint8_t outIndex, float minOut, float maxOut, float minIn, float maxIn){
     if(index >= NUM_FEEDBACK_CHANNELS){
         return;
     }

     FeedbackControlState *s = &(m_states[index]);

     s->inputType = inType;

     s->outputType = outType;
     s->inputIndex = inIndex;
     s->outputIndex = outIndex;

     //reset stuff if the channel is disabled
     if(inType == FB_INPUT_NULL && outType == FB_OUTPUT_NULL){
//    	  s->trajQueueFront = 0;
//    	  s->trajQueueBack = 0;
    	  s->trajectoryEndTime = 0;
//    	  s->trajectoryPosition = 0;
    	  s->trajectoryLastEndTime = 0;
    	  s->trajectoryTime = 0;
//    	  testForBreak(index);
     }



     s->minOutput = minOut;
     s->maxOutput = maxOut;
     s->minInput = minIn;
     s->maxInput = maxIn;

     //reset trajectory queue
//     s->trajQueueTail = 0;
//     s->trajQueueHead = 0;



}
/**
 * Setup biquad parameters
 * output is computed using a biquad filter of the form:
 * y[n] = c0*x[n] + c1*x[n - 1] + c2*x[n - 2] - d1*y[n - 1] - d2*y[n - 2]
 * @param c0 - the z-1 constant term coefficient of the numerator
 * @param c1 - the s term coefficient of the numerator
 * @param c2 - the z-2 coefficient of the numerator
 * @param d1 - the z-1 term coefficient of the denominator
 * @param d2 - the z-2 term coefficient of the denominator
 */
void setFeedbackBiquad(uint8_t index, float c0, float c1, float c2, float d1, float d2){
	 if(index >= NUM_FEEDBACK_CHANNELS){
		 return;
	 }

	 FeedbackControlState *s = &(m_states[index]);

	 s->c0 = c0;
	 s->c1 = c1;
	 s->c2 = c2;
	 s->d1 = d1;
	 s->d2 = d2;
}
/**
 * setup PID parameters
 * @param index - the feedback channel to configure
 * @param kp - the proportional coefficient
 * @param ki - the integral coefficient
 * @param kd = the derivative coefficient
 */
void setFeedbackPid(uint8_t index, float kp, float ki, float kd){
	if(index >= NUM_FEEDBACK_CHANNELS){
		 return;
	}

	FeedbackControlState *s = &(m_states[index]);

	float ts = getFeedbackSampleTime();
	s->d1 = -1;
	s->d2 = 0;
	s->c0 = (kp + ki*ts/2 + kd/ts);
	s->c1 = (-kp + ki*ts/2 - 2*kd/ts);
	s->c2 = kd/ts;

}
/**
 * setup a PI loop
 * @param index - the feedback channel to configure
 * @param kp - the proportional coefficient
 * @param ki - the integral coefficient
 */
void setFeedbackPi(uint8_t index, float kp, float ki){
	FeedbackControlState *s = &(m_states[index]);

	float ts = getFeedbackSampleTime();
	s->d1 = -1;
	s->d2 = 0;
	s->c0 = (kp + ki*ts/2);
	s->c1 = (-kp + ki*ts/2);
	s->c2 = 0;
}

/**
 * setup a PI loop of the form Vc = gain*(s + 1/tau)/s
 * @param index - the feedback channel to configure
 * @param gain - the proportional coefficient
 * @param tau - the time constant of the zero
 */
void setFeedbackPiGainTau(uint8_t index, float gain, float tau){
	if(tau == 0.0f){
		setFeedbackP(index, gain);
	} else {
		setFeedbackPi(index, gain, gain/tau);
	}
}
/**
 * setup a simple proportional loop
 * @param index - the feedback channel to configure
 * @param kp - the proportional coefficient
 */
void setFeedbackP(uint8_t index, float kp){
	if(index >= NUM_FEEDBACK_CHANNELS){
		 return;
	}

	FeedbackControlState *s = &(m_states[index]);
	s->d1 = 0;
	s->d2 = 0;
	s->c0 = kp;
	s->c1 = 0;
	s->c2 = 0;


}

/**
  * computes the next output value. This does not clip the output based on the output limits. That must be done later
  */
static void computeNextState(FeedbackControlState *s){
	float input = getFeedbackInputOfType(s->inputType, s->inputIndex);
     //first advance all the vars
	s->input = wrap(input, s->units);
	setSinkSource(s->inputSink, s->input);
	s->x2 = s->x1;
	s->x1 = s->x0;
	//if in pass-through mode then input to biquad is the measured input alone
	//otherwise the input to the biquad is (-error) = setpoint - actual. Negative sign because it's negative feedback.
	float x0;
	if(!s->passThrough){
	 x0 = s->trajectoryPosition  - input;
	} else {
	 x0 = input;
	}


	s->x0 = wrap(x0, s->units);
	setSinkSource(s->errorSink, s->x0);

	s->y2 = s->y1;
	s->y1 = s->y0;

	s->x2 = forceFinite(s->x2);
	s->x1 = forceFinite(s->x1);
	s->x0 = forceFinite(s->x0);
	s->y1 = forceFinite(s->y1);
	s->y2 = forceFinite(s->y2);



	float output = 0;

	output += s->c0 * s->x0;
	output += s->c1 * s->x1;
	output += s->c2 * s->x2;
	output -= s->d1 * s->y1;
	output -= s->d2 * s->y2;
	output = forceFinite(output);


     s->y0 = output;

}

static float forceFinite(float f){
	float result = f;
	if(isnanf(f)){
		result = 0;
	} else if(isinff(f)){
		result = 0;
	}
	return result;
}


/**
  * this must be run once every sample time.
  * It computes the latest output values for all defined channels
  */
void calcFeedbackOutputs(void){
	FeedbackControlState *si;
	FeedbackControlState *sj;
     for(int i = 0; i < NUM_FEEDBACK_CHANNELS; ++i){

         si = &(m_states[i]);
         if(isOn(si)){
        	 if(si->useTrajectories){
        		 //don't process a trajectory if we're getting our setpoint some other way
        		 processTrajectory(si);
        	 }
             computeNextState(si);
             clip(si);
         }
     }
     //now arbitrate channels that might be set to the same output
     //this may not be the most efficient way to cover all of the possibilities

     for(int i = 0; i < NUM_FEEDBACK_CHANNELS; ++i){
    	 si = &(m_states[i]);
    	 if(isOn(si)){
    		 si->arbitrationMax = NAN;
    		 //only test configured channels
			 float limit = si->y0;
			 bool lowInLimit = si->inputLimited < 0;
			 bool highInLimit = si->inputLimited > 0;
			 bool oneChannelOutNeg = si->y0 < 0;
			 bool oneChannelOutPos = si->y0 > 0;
			 for(int j = 0; j < NUM_FEEDBACK_CHANNELS; ++j){
				 sj = &(m_states[j]);
				 if(j != i && isOn(sj)){
					 if(si->outputType == sj->outputType && si->outputIndex == sj->outputIndex){

						 //these channels are set to the same output so arbitrate
						 //for now assume that the one with the lowest control output is the winner

						 if(sj->inputLimited < 0){//if this channel is outside its input limits then turn off the linked channel
							 lowInLimit = true;

						 } else if(sj->inputLimited > 0){
							 highInLimit = true;
						 }
						 if(limit > (sj->y0)){
							limit = sj->y0;
						 }
						 if(sj->y0 > 0){
							 oneChannelOutPos = true;
						 } else if(sj->y0 < 0){
							 oneChannelOutNeg = true;
						 }
					 }
				 }
			 }
			 if(highInLimit && oneChannelOutPos){//if this channel is being driven positive and another is input limited positive then prepare to zero drive

				 si->arbitrationMax = 0;
			 } else if(lowInLimit && oneChannelOutNeg){//if this channel is being driven negative and another is input limited negative then don't drive this one

				 si->arbitrationMax = 0;
			 } else if(si->y0 > limit){//otherwise, limit this channel's output to the lowest channel output
				 si->arbitrationMax = limit;

			 }

		 }
     }



     //now set the outputs
     for(int i = 0; i < NUM_FEEDBACK_CHANNELS; ++i){

    	 //clip based on output min and max

    	 setOutput(i);
     }

}
/**
  * @return the last computed output
  */
float getOutput(uint8_t index){
	float result = NAN;
     if(index < NUM_FEEDBACK_CHANNELS){



		FeedbackControlState *state = &(m_states[index]);
		result = state->y0;
		if(state->inputLimited == 0 && finitef(state->arbitrationMax)){
			result = state->arbitrationMax;

		} else if(state->inputLimited != 0){

		} else {

		}
		 if(state->invertOutput){
			 result = -result;
		 }
     }

     return result;
}
/**
 * Indicates whether this channel is enabled or not
 */
static bool isOn(FeedbackControlState *state){
	return (state->inputType != FB_INPUT_NULL && state->outputType != FB_OUTPUT_NULL);
}

bool isConfigured(uint32_t i){
     bool result = false;
     if(i >= 0 || i < NUM_FEEDBACK_CHANNELS){

         FeedbackControlState *state = &(m_states[i]);



        result = isOn(state);
     }
     return result;
}




float getFeedbackInputOfType(FeedbackInputType type, uint32_t index){
     float result = NAN;
     //TODO: figure this out
     switch(type){

     case FB_INPUT_NULL:
         result = NAN;
         break;
     case FB_INPUT_LOW_RES_ADC:
         result = getAdcValue(index);
         break;
     case FB_INPUT_ENCODER:
         result = getEncoderDistance(index);
         break;
     case FB_INPUT_HI_RES_ADC:
         result = ad7172GetValue(index);
         break;
     case FB_INPUT_TEMPERATURE:
         result = getThermistorTemperature(index);
         break;
     case FB_INPUT_ENCODER_VELOCITY:
         result = getEncoderVelocity(index);
         break;
     case FB_INPUT_BRUSHLESS_MOTOR:
    	 switch(index){
    	 case 0:
    		 result = getBrushlessPosition();
    		 break;
    	 case 1:
    		 result = getBrushlessRevPerSec();
    		 break;
    	 default:
    		 break;
    	 }

    	 break;
	 case FB_INPUT_RX_APP_DATA:
		 result = getRxAppData(index);
		 break;
	 case FB_INPUT_TX_APP_DATA:
		 result = getTxAppData(index);
		 break;
     case FB_INPUT_OPEN_LOOP:
         result = 0;
         break;
     case FB_INPUT_STREAM_QUEUE:
    	 result = peekLastSampleAdded(index);
    	 break;
     default:
		 result = NAN;
		 break;
     }

     return result;
}
/**
  * gets the drive value for one of the PWM outputs if there is a
feedback channel configured for it
  * @return the drive value if there's a feedback channel configured or
NAN if not
  */
float getOutputForPWM(uint8_t index){
     float result = 0;

     int32_t chan = getChannelWithSpecifiedPWMOutput(index);
     if(chan >= 0){
    	 FeedbackControlState *s = &(m_states[chan]);
    	 result = getOutput(chan);
    	 //if we're driving a bipolar output then centre on 0.5 instead of 0.0
    	 if(s->outputType == FB_OUTPUT_HI_POWER_BIPOLAR){
    		 result += 0.5f;
    	 }

     }
     return result;
}
int32_t getChannelWithSpecifiedPWMOutput(uint8_t index){
     int32_t result = -1;
     for(uint8_t i = 0; i < NUM_FEEDBACK_CHANNELS; ++i){
         FeedbackControlState s = m_states[i];
         if(s.outputIndex == index){
             if(s.outputType == FB_OUTPUT_HI_POWER_UNIPOLAR){
                 result = i;
                 break;
             } else if(s.outputType == FB_OUTPUT_HI_POWER_BIPOLAR){
                 result = i;
                 break;
             }

         }

     }
     return result;
}
static void setOutput(uint32_t index){
	FeedbackControlState *state = &(m_states[index]);
	if(!isOn(state)){
		return;
	}
	float op = getOutput(index);
	setSinkSource(state->outputSink, op);
     switch(state->outputType){
     case FB_OUTPUT_NULL:
         //this is not configured to set anything
         break;
     case FB_OUTPUT_HI_POWER_BIPOLAR:
         //this will be set with the sig gen code so don't work
         break;
     case FB_OUTPUT_HI_POWER_UNIPOLAR:
         //this will be set with the sig gen code so don't work
         break;
     case FB_OUTPUT_STEPPER:
    	 computeStepperOutputs(state->outputIndex, op, getFeedbackSampleTime());
         break;
     case FB_OUTPUT_STEPPER_VELOCITY:
    	 computeStepperOutputsFromVelocity(state->outputIndex, op, getFeedbackSampleTime());
    	 break;
     case FB_OUTPUT_DAC:
         //set the dac here
         setDacAsFloat(state->outputIndex, op);
         break;
     case FB_OUTPUT_BRUSHLESS_MOTOR:
         //set the motor here
         setBrushlessDrive(op);
         break;
     case FB_OUTPUT_FEEDBACK:
    	 ;
    	 //set another feedback channel with this channel's output
    	 //note that that channel will stop computing trajectories until one is set for it
    	 //instead it will rely on this channel to set its setpoint
    	 FeedbackControlState *fb = &(m_states[state->outputIndex]);
    	 fb->useTrajectories = false;
    	 fb->trajectoryPosition = op;
    	 break;
     case FB_OUTPUT_FEEDBACK_MAX:
    	 ;
    	 //set the max output of another feedback channel with this channel's output
    	 fb = &(m_states[state->outputIndex]);
		 fb->maxOutput = op;
		 break;
     case FB_OUTPUT_PORT_EXPANDER:
    	 ;
    	 uint32_t i = (state->outputIndex >> 4) & 0xf;
    	 uint32_t p = (state->outputIndex) & 0xf;
    	 uint8_t pm = 1 << p;
    	 PinConfig pc = op > 0.5 ? OUTPUT_SET : OUTPUT_CLEAR;

    	 setupPortExpanderPin(i, pm, pc);
    	 break;
     case FB_OUTPUT_APP_DATA:
    	 setTxAppData(state->outputIndex, op);
    	 break;
     case FB_OUTPUT_SOFT_PWM:
    	 updateSoftPwm(state->outputIndex, op);
    	 break;


     }
}
static void clip(FeedbackControlState *s){
	float setpoint = s->y0;

	FeedbackOutputType type = s->outputType;
	float min = -INFINITY;
	float max = INFINITY;
	float result = setpoint;
	switch(type){
	case FB_OUTPUT_NULL:
	case FB_OUTPUT_STEPPER:
	case FB_OUTPUT_STEPPER_VELOCITY:
		//this is not configured to set anything
		break;
	case FB_OUTPUT_BRUSHLESS_MOTOR:
	case FB_OUTPUT_HI_POWER_BIPOLAR:
	case FB_OUTPUT_FEEDBACK:
	case FB_OUTPUT_FEEDBACK_MAX:
	case FB_OUTPUT_SOFT_PWM:
		min = -1;
		max = 1;

		break;
	case FB_OUTPUT_DAC:
	case FB_OUTPUT_PORT_EXPANDER:
	case FB_OUTPUT_HI_POWER_UNIPOLAR:
		min = 0;
		max = 1.0;
		break;
	case FB_OUTPUT_APP_DATA:
		break;
	}

	if(s->minOutput > min){
		min = s->minOutput;
	}
	if(s->maxOutput < max){
		max = s->maxOutput;
	}
//	if(finitef(s->arbitrationMax)){
//		result = s->arbitrationMax;
//
//	}


	if(result > max){
		result = max;
	} else if(result < min){
		result = min;
	}
	result = inLimits(s, result);
	s->y0 = result;
}

/**
 * checks the input value and makes sure it is within the set limits
 * If not then it zeros whichever direction will cause it to cross that limit
 * Note that if the feedback loop is setup with positive instead of negative feedback then this won't prevent crashing
 * if it crosses over a limit then the output is cut
 */
static float inLimits(FeedbackControlState *s, float output){
	float result = output;
	float minErr = wrap(s->input - s->minInput, s->units);
	float maxErr = wrap(s->input - s->maxInput, s->units);
	if(minErr < 0){
		if(output < 0){
			result = 0;
		}
		s->inputLimited = -1;
	} else if(maxErr > 0){
		if(output > 0){
			result = 0;
		}
		s->inputLimited = +1;
	} else {
		s->inputLimited = 0;
	}
	return result;
}

/**
  * @return the last added trajectory. If queue is empty then NULL
  */
static Trajectory* getLastAddedTrajectory(FeedbackControlState *state){

	Trajectory* result = NULL;

	 //make local pointers to queue front and back
	 uint32_t* front = &(state->trajQueueFront);
	 uint32_t* back = &(state->trajQueueBack);

	if(isQueueNotEmpty(front, back, TRAJECTORY_QUEUE_SIZE)){
		result = &(state->trajectories[getLastAddedQueueItemIndex(front, back, TRAJECTORY_QUEUE_SIZE)]);

	}

	return result;
}


/**
  * @return current trajectory or null if none
  */
static Trajectory* getCurrentTrajectory(FeedbackControlState *state){

     uint32_t* front = &(state->trajQueueFront);
     uint32_t* back = &(state->trajQueueBack);
     Trajectory* t = NULL;

     if(isQueueNotEmpty(front, back, TRAJECTORY_QUEUE_SIZE)){
    	 t = &(state->trajectories[*front]);
     }


     return t;
}
/**
  * advances to the next trajectory in the queue to execute
  * @return true if it did advance, false if there's nothing to advance to
  */
static bool nextTrajectory(FeedbackControlState *state){
     bool result = false;
     uint32_t* front = &(state->trajQueueFront);
     uint32_t* back = &(state->trajQueueBack);

     if(isQueueNotEmpty(front, back, TRAJECTORY_QUEUE_SIZE)){//only do anything if there is a new trajectory
         Trajectory* ot = getCurrentTrajectory(state);
         ot->state = FB_DONE;
         state->trajectoryTime = ot->finishTime;

         doneWithQueueFront(front, back, TRAJECTORY_QUEUE_SIZE);

     }

     if(isQueueNotEmpty(front, back, TRAJECTORY_QUEUE_SIZE)){

         result = true;

         Trajectory* t = getCurrentTrajectory(state);
         if(t != NULL && t->state != FB_DONE){
//            t->state = STARTING;
         } else {
        	 //this should never happen!
         }
     }
     return result;
}
/**
  * adds a new trajectory to the queue
  * The new trajectory will be placed in queue after the last item with
a sooner host time
  * Every remaining item in the queue will be cleared.
  * If new time is equal to an existing time and we're at the queue tail
then ignore
  * Note that if the queue is full then this will simply spill a new
waypoint if there's not enough room
  * It's up to the comm protocol to acknowledge whether the trajectory
was added
  * @param setpoint the target location
  * @param speed the target speed to travel to the setpoint
  * @param finishTime the host time when the trajectory should finish
  */
void trajectoryAddWaypoint(uint8_t index, float position, float velocity, uint32_t finishTime){
     if(index >= NUM_FEEDBACK_CHANNELS){
         return;
     }

     FeedbackControlState *state = &(m_states[index]);
     //scan for last trajectory that is sooner than the new one
     state->useTrajectories = true;

     //make local pointers to queue front and back
     uint32_t* front = &(state->trajQueueFront);
     uint32_t* back = &(state->trajQueueBack);



     if(isQueueNotFull(front, back, TRAJECTORY_QUEUE_SIZE)){

         //get last added trajectory's end time
    	 uint32_t currentEndTime = getLastEndTime(index);


         if(compareTimesMilliSec(finishTime, currentEndTime) > 0){//make sure the finish time is in the future


             //it is assumed that the target position and velocity are sensible
             //it is up the the java app or the user to ensure this
             state->trajectories[*back].state = FB_QUEUED;
             state->trajectories[*back].position = position;
             state->trajectories[*back].velocity = velocity;
             state->trajectories[*back].finishTime = finishTime;
             state->trajectoryLastEndTime = finishTime;

             justAddedToQueueBack(front, back, TRAJECTORY_QUEUE_SIZE);
         } else {
        	 asm("nop");
         }
     } else {
     }
}




/**
  * @return the last added trajectory end time. This is needed to
construct the trajectory response payload
  */
uint32_t getLastEndTime(uint8_t index){
     if(index >= NUM_FEEDBACK_CHANNELS){
		 return 0;
	 }

	 FeedbackControlState *state = &(m_states[index]);
	 uint32_t ct = getLocalTimeMillis();
	 uint32_t tt = state->trajectoryLastEndTime;
	 if(getTrajectoryQueueLength(index) != 0){
		 //only check the last trajectory end time if there is a current trajectory
		 if(compareTimesMilliSec(ct, tt) > 0){
			 tt = ct;
		 }
	 } else {
		 tt = ct;
	 }
	 return tt;
}
/**
 * @return the last added trajectory end time. This is needed to construct the trajectory response payload
 */
float getLastSetpoint(uint8_t index){
	float result = NAN;
    if(index < NUM_FEEDBACK_CHANNELS){


		 FeedbackControlState *state = &(m_states[index]);
		 Trajectory* t = getLastAddedTrajectory(state);
		 if(t != NULL){
			 result = t->position;
		 } else {
			 result = state->trajectoryPosition;
		 }

    }

	 return result;
}
/**
 * get the number of items in the trajectory queue of the specified feedback channel.
 * @return the number of items or zero if the specified channel is not enabled.
 */
uint32_t getTrajectoryQueueLength(uint8_t index){
     uint32_t result = 0;
	 if(index < NUM_FEEDBACK_CHANNELS){

		 FeedbackControlState *state = &(m_states[index]);
		 if(isOn(state)){
			 //make local pointers to queue front and back
			 uint32_t* front = &(state->trajQueueFront);
			 uint32_t* back = &(state->trajQueueBack);

			 result = getQueueItemCount(front, back, TRAJECTORY_QUEUE_SIZE);
		 }

	 }
	 return result;
}
/**
  * @return the current setpoint position. this is needed to construct
the trajectory response payload. NaN if not configured
  */
float getCurrentSetpoint(uint8_t index){
     float result = NAN;
     if(index < NUM_FEEDBACK_CHANNELS){

         FeedbackControlState *state = &(m_states[index]);
         if(isOn(state)){

             result = state->trajectoryPosition;
         }

     }
     return result;

}
///**
//  * @return the number of feedback channels
//  */
//uint8_t getFeedbackChannelNum(void){
//     return NUM_FEEDBACK_CHANNELS;
//}
/**
  * indicates if any channels have been configured. This is useful for
sending response packets
  */
bool anyTrajectoriesConfigured(void){
     bool result = false;
     for(uint8_t i = 0; i < NUM_FEEDBACK_CHANNELS; ++i){
//        if(finitef(getCurrentSetpoint(i))){
         if(isConfigured(i)){
             result = true;
             break;
         }
     }
     return result;
}
/**
  * chip away at current trajectory
  */
static void processTrajectory(FeedbackControlState *s){




     //first compute a bunch of stuff
     //current traj
     Trajectory *t = getCurrentTrajectory(s);
//     Trajectory *tLast = t;

     uint32_t currentTime = getLocalTimeMillis();

     if(t == NULL){
         //there's no new trajectory
//        s->trajectoryPosition = s->trajectoryPosition;//don't change current position because there's nowhere to go
         s->trajectoryVelocity = 0;
     } else {
         if(t->state == FB_QUEUED){
             //we're just starting a trajectory so calc a bunch of stuff to start with
             uint32_t tNow = s->trajectoryEndTime;
             //if our velocity is zero then it's ok to reset the time
             //similarly if our known time is zero, then we're likely only just enabled
             if(s->trajectoryVelocity == 0.0f || tNow == 0){
                 tNow = getLocalTimeMillis();
             }


             t->state = FB_GOING;



             //We have a current position,current time, current velocity
             //We have a position and velocity that we want to achieve at a certain time in the future
             //assume quadratic form, i.e. constant acceleration for each trajectory
             //t0 = starting time (this is the negative delta from the start to the finish, because the finish time is assumed to be zero)
             //x0 = starting pos
             //v0 = starting velocity
             //t1 = finishing time (assumed to be zero)
             //x1 = finishing pos
             //v1 = finishing velocity
             //x = c0 + c1*t + c2*t^2 + c3*t^3
             //v = x' = c1 + 2*c2*t + 3*c3*t^2
             //a = x'' = 2*c2 + 6*c3*t
             //assume that at the start of each trajectory the time is offset so that the target time (t1) is zero
             //x0 = c0 + c1*t0 + c2*t0^2 + c3*t0^3
             //x1 = c0 + c1*t1 + c2*t1^2 + c3*t1^3
             //v0 = c1 + 2*c2*t0 + 3*c3*t0^2
             //v1 = c1 + 2*c2*t1 + 3*c3*t0^2
             //c3 = (v0 + v1)/t0^2 + (x1 - x0)*2/t0^3
             //c2 = (x0 - x1)*3/t0^2 - (2*v1 + v0)/t0
             //c1 = v1
             //c0 = x1
             //
             //a0 =
             uint32_t tTarget = t->finishTime;
             float x0 = s->trajectoryPosition;
             float v0 = s->trajectoryVelocity;
             float t0 = compareTimesMilliSec(tNow, tTarget)*1e-3;
//             float t1 = 0;
             float v1 = t->velocity;
             float x1 = t->position;
             float invT0 = 1.0/t0;
             float c0, c1, c2, c3;
             if(finitef(v1)){//this will be NAN if it's to be a
            	 //a cubic trajectory segment


				 c3 = (v0+ v1 + 2*invT0*(x1 - x0))*invT0*invT0;
				 c2 = (3*invT0*(x0 - x1) - (2*v1 + v0))*invT0;
				 c1 = v1;
				 c0 = x1;
             } else {
            	 //a linear trajectory segment
            	 c3 = 0;
            	 c2 = 0;
            	 c1 = (x0 - x1)*invT0;
            	 c0 = x1;
             }
			 if(!finitef(c3+c2+c1+c0)){
				 c3 = 0;
				 c2 = 0;
				 c1 = 0;
				 c0 = x1;
			 }

			 s->trajectoryC3 = c3;
			 s->trajectoryC2 = c2;
			 s->trajectoryC1 = c1;
			 s->trajectoryC0 = c0;
			 s->trajectoryEndTime =tTarget;

         }
		float secondsToFinish = t == NULL ? 0 : compareTimesMilliSec(t->finishTime, currentTime)*1e-3;
		float newPos;
		float newVelocity;
		float tNow = -secondsToFinish;
		if(secondsToFinish <= 0){
        	 newPos = t->position;
        	 newVelocity = t->velocity;

			 //I guess we're at our destination!
			 nextTrajectory(s);
         } else {
			//x = c0 + c1*t + c2*t^2
			//v = c1 + 2*c2*t
			float c0 = s->trajectoryC0;
			float c1 = s->trajectoryC1;
			float c2 = s->trajectoryC2;
			float c3 = s->trajectoryC3;



			newPos = c0 + (c1 + (c2 + c3*tNow)*tNow)*tNow;
			newVelocity = c1 + (2*c2 + 3.0*c3*tNow)*tNow;


         }

//		if(newPos > s->maxOutput || newPos < s->minOutput){
//			asm("nop");
//		}
		//check that we're not running into the endstops
		if(newVelocity < 0){
			if(isMinTravelPinSet(s)){
				 //we're traveling towards min travel and the limit pin is set
				 //so
				 newPos = s->trajectoryPosition;
				 newVelocity = 0;
			}
		} else {
			if(isMaxTravelPinSet(s)){
				//we're traveling towards max travel and the limit pin is set
				 newPos = s->trajectoryPosition;
				 newVelocity = 0;
			}
		}
		s->trajectoryPosition = newPos;

		s->trajectoryVelocity = newVelocity;
		s->trajectoryTime = tNow;
		setSinkSource(s->setpointSink, newPos);
     }





}

float getFeedbackInput(uint32_t index){
	 float result = NAN;
	 if(index < NUM_FEEDBACK_CHANNELS){

		 FeedbackControlState *state = &(m_states[index]);
		 if(isOn(state)){

			 result = state->input;
		 }

	 }
	 return result;
}


float getFeedbackOutput(uint32_t index){
	 float result = NAN;
	 if(index < NUM_FEEDBACK_CHANNELS){

		 FeedbackControlState *state = &(m_states[index]);
		 if(isConfigured(index)){

			 result = state->y0;
		 }

	 }
	 return result;
}

/**
 * @return the current error of this channel
 */
float getFeedbackError(uint32_t index){
	 float result = NAN;
	 if(index < NUM_FEEDBACK_CHANNELS){

		 FeedbackControlState *state = &(m_states[index]);
		 if(isConfigured(index)){

			 result = state->x0;
		 }

	 }
	 return result;
}

float getFeedbackSampleTime(void){
	return FEEDBACK_SAMPLE_TIME_US*1e-6f;
}
uint32_t getFeedbackSampleTimeUs(void){
	return FEEDBACK_SAMPLE_TIME_US;
}
/**
 * a handy function to put a break but only if we're in the right fb channel
 */
static void testForBreak(uint32_t i){
	if(i == 0){
		asm("nop");
	}
}
static bool isMinTravelPinSet(FeedbackControlState *s){
	bool result = false;
	if(s->minTravelLimitPin != NULL_PIN){
		bool pinState = isPinInputSet(s->minTravelLimitPin);
		result = pinState != s->invertMinLimitPin;
	}
	return result;
}
static bool isMaxTravelPinSet(FeedbackControlState *s){
	bool result = false;
	if(s->maxTravelLimitPin != NULL_PIN){
		bool pinState = isPinInputSet(s->maxTravelLimitPin);
		result = pinState != s->invertMaxLimitPin;
	}
	return result;
}


/**
 * sets up the minimum travel limit sensor pins for a specific feedback channel
 * @param index can be up to NUM_FEEDBACK_CHANNELS - 1
 * @param minPin the pin to check for minimum position. A rising (or falling if invert true) edge is assumed to occur at exact home spot. Set to NULL_PIN to disable.
 * @param invertMinPin false = rising edge, true = falling edge
 * @param maxPin the pin to check for maximum position. A rising (or falling if invert true) edge is assumed to occur at exact home spot. Set to NULL_PIN to disable.
 * @param invertMaxPin false = rising edge, true = falling edge
 */
void configFeedbackTravelLimitPins(uint32_t index, PortPin minPin, bool invertMinPin, PortPin maxPin, bool invertMaxPin){
	if(index < NUM_FEEDBACK_CHANNELS){
		FeedbackControlState *s = &(m_states[index]);
		s->minTravelLimitPin = minPin;
		s->invertMinLimitPin = invertMinPin;
		s->maxTravelLimitPin = maxPin;
		s->invertMaxLimitPin = invertMaxPin;
	}
}
/**
 * sets the specified channel to pass through mode.
 * If in passThrough, then input to biquad = actual, else input = setpoint - actual.
 * This allows the channel to simply filter an input and pass it through to an output channel.
 * For instance, app data could be routed to a DAC output after filtering by the biquad.
 *
 * @param index can be up to NUM_FEEDBACK_CHANNELS - 1
 * @param enable 0 - normal mode where biquad input = setpoint - actual; 1 - passthrough mode where biquad input = actual.
 */
void setFeedbackPassThrough(uint32_t index, bool enable){
	if(index < NUM_FEEDBACK_CHANNELS){
		FeedbackControlState *s = &(m_states[index]);
		s->passThrough = enable;
	}
}
/**
 * Sets whether the controller will use circular units or not.
 * When using circular units, the error will be confined to +/-180 degrees or +/- PI radians. Any value outside of this range will be wrapped around into the range.
 * @param units specifies whether to use degrees or radians, or to not wrap at all because the units are linear.
 */
void setFeedbackUnits(uint32_t index, FeedbackCircularUnits units){
	if(index >= NUM_FEEDBACK_CHANNELS){
		 return;
	 }

	 FeedbackControlState *s = &(m_states[index]);

	 switch(units){
	 case FB_RADIANS:
	 case FB_DEGREES:
	 case FB_LINEAR:
		 s->units = units;
		 break;
	 default:
		 s->units = FB_LINEAR;
		 break;
	 }

}

void setFeedbackOutputInverted(uint32_t index, bool invert){
	if(index >= NUM_FEEDBACK_CHANNELS){
		 return;
	 }

	 FeedbackControlState *s = &(m_states[index]);
	 s->invertOutput = invert;

}

/**
 * wraps the input to keep it within circular bounds, if necessary.
 */
static float wrap(float x, FeedbackCircularUnits units){
	float result = x;
	if(units != FB_LINEAR){
		float range = units == FB_RADIANS ? M_TWOPI : 360.0f;
		float oneOverRange = units == FB_RADIANS ? (1.0/M_TWOPI) : (1.0f/360.0f);
		float newX = x;
		newX *= oneOverRange;
		newX += 0.5f;

		if(newX < 0.0f || newX >= 1.0f){
			//now we wrap to >= 0 and < 1
			newX -= floorf(newX);

		}

		newX -= 0.5f;
		newX *= range;
		result = newX;


	}
	return result;

}

/**
 * Sets some sinks for useful brushless motor info
 * @param channel the channel of interest
 * @param index 0,1,2,3 = input, output, error, setpoint.
 * @param sink the sink
 *
 */
void setFeedbackSink(uint32_t channel, uint32_t index, SinkSource sink){
	if(channel >= NUM_FEEDBACK_CHANNELS){
		 return;
	 }

	FeedbackControlState *s = &(m_states[channel]);
	switch(index){
	case 0:
		s->inputSink = sink;
		break;
	case 1:
		s->outputSink = sink;
		break;
	case 2:
		s->errorSink = sink;
		break;
	case 3:
		s->setpointSink = sink;
		break;
	default:
		break;
	}

}

/**
 * gets some sinks for useful brushless motor info
 * @param channel the feedback channel of interest
 * @param index 0,1,2,3 = input, output, error, setpoint.
 * @return sink the sink
 *
 */
SinkSource getFeedbackSink(uint32_t channel, uint32_t index){
	if(channel >= NUM_FEEDBACK_CHANNELS){
		 return NULL_SINK_SOURCE;
	}

	FeedbackControlState *s = &(m_states[channel]);
	SinkSource result = NULL_SINK_SOURCE;
	switch(index){
	case 0:
		result = s->inputSink;
		break;
	case 1:
		result = s->outputSink;
		break;
	case 2:
		result = s->errorSink;
		break;
	case 3:
		result = s->setpointSink;
		break;
	default:
		break;
	}
	return result;
}
//static float circDistance(float x, FeedbackCircularUnits units){
//
//}
