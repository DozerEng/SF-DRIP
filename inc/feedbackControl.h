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

#ifndef FEEDBACKCONTROL_H_
#define FEEDBACKCONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "ports.h"
#include <sinkSource.h>


#define FEEDBACK_SAMPLE_TIME_US 1000
#define NUM_FEEDBACK_CHANNELS 8


typedef enum {

	FB_INPUT_NULL = 0,
	FB_INPUT_LOW_RES_ADC = 1,
	FB_INPUT_TEMPERATURE = 2,
	FB_INPUT_ENCODER = 3,
	FB_INPUT_ENCODER_VELOCITY = 4,
	FB_INPUT_HI_RES_ADC = 5,
	FB_INPUT_OPEN_LOOP = 6,
	FB_INPUT_BRUSHLESS_MOTOR = 7,
	FB_INPUT_RX_APP_DATA = 8,
	FB_INPUT_TX_APP_DATA = 9,
	FB_INPUT_STREAM_QUEUE = 10, //this is assumed to be driven from some other module, that will manually stuff the queue
	FB_INPUT_QUERY = 255,

} FeedbackInputType;

typedef enum {
	FB_OUTPUT_NULL = 0,
	FB_OUTPUT_HI_POWER_UNIPOLAR = 1,
	FB_OUTPUT_DAC = 2,
	FB_OUTPUT_BRUSHLESS_MOTOR = 3,
	FB_OUTPUT_HI_POWER_BIPOLAR = 4,
	FB_OUTPUT_STEPPER = 5,
	FB_OUTPUT_FEEDBACK = 6,
	FB_OUTPUT_FEEDBACK_MAX = 7,
	FB_OUTPUT_PORT_EXPANDER = 8,
	FB_OUTPUT_APP_DATA = 9,
	FB_OUTPUT_SOFT_PWM = 10,
	FB_OUTPUT_STEPPER_VELOCITY = 11
} FeedbackOutputType;

typedef enum {
	FB_LINEAR = 0,
	FB_RADIANS = 2,
	FB_DEGREES = 1

} FeedbackCircularUnits;

/**
 * initialize module. Must be called at startup
 */
void initFeedback(void);
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
void setupFeedback(uint8_t index, FeedbackInputType inType, uint8_t inIndex, FeedbackOutputType outType, uint8_t outIndex, float minOut, float maxOut, float minIn, float maxIn);

/**
 * Setup biquad parameters
 * output is computed using a biquad filter of the form:
 * y[n] = c0*x[n] + c1*x[n - 1] + c2*x[n - 2] - d1*y[n - 1] - d2*y[n - 2]
 * @param index - the feedback channel to configure
 * @param c0 - the z-1 constant term coefficient of the numerator
 * @param c1 - the s term coefficient of the numerator
 * @param c2 - the z-2 coefficient of the numerator
 * @param d1 - the z-1 term coefficient of the denominator
 * @param d2 - the z-2 term coefficient of the denominator
 */
void setFeedbackBiquad(uint8_t index, float c0, float c1, float c2, float d1, float d2);
/**
 * setup PID parameters
 * @param index - the feedback channel to configure
 * @param kp - the proportional coefficient
 * @param ki - the integral coefficient
 * @param kd = the derivative coefficient
 */
void setFeedbackPid(uint8_t index, float kp, float ki, float kd);
/**
 * setup a PI loop
 * @param index - the feedback channel to configure
 * @param kp - the proportional coefficient
 * @param ki - the integral coefficient
 */
void setFeedbackPi(uint8_t index, float kp, float ki);
/**
 * setup a PI loop of the form Vc = gain*(s + 1/tau)/s
 * @param index - the feedback channel to configure
 * @param gain - the proportional coefficient
 * @param tau - the time constant of the zero
 */
void setFeedbackPiGainTau(uint8_t index, float gain, float tau);
/**
 * setup a simple proportional loop
 * @param index - the feedback channel to configure
 * @param kp - the proportional coefficient
 */
void setFeedbackP(uint8_t index, float kp);

/**
 * this must be run once every sample time.
 * It computes the latest output values for all defined channels
 */
void calcFeedbackOutputs(void);
/**
 * @return the last computed output
 */
float getOutput(uint8_t index);
int32_t getChannelWithSpecifiedPWMOutput(uint8_t index);
/**
 * gets the drive value for one of the PWM outputs if there is a feedback channel configured for it
 * @return the drive value if there's a feedback channel configured or NAN if not
 */
float getOutputForPWM(uint8_t index);
/**
 * adds a new trajectory to the queue
 * The new trajectory will be placed in queue after the last item with a sooner host time
 * Every remaining item in the queue will be cleared.
 * If new time is equal to an existing time and we're at the queue tail then ignore
 * @param setpoint the target location
 * @param speed the target speed that should occur by the time we reach the setpoint location. If this is NAN then assume a linear trajectory
 * @param finishTime the host time when the trajectory should finish
 */
void trajectoryAddWaypoint(uint8_t index, float position, float velocity, uint32_t finishTime);

/**
 * @return the last added trajectory end time. This is needed to construct the trajectory response payload
 */
uint32_t getLastEndTime(uint8_t index);
/**
 * @return the last added trajectory position setpoint
 */
float getLastSetpoint(uint8_t index);
/**
 * @return the current setpoint position. this is needed to construct the trajectory response payload. NaN if not configured
 */
float getCurrentSetpoint(uint8_t index);
///**
// * @return the number of feedback channels
// */
//uint8_t getFeedbackChannelNum(void);
/**
 * indicates if any channels have been configured. This is useful for sending response packets
 */
bool anyTrajectoriesConfigured(void);
/**
 * get the number of items in the trajectory queue of the specified feedback channel.
 * @return the number of items or zero if the specified channel is not enabled.
 */
uint32_t getTrajectoryQueueLength(uint8_t index);
bool isConfigured(uint32_t i);
float getFeedbackSampleTime(void);
uint32_t getFeedbackSampleTimeUs(void);
float getFeedbackInput(uint32_t index);
float getFeedbackOutput(uint32_t index);
void setFeedbackOutputInverted(uint32_t index, bool invert);
/**
 * sets up the minimum travel limit sensor pins for a specific feedback channel
 * @param index can be up to NUM_FEEDBACK_CHANNELS - 1
 * @param minPin the pin to check for minimum position. A rising (or falling if invert true) edge is assumed to occur at exact home spot. Set to NULL_PIN to disable.
 * @param invertMinPin false = rising edge, true = falling edge
 * @param maxPin the pin to check for maximum position. A rising (or falling if invert true) edge is assumed to occur at exact home spot. Set to NULL_PIN to disable.
 * @param invertMaxPin false = rising edge, true = falling edge
 */
void configFeedbackTravelLimitPins(uint32_t index, PortPin minPin, bool invertMinPin, PortPin maxPin, bool invertMaxPin);
/**
 * sets the specified channel to pass through mode.
 * If in passThrough, then input to biquad = actual, else input = setpoint - actual.
 * This allows the channel to simply filter an input and pass it through to an output channel.
 * For instance, app data could be routed to a DAC output after filtering by the biquad.
 *
 * @param index can be up to NUM_FEEDBACK_CHANNELS - 1
 * @param enable 0 - normal mode where biquad input = setpoint - actual; 1 - passthrough mode where biquad input = actual.
 */
void setFeedbackPassThrough(uint32_t index, bool enable);
/**
 * Sets whether the controller will use circular units or not.
 * When using circular units, the error will be confined to +/-180 degrees or +/- PI radians. Any value outside of this range will be wrapped around into the range.
 * @param units specifies whether to use degrees or radians, or to not wrap at all because the units are linear.
 */
void setFeedbackUnits(uint32_t index, FeedbackCircularUnits units);
/**
 * @return the current error of this channel
 */
float getFeedbackError(uint32_t index);
/**
 * samples the specified input
 * @param type the type of input
 * @param index the index of the specified type of input
 * @return the most recent value of the specified input
 */
float getFeedbackInputOfType(FeedbackInputType type, uint32_t index);

/**
 * Sets some sinks for useful brushless motor info
 * @param channel the channel of interest
 * @param index 0,1,2,3 = input, output, error, setpoint.
 * @param sink the sink
 *
 */
void setFeedbackSink(uint32_t channel, uint32_t index, SinkSource sink);
/**
 * gets some sinks for useful brushless motor info
 * @param channel the feedback channel of interest
 * @param index 0,1,2,3 = input, output, error, setpoint.
 * @return sink the sink
 *
 */
SinkSource getFeedbackSink(uint32_t channel, uint32_t index);

#endif /* FEEDBACKCONTROL_H_ */
