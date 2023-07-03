/*
Copyright (c) 2021 STARFISH PRODUCT ENGINEERING INC.

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
//************************************************
//note
//************************************************
//this module relies on the quadrature encoder functionality built into the timer peripherals of the STM32F446
//this module assumes that CH1 and CH2 are the A and B quadrature inputs. CH3 is the Z input (which is likely a preload trigger)
//it uses the capture compare register CCR3 to zero the counter. The z input is really the capture trigger for CCR3

//************************************************
//includes
//************************************************
#include <hardwareEncoder.h>
#include <math.h>
#include <ports.h>
#include <stddef.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx.h>
#include <fastcodeUtil.h>



//*************************************************
//defines
//*************************************************
#define VELOC_Q_LEN 10
#define NUM_ENCODER_CHANNELS (HW_ENCODER_TIMER8 + 1)

#define TIM_CCMR1_CONST (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_0) //CC1 & CC2 channels are mapped to TI1 & TI2 respectively
#define TIM_CCMR2_CONST (TIM_CCMR2_CC3S_0)// | TIM_CCMR2_IC3F_0 | TIM_CCMR2_IC4F_0) //CC3S: CC3 channel is input IC3 mapped on TI3)
#define TIM_CCER_CONST (TIM_CCER_CC3E) //enable capture on this pin
#define TIM_SMCR_CONST (0b011) //Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input)
#define TIM_ARR_CONST (0xffffffff) //this is the reload value. Set it really high so the full range of the reg is available

//********************************************************
//TypeDefs
//********************************************************

typedef struct {
	TIM_TypeDef* timer;
	float distancePerCount;//this channel is considered disabled if this is NAN
	float count;
	float distance;
	float velocity;
	float modelDistance;//this is a predicted distance travelled based on the current velocity estimate
	float oldDeltaDist;
	float distanceOffset;
	float countsPerRevolution;//if this value is finite, then it means this is a circular encoder that will cycle around, rather than a linear encoder
	float deltaTimeSinceLastCount;
	uint32_t lastTimeUs;
	SinkSource distanceSink;
	SinkSource velocitySink;
	uint32_t cnt;
	float zeroSig;
	uint32_t ccr;

	SinkSource zeroSink;
} HardwareEncoderState;


//*************************************************
//Variables;
//*************************************************


static HardwareEncoderState m_encoders[NUM_ENCODER_CHANNELS] = {
		{TIM1, NAN, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, NULL_SINK_SOURCE, NULL_SINK_SOURCE, 0, 0.0f, 0},    //HW_ENCODER_TIMER1
		{TIM2, NAN, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, NULL_SINK_SOURCE, NULL_SINK_SOURCE, 0, 0.0f, 0},    //HW_ENCODER_TIMER2
		{TIM3, NAN, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, NULL_SINK_SOURCE, NULL_SINK_SOURCE, 0, 0.0f, 0},    //HW_ENCODER_TIMER3
		{TIM4, NAN, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, NULL_SINK_SOURCE, NULL_SINK_SOURCE, 0, 0.0f, 0},    //HW_ENCODER_TIMER4
		{TIM5, NAN, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, NULL_SINK_SOURCE, NULL_SINK_SOURCE, 0, 0.0f, 0},    //HW_ENCODER_TIMER5
		{TIM8, NAN, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, NULL_SINK_SOURCE, NULL_SINK_SOURCE, 0, 0.0f, 0}     //HW_ENCODER_TIMER8


};


//*************************************************
//function prototypes
//*************************************************
static HardwareEncoderState* getEncoder(HardwareEncoderTimer timer);
static float updateDistance(HardwareEncoderState* s);
static void configTimer(HardwareEncoderState* s);
static bool checkForCapture(HardwareEncoderState* s);
static float updateVelocity(HardwareEncoderState* s, float deltaX, float deltaT);
static void updateEncoder(HardwareEncoderState* s);


//*************************************************
//Code
//*************************************************

/**
 * initialize this module. This should be called at startup
 */
void initHardwareEncoder(void){

}
/**
 * configures the specified encoder channel
 * @param timer - the timer to configure
 * @param sink - the desitination for the encoder values
 */
void setupHardwareEncoder(HardwareEncoderTimer timer, SinkSource sink){
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		t->distanceSink = sink;
		configTimer(t);
	}


}
/**
 * @return the position of the encoder in the configured meaningful distance unit
 */
float getHardwareEncoderDistance(HardwareEncoderTimer timer){
	float result = NAN;
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		updateDistance(t);
	}
	return result;
}


/**
 * must be run in fast code.
 * updates distance based on configured inputs
 */
//void hardwareEncoderFastcode(void);


/**
 * @return true if the index is a valid channel, false if the index is too big or disabled.
 */
bool isHardwareEncoderEnabled(HardwareEncoderTimer timer){
	bool result = false;
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		result = !finitef(t->distancePerCount);
	}
	return result;

}

/**
 * set the encode channel to a specific value
 */
void setHardwareEncoderDistance(HardwareEncoderTimer timer, float value){

}

static HardwareEncoderState* getEncoder(HardwareEncoderTimer timer){
	HardwareEncoderState* result = NULL;
	if(timer <= HW_ENCODER_TIMER8){
		result = &m_encoders[timer];
	}
	return result;
}


/**
 * set the distance per count on the specified channel
 * @param timer - the timer to configure
 * @param value the distance per count. This is essentially a scale factor
 */
void setHardwareEncoderDistancePerCount(HardwareEncoderTimer timer, float value){
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		t->distancePerCount = value;
	}
}
float getHardwareEncoderDistancePerCount(HardwareEncoderTimer timer){
	float result = NAN;
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		result = t->distancePerCount;
	}
	return result;
}

/**
 * sets the circular limit of this encoder
 * @param limit - set to NAN for a linear encoder. Set to a max value for a rotary encoder - the total counts per revolution
 */
void setHardwareEncoderCountsPerRev(HardwareEncoderTimer timer, float value){
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		t->countsPerRevolution = value;
	}
}
float getHardwareEncoderCountsPerRev(HardwareEncoderTimer timer){
	float result = NAN;
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		result = t->countsPerRevolution;
	}
	return result;
}


/**
 * @param timer the specified timer channel
 * @return the sink value for this channel
 */
SinkSource getHardwareEncoderSink(HardwareEncoderTimer timer){
	SinkSource result = NULL_SINK_SOURCE;
	HardwareEncoderState* t = getEncoder(timer);

	if(t != NULL){
		result = t->distanceSink;
	}
	return result;
}





void hardwareEncoderSlowcode(void){

	static uint32_t timer = 0;

	bool isTime = slowTimer(&timer, 1000);



	for(HardwareEncoderTimer i = HW_ENCODER_TIMER1; i <= HW_ENCODER_TIMER8; ++i){
		HardwareEncoderState* s = getEncoder(i);
		if(finitef(s->distancePerCount) || isTime || isStreamSampleTime(s->distanceSink) || isStreamSampleTime(s->velocitySink)){

			updateEncoder(s);
		}
	}
}

//void setHardwareEncoderSampleTime(HardwareEncoderTimer timer, float seconds){
//	HardwareEncoderState* t = getEncoder(timer);
//	if(t != NULL){
//		t->samplePeriodSeconds = seconds;
//	}
//}
/**
 * sets an offset (in distance units) that will be applied to this encoder
 * This means that when the zero flag is detected, the output will be this specified offset
 * @param value the value (in distance units) of the desired offset
 */
void setHardwareEncoderOffset(HardwareEncoderTimer timer, float value){
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		t->distanceOffset = value;
	}
}

float getHardwareEncoderOffset(HardwareEncoderTimer timer){
	float result = NAN;
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		result = t->distanceOffset;
	}
	return result;
}


static void configTimer(HardwareEncoderState* s){
	if(s->timer != TIM1 && s->timer != TIM8){
		//timers2 - 5 are all configured in the same way


		//these should only be written when CC_E bits are clear in CCER
		s->timer->CCMR1 = TIM_CCMR1_CONST;
		s->timer->CCMR2 = TIM_CCMR2_CONST;
		s->timer->CCER = TIM_CCER_CONST;


		//the SMCR
		//first set sms = 000
		//ts=110 for ti2 as input

		s->timer->SMCR = TIM_SMCR_CONST;



		//ARR
		//this sets the roll-over value. Presumably could be 0xffffffff
		s->timer->ARR = TIM_ARR_CONST;

		//CR1 reg
		//CEN=1 to enable

		s->timer->CR1 = TIM_CR1_CEN;
	} else {
		//it's tim1 or 8


	}
}

/**
 * sets up a sink that will indicate when a zeroing event occurs and to what encoder value
 */
void setHardwareEncoderZeroSink(HardwareEncoderTimer timer, SinkSource sink){
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		t->zeroSink = sink;
	}
}


/**
 * sets up a sink that will indicate the calculated encoder velocity
 */
void setHardwareEncoderVelocitySink(HardwareEncoderTimer timer, SinkSource sink){
	HardwareEncoderState* t = getEncoder(timer);
	if(t != NULL){
		t->velocitySink = sink;
	}
}

static bool checkForCapture(HardwareEncoderState* s){
	bool result = false;
	uint32_t sr = s->timer->SR;
	if((sr & (TIM_SR_CC3IF)) != 0){

//		s->ccr = (int32_t)s->timer->CCR3;
		result = true;
	}
	return result;
}


static float updateDistance(HardwareEncoderState* s){



	if(finitef(s->countsPerRevolution)){
		if(s->count > s->countsPerRevolution){
//			s->count -= s->countsPerRevolution;

			float v = s->count / s->countsPerRevolution;
			v -= floorf(v);
			v *= s->countsPerRevolution;
			s->count = v;
		} else if(s->count < 0){
			s->count += s->countsPerRevolution;
		}
	}

	s->distance = ((s->count * s->distancePerCount) - s->distanceOffset);


	return s->distance;
}
/**
 * updates the current computed position of the velocity model and also the velocity
 */
static float updateVelocity(HardwareEncoderState* s, float deltaX, float deltaT){
	static float dts[VELOC_Q_LEN];
	static float dxs[VELOC_Q_LEN];
	static uint32_t i;
	//static uint32_t j;

	dts[i] = deltaT;
	dxs[i] = deltaX;

	float tTot = 0;
	float xTot = 0;
	for(uint32_t j = 0; j < VELOC_Q_LEN; ++j){
		tTot += dts[j];
		xTot += dxs[j];
	}

	s->velocity = (xTot/tTot)*s->distancePerCount;


	return s->velocity;


}

static void updateEncoder(HardwareEncoderState* s){

	uint32_t lastTime = s->lastTimeUs;
	s->lastTimeUs = getTimeInMicroSeconds();
	float deltaT = compareTimeMicroSec(s->lastTimeUs, lastTime);

	int32_t temp = s->cnt;



	//see if there's been a capture
	//if so then set the zero sig
	bool zeroed = checkForCapture(s);
	s->cnt = (uint32_t)(s->timer->CNT) & 0xffff;


	int32_t deltaCnt;
	if(zeroed){
		s->ccr = (uint32_t)(s->timer->CCR3) & 0xffff;
		//check if we've zeroed.
		s->zeroSig = s->ccr;
		deltaCnt = (int32_t)((s->cnt - s->ccr));
		s->count = 0.0f;
	} else {
		s->zeroSig = 0;
		//compute how much the encoder has moved since last.
		deltaCnt = (int32_t)((s->cnt - temp));
	}

//	if((deltaCnt & 0x8000) != 0){
//		deltaCnt |= 0xffff0000;
//	}


	if(deltaCnt > 32767){
		deltaCnt -= 65536;
	} else if(deltaCnt < -32768){
		deltaCnt += 65536;
	}

	float deltaCount = (float)deltaCnt;

	s->count += deltaCount;

	if(s->distance > 360){
		asm("nop");
	}


	setSinkSource(s->distanceSink, updateDistance(s));
	setSinkSource(s->velocitySink, updateVelocity(s, deltaCount, deltaT));
	setSinkSource(s->zeroSink, s->zeroSig);




}
