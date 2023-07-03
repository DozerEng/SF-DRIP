/*
Copyright (c) 2019 STARFISH PRODUCT ENGINEERING INC.

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
//#include <encoder.h>
#include <hardwareEncoder.h>
#include <math.h>
#include <ports.h>
#include <stddef.h>


//*************************************************
//defines
//*************************************************

#define NUM_ENCODER_CHANNELS 4

//********************************************************
//TypeDefs
//********************************************************

typedef struct {
	PortPin inputA;
	PortPin inputB;
	PortPin inputZ;
	float distancePerCount;//this channel is considered disabled if this is NAN
	float distance;
	uint8_t lastPhase;
	uint8_t lastDebounce;
	uint8_t debounceCount;
	float lastIncrement;
	uint8_t debounceNum;
	SinkSource sink;
} EncoderState;


//*************************************************
//Variables;
//*************************************************


static EncoderState m_encoders[NUM_ENCODER_CHANNELS] = {
		{NULL_PIN, NULL_PIN, NULL_PIN, NAN, 0.0f, 0, 0, 0, 0.0f, 0, NULL_SINK_SOURCE},
		{NULL_PIN, NULL_PIN, NULL_PIN, NAN, 0.0f, 0, 0, 0, 0.0f, 0, NULL_SINK_SOURCE},
		{NULL_PIN, NULL_PIN, NULL_PIN, NAN, 0.0f, 0, 0, 0, 0.0f, 0, NULL_SINK_SOURCE},
		{NULL_PIN, NULL_PIN, NULL_PIN, NAN, 0.0f, 0, 0, 0, 0.0f, 0, NULL_SINK_SOURCE}
};



//*************************************************
//function prototypes
//*************************************************
static EncoderState* getEncoder(uint8_t index);

//*************************************************
//Code
//*************************************************




/**
 * initialize this module. This should be called at startup
 */
void initEncoder(void){

}
/**
 * configures the specified encoder channel
 * @param index - the channel being configured
 * @param inputA - the IO index of the 'A' quadrature input
 * @param inputB - the IO index of the 'B' quadrature input
 * @param inputZ - the IO index of the 'Z' zero position input
 * @param distancePerCount - the conversion from encoder counts to a meaningful distance unit, like mm for instance.
 * @param debounceNum - the number of fastcode cycles required to accept a new quadrature state
 */
void setupEncoder(uint8_t index, PortPin inputA, PortPin inputB, PortPin inputZ, float distancePerCount, uint8_t debounceNum){
	EncoderState* e = getEncoder(index);
	if(e != NULL){
		e->inputA = inputA;
		e->inputB = inputB;
		e->inputZ = inputZ;
		e->distancePerCount = distancePerCount;
		e->debounceNum = debounceNum;
	}
}
/**
 * @return the position of the encoder in the configured meaningful distance unit
 */
float getEncoderDistance(uint8_t index){
	EncoderState* e = getEncoder(index);
	return e != NULL ? e->distance : NAN;
}

float getEncoderVelocity(uint8_t index){
	EncoderState* e = getEncoder(index);
	return e != NULL ? 0 : NAN;//TODO: figure out what to do here
}

/**
 * @return true if the index is a valid channel, false if the index is too big.
 */
bool isEncoderEnabled(uint8_t index){
	EncoderState* e = getEncoder(index);
	return e != NULL ? finitef(e->distancePerCount) : false;
}
/**
 * must be run in fast code.
 * updates distance based on configured inputs
 */
void encoderFastcode(void){
	//this table looks up the increment amount based on the previous and current phase pins
	//note that the sign of 2.0f is ambiguous. It could be + or -, depending on the previously determined direction of the encoder.
	static float lut[16] = {
		 0.0f,   //00 00 oldphase newphase
		-1.0f,   //00 01 oldphase newphase
		+1.0f,   //00 10 oldphase newphase
		 2.0f,   //00 11 oldphase newphase

		+1.0f,   //01 00 oldphase newphase
		 0.0f,   //01 01 oldphase newphase
		 2.0f,   //01 10 oldphase newphase
		-1.0f,   //01 11 oldphase newphase

		-1.0f,   //10 00 oldphase newphase
		 2.0f,   //10 01 oldphase newphase
		 0.0f,   //10 10 oldphase newphase
		+1.0f,   //10 11 oldphase newphase

		 2.0f,   //11 00 oldphase newphase
		+1.0f,   //11 01 oldphase newphase
		-1.0f,   //11 10 oldphase newphase
		 0.0f,   //11 11 oldphase newphase
	};

	for(uint32_t index = 0; index < NUM_ENCODER_CHANNELS; ++index){
		EncoderState* e = getEncoder(index);
		if(finitef(e->distancePerCount)){//this tests if it is enabled
			bool a = isPinInputSet(e->inputA);
			bool b = isPinInputSet(e->inputB);
			bool z = isPinInputSet(e->inputZ);

			uint8_t newPhase = 0;
			if(a){
				newPhase |= 0b001;
			}
			if(b){
				newPhase |= 0b010;
			}
			if(z){
				newPhase |= 0b100;
			}

			if(e->lastDebounce == newPhase){
				if(e->debounceCount <= e->debounceNum){//this will count up to ENCODER_DEBOUNCE_NUM+1 and stop
					++(e->debounceCount);
				}
			} else {
				e->debounceCount = 0;
			}
			e->lastDebounce = newPhase;


			if(e->debounceCount == e->debounceNum){

				//if we're not zeroing then increment the distance according to the phase change
				//we've had a valid phase change
				uint8_t luti = (e->lastPhase & 0b11) << 2;
				luti |= newPhase & 0b11;
				luti &= 0xf;//this line is redundant.

				//store new phase for next time
				e->lastPhase = newPhase;

				//lookup what direction we've moved, if any
				float incr = lut[luti];

				if(incr == 2.0f){
					if(e->lastIncrement < 0.0f){
						incr = -2.0f;
					}
				} else {
					e->lastIncrement = incr;
				}
				e->lastIncrement = incr;

				incr *= e->distancePerCount;

				//if the zero input is set then zero the distance register
				if((newPhase & 0b100) != 0){
					e->distance = 0.0f;
				} else {
					e->distance += incr;

				}
			}
			//push the latest value to the specified sink
			setSinkSource(e->sink, e->distance);





			
			//push the latest value to the specified sink
			setSinkSource(e->sink, e->distance);




		}
	}
}
/**
 * zero the specified channel
 */
void zeroEncoder(uint8_t index){
	EncoderState* e = getEncoder(index);
	if(e != NULL){
		e->distance = 0;
	}
}

/**
 * set the encode channel to a specific value
 */
void setEncoderDistance(uint8_t index, float value){
	EncoderState* e = getEncoder(index);
	if(e != NULL){
		e->distance = value;
	}
}

uint32_t getEncoderCount(void){
	return NUM_ENCODER_CHANNELS;
}


static EncoderState* getEncoder(uint8_t index){
	if(index > NUM_ENCODER_CHANNELS){
		return NULL;
	}
	return &m_encoders[index];
}

/**
 * set the sink for the specified encoder channel
 * Presently this must only be app data
 * @param index the channel to set the sink for
 * @param sink the specified sink
 */
void setEncoderSink(uint8_t index, SinkSource sink){
	EncoderState* e = getEncoder(index);
	if(e != NULL){
		e->sink = sink;
	}
}
