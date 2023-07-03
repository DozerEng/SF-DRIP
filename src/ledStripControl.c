/*
Copyright (c) 2023 STARFISH PRODUCT ENGINEERING INC.

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
//Includes
//*************************************************
#include <fastcodeUtil.h>
#include <ledStripControl.h>
#include <ports.h>
#include <math.h>


//*************************************************
//Defines
//*************************************************
#define WAYPOINT_TOTAL 10
#define POLLING_TIME (10*MILLISECONDS)
#define SECONDS_PER_POLL (0.01f)

#define GET_RED(i) (((float)((i >> 24) & 0xff)) * (1.0f/255.0f))
#define GET_GREEN(i) (((float)((i >> 16) & 0xff)) * (1.0f/255.0f))
#define GET_BLUE(i) (((float)((i >> 8) & 0xff)) * (1.0f/255.0f))
#define GET_WHITE(i) (((float)((i >> 0) & 0xff)) * (1.0f/255.0f))


//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************
static uint8_t* m_bytes;
static uint32_t m_byteTotal;
static bool m_grbNotGrbw;
static uint32_t m_ledCount;
static SpiDev m_spiPort;
static float m_waypoints[WAYPOINT_TOTAL][4];
static uint32_t m_waypointTotal;
static uint32_t m_waypointIndex0;
static uint32_t m_waypointIndex1;
static uint32_t m_timer;
static float m_cycleTime;
static float m_phasePerLed;
static float m_cycleLedWidth;
static uint32_t m_txDone;
static float m_secondsCounter;
static float m_deltaR;
static float m_deltaG;
static float m_deltaB;
static float m_deltaW;
static float m_x0;
static float m_x1;

//*************************************************
//function prototypes
//*************************************************


/**
 * computes next incremental LED state
 */
static void computeNextState(void);

void ledStripSetInt(uint32_t index, uint32_t grbw);
static uint32_t computeWaypointColour(uint32_t ledIndex, float phase);



//*************************************************
//Code
//*************************************************

void ledStripConfig(SpiDev dev, uint32_t byteCount, uint8_t* byteArray, bool rgbNotRgbw){
	m_bytes = byteArray;
	m_byteTotal = byteCount;
	m_grbNotGrbw = rgbNotRgbw;
	m_spiPort = dev;
	uint32_t bytesPerLed = rgbNotRgbw ? LED_STRIP_BYTES_FROM_COUNT_RGB(1) : LED_STRIP_BYTES_FROM_COUNT_RGBW(1);
	m_ledCount = byteCount / bytesPerLed;

}

void ledStripWaypointAdd(float r, float g, float b, float w, float gain){
	if(m_waypointTotal < WAYPOINT_TOTAL - 1){


		m_waypoints[m_waypointTotal][0] = r*gain;
		m_waypoints[m_waypointTotal][1] = g*gain;
		m_waypoints[m_waypointTotal][2] = b*gain;
		m_waypoints[m_waypointTotal][3] = w*gain;
		++m_waypointTotal;
	}
}

uint32_t ledStripGetWaypointCount(void){
	return m_waypointTotal;
}

void ledStripSlowcode(void){
	if(slowTimer(&m_timer, POLLING_TIME)){
		//TODO: compute new state
		computeNextState();
		if(finitef(m_cycleTime) != 0 && m_cycleTime > 0.0f){

		}

		//Now update the strip from the current state
		spiQueue8(m_spiPort, GPIO_NULL_PIN, m_bytes, 0, m_byteTotal, SPI_CPOL_0_CPHA_1, SPI_DIV_16, false, &m_txDone);	//setup read of data register


	}
}
void ledStripSetInt(uint32_t index, uint32_t grbw){

	//check to be sure the index is not too big
	if(index < m_ledCount){

		uint32_t bi = m_grbNotGrbw ? LED_STRIP_BYTES_FROM_COUNT_RGB(index) : LED_STRIP_BYTES_FROM_COUNT_RGBW(index);
		uint16_t newByte = 0;//this is 16 bits to account for possible overflow without losing the contents
		uint32_t n = m_grbNotGrbw ? 24 : 32;
		bool bit;
		uint32_t testBitIndex = 31;
		uint32_t bitIndex = 13;
		for(uint32_t i = 0; i < n; ++i){//step through the bits - either 24 or 32

			bit = (grbw & (1 << testBitIndex)) != 0;

			newByte |= (bit ? 0b110 : 0b100) << bitIndex;//this is the bit pattern that we use on SPI
			bitIndex -= 3;
			--testBitIndex;

			if(bitIndex <= 5){//if it's less than 5 then adding three bits won't affect the top 8 bits, so we must be done that byte
				bitIndex += 8;
				uint8_t byte = (uint8_t)((newByte >> 8) & 0xff);
				newByte <<= 8;
				m_bytes[bi] = byte;//shift back if necessary
//				newByte >>= 8;
				++bi;
			}
		}
	}
}
/**
 * sets the state of the LED with the specified index
 */
void ledStripSet(uint32_t index, float r, float g, float b, float w){
	//first construct a 32 bit int
	uint32_t grbw = ledStripGetColour(r, g, b, w);
	ledStripSetInt(index, grbw);
}

void ledStripSetBrightness(uint32_t index, uint32_t rgbw, float brightness){
	float r = GET_RED(rgbw) / 255.0f;
	float g = GET_GREEN(rgbw) / 255.0f;
	float b = GET_BLUE(rgbw) / 255.0f;
	float w = GET_WHITE(rgbw) / 255.0f;
	uint32_t grbw = ledStripGetColour(r, g, b, w);
	ledStripSetInt(index, grbw);


}

/**
 * gets the state of the LED with the specified index
 * Result is a 32 bit integer that is the concatenation MSB to LSB of r, g, b, w
 */
uint32_t ledStripGet(uint32_t index){
	uint32_t result = 0;

	//check to be sure the index is not too big
	if(index < m_ledCount){
		uint32_t bi = m_grbNotGrbw ? LED_STRIP_BYTES_FROM_COUNT_RGB(index+1) - 1 : LED_STRIP_BYTES_FROM_COUNT_RGBW(index+1) - 1;
		uint32_t testBitPos = 1;//check the middle bit of three to see if it's a 1 or 0
		uint32_t n = m_grbNotGrbw ? 24 : 32;
		bool bit;
		uint8_t testByte;
		for(uint32_t i = 0; i < n; ++i){//step through the bits
			//sample from LSB of the last byte up towards the MSB of the first byte
			result >>= 1;
			testByte = m_bytes[bi];
			bit = (testByte & 1<<testBitPos) != 0;
			result |= bit ? 0x80000000 : 0;//shift in next bit
			//increment to the next bit
			testBitPos += 3;

			if(testBitPos >= 8){
				testBitPos -= 8;
				--bi;

			}

		}
	}
	return result;
}

/**
 * sets the cycle time for traversing all waypoints
 */
void ledStripSetCycleTime(float seconds){
	m_cycleTime = seconds;

}

void ledStripSetPhasePerLed(float phase){
	m_phasePerLed = phase;
}
float ledStripGetPhasePerLed(){
	return m_phasePerLed;
}
/**
 * clears all waypoints.
 * This will freeze the strip at the instantaneous state
 */
void ledStripClearWaypoints(void){
	m_waypointTotal = 0;

}



static void computeNextState(void){
	//update polling time is: POLLING_TIME
	//total cycle time is m_cycleTime
	//number of waypoints is m_waypointTotal
	//we need to compute the deltas to the next waypoint

	float phasePerSecond = 0.1f;//amount the phase moves per second

	static uint32_t lastTime = 0;
	static float phase = 0.0f;

	uint32_t t = getTimeInMicroSeconds();

	float deltaT = compareTimeMicroSec(t,  lastTime);
	lastTime = t;

	float deltaPhase = phasePerSecond*deltaT;
	phase += deltaPhase;
	phase -= floorf(phase);//ensure phase is always between zero and one


	if(m_waypointTotal > 0){

		for(int i = 0; i < m_ledCount; ++i){
			uint32_t grbw = computeWaypointColour(i, phase);
			ledStripSetInt(i, grbw);
		}


	}
}

static uint32_t computeWaypointColour(uint32_t ledIndex, float phase){
	float phasePerLed = m_phasePerLed;//the amount the phase moves per LED
	uint32_t in = ledStripGetWaypointCount();
	float n = (float)in;

	float ledPhase = phasePerLed*((float)ledIndex);
	ledPhase += phase;

	ledPhase -= floorf(ledPhase);//ensure it's between zero and one

	float x1 = ledPhase * n;
	uint32_t i0 = (uint32_t)floorf(x1);//now resolve to individial waypoints
	uint32_t i1 = i0 + 1;
	if(i1 >= in){
		i1 = 0;
	}

	x1 -= floorf(x1);
	float x0 = 1.0f - x1;





	//lookup waypoints at each side of our phase
	float r0 = m_waypoints[i0][0];
	float g0 = m_waypoints[i0][1];
	float b0 = m_waypoints[i0][2];
	float w0 = m_waypoints[i0][3];

	float r1 = m_waypoints[i1][0];
	float g1 = m_waypoints[i1][1];
	float b1 = m_waypoints[i1][2];
	float w1 = m_waypoints[i1][3];


	//interpolate
	float r = r0*x0 + r1*x1;
	float g = g0*x0 + g1*x1;
	float b = b0*x0 + b1*x1;
	float w = w0*x0 + w1*x1;

	return ledStripGetColour(r, g, b, w);


}

uint32_t ledStripGetColour(float r, float g, float b, float w){
	uint32_t result = 0;

	uint32_t temp;
	temp = (int)(g * 255.0f);
	if(temp > 255){
		temp = 255;
	}
	result += temp << 24;
	temp = (int)(r * 255.0f);
	if(temp > 255){
		temp = 255;
	}
	result += temp << 16;
	temp = (int)(b * 255.0f);
	if(temp > 255){
		temp = 255;
	}
	result += temp << 8;
	temp = (int)(w * 255.0f);
	if(temp > 255){
		temp = 255;
	}
	result += temp << 0;
	return result;
}




