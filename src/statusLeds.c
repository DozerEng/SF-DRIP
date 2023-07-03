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


//*************************************************
//notes
//*************************************************



//*************************************************
//includes
//*************************************************
#include <statusLeds.h>
#include <fastCodeUtil.h>

//*************************************************
//defines
//*************************************************


//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************
static PortPin m_commsLed = NULL_PIN;
static PortPin m_heartbeatLed = NULL_PIN;

//static uint32_t blinkTimerReg = 0;
//static uint32_t commBlinkTimerReg = 0;

static uint32_t m_heartbeatTimer = 0;
static uint32_t m_commBlinkTimer = 0;


//*************************************************
//function prototypes
//*************************************************
///**
// * turn on or off the heartbeat LED.
// * This will likely be called in main
// */
//static void toggleHeartbeatStatusLed(void);
///**
// * turn on or off the comms LED
// * This will likely be called in main
// * @param enable true = LED on. false = LED off. This assumes that the IO pin is connected to the cathode.
// */
//static void setCommsStatusLed(bool enable);

//*************************************************
//code
//*************************************************


/**
 * configures which port pins to use as LEDs
 * This does not setup the pin directions
 */
void configStatusLeds(PortPin commsLed, PortPin heartbeatLed){
	m_commsLed = commsLed;
	m_heartbeatLed = heartbeatLed;
}

///**
// * turn on or off the heartbeat LED.
// * This will likely be called in main
// */
//static void toggleHeartbeatStatusLed(void){
//	togglePin(m_heartbeatLed);
//}
///**
// * turn on or off the comms LED
// * This will likely be called in main
// * @param enable true = LED on. false = LED off. This assumes that the IO pin is connected to the cathode.
// */
//static void setCommsStatusLed(bool enable){
//	setPin(m_commsLed, !enable);
//}

/**
 * this must be run in slowcode
 */
void statusLedSlowcode(void){

	uint32_t time = getTimeInMicroSeconds();


	float heartDelta = compareTimeMicroSec(time, m_heartbeatTimer);
	float blinkDelta = compareTimeMicroSec(time, m_commBlinkTimer);


	if(m_commsLed == m_heartbeatLed){
		//there's only one LED so do both heartbeat and comms blink on single
		if(blinkDelta > 1.0f){
			m_commBlinkTimer = time;
		} else if(blinkDelta > 0.52f){
			setPin(m_commsLed, false);//turn on LED, for heartbeat

		} else if(blinkDelta > 0.02f){
			setPin(m_commsLed, true);//turn off LED, for heartbeat

		} else {

		}
	} else {
		//do heartbeat
		if(heartDelta > 1.0f){
			m_heartbeatTimer = time;
		} else if(heartDelta > 0.5f){
			setPin(m_heartbeatLed, false);//turn off LED for heartbeat
		} else {
			setPin(m_heartbeatLed, true);//turn off LED for heartbeat
		}
		//do comms blink
		if(blinkDelta > 1.0f){
			m_commBlinkTimer = time;
			setPin(m_commsLed, true);//turn off LED
		} else if(blinkDelta > 0.02f){
			setPin(m_commsLed, true);//turn off comm blink led
		} else {

		}
	}


}
/**
 * Call this when a packet is received, this will cause the comm LED to blink.
 */
void statusLedPacketReceived(void){
	resetSlowTimer(&m_commBlinkTimer);
	setPin(m_commsLed, false);//turn on comms LED

}
