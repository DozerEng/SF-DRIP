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
#ifndef INC_LEDSTRIPCONTROL_H_
#define INC_LEDSTRIPCONTROL_H_
//*************************************************
//Includes
//*************************************************
#include <stdint.h>
#include <stdbool.h>
#include <spi.h>



//*************************************************
//Defines
//*************************************************
//returns the number of bytes required for this module to communicate with the specified number of RGB LEDs
//this requires 3 bits for each bit of each individual R, G & B led, each requiring 8 bits to specify their amplitude
#define LED_STRIP_BYTES_FROM_COUNT_RGB(ledCount) ((ledCount)*9)
//returns the number of bytes required for this module to communicate with the specified number of RGBW LEDs
#define LED_STRIP_BYTES_FROM_COUNT_RGBW(ledCount) ((ledCount)*12)

//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************
/**
 * configures the module to control a daisy chain of LEDs
 * @param dev - the SPI peripheral to use: SPI1_DEV, SPI2_DEV, SPI3_DEV
 * @param byteCount - the number of bytes in the bytes array. This defines the number of LEDs, using the LED_STRIP_BYTES_FROM_COUNT_RGB or RBGW macros
 * @param byteArray - a pointer to the byte array
 * @param rgbNotRgbw - true = RGB leds. false = RGBW leds.
 */
void ledStripConfig(SpiDev dev, uint32_t byteCount, uint8_t* byteArray, bool rgbNotRgbw);
/**
 * adds a waypoint to the current pattern. If using a RGB strip, not a RBGW then w is ignored
 */
void ledStripWaypointAdd(float r, float g, float b, float w, float gain);

/**
 * sets the state of the LED with the specified index
 */
void ledStripSet(uint32_t index, float r, float g, float b, float w);
/**
 * sets the state of the specified LED given a colour integer and a brightness factor (0.0f to 1.0f)
 */
void ledStripSetBrightness(uint32_t index, uint32_t rgbw, float brightness);
/**
 * gets the state of the LED with the specified index
 * Result is a 32 bit integer that is the concatenation MSB to LSB of r, g, b, w
 */
uint32_t ledStripGet(uint32_t index);
/**
 * sets the cycle time for traversing all waypoints
 */
void ledStripSetCycleTime(float seconds);
/**
 * clears all waypoints.
 * This will freeze the strip at the instantaneous state
 */
void ledStripClearWaypoints(void);

/**
 * this should be run regularly in slow code
 */
void ledStripSlowcode(void);

uint32_t ledStripGetWaypointCount(void);

uint32_t ledStripGetColour(float red, float green, float blue, float white);
void ledStripSetPhasePerLed(float phase);
float ledStripGetPhasePerLed();

#endif /* INC_LEDSTRIPCONTROL_H_ */
