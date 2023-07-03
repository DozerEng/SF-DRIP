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

#ifndef INC_PORTEXPANDERMANAGER_H_
#define INC_PORTEXPANDERMANAGER_H_

#include <stdint.h>
#include <stdbool.h>
#include <i2c.h>

typedef enum {
	TCA9534,

} IcType;
typedef enum {
	INPUT_LO = 0,
	INPUT_HI = 1,
	OUTPUT_CLEAR = 2,
	OUTPUT_SET = 3,
	OUTPUT_NO_CHANGE = 4

} PinConfig;

/**
 * sets the configuration of the specified port expander
 * @param i the index of this port expander
 * @param bus the I2C bus that the expander is ons
 * @param address offset of the expander. This is equivalent to the address pin value
 * @param  type the type of expander. Valid types so far: TCA9534
 * @return the index of the expander if found, else the next available space. Returns 0xff if no space left.
 */
void setupPortExpander(uint32_t i, I2cDev bus, uint8_t address, IcType type);
/**
 * sets the configuration of the specified pin of the specified port expander
 * @param i the index of the port expander
 * @param pinMask a mask representing the pin to config.  E.g. if the pin number is 2 then the mask is 0b100.
 * @param config how to configure the pin
 */
void setupPortExpanderPin(uint32_t i, uint8_t pinMask, PinConfig config);

uint32_t getPortExpanderCount(void);
/**
 * reconstructs the port config based on the current state of the port.
 * This will not necessarily reflect how the port was initially configured.
 * @param i the index of the specified port expander
 * @param pinMask a mask indicating the pin number in questions. E.g. if the pin number is 2 then the mask is 0b100.
 */
PinConfig getPortPinConfig(uint32_t i, uint32_t pinMask);
I2cDev getPortExpanderBus(uint32_t i);
uint32_t getPortExpanderAddress(uint32_t i);
IcType getPortExpanderType(uint32_t i);
bool isPortExpanderEnabled(uint32_t i);
/**
 * indicates the number of enabled channels. This will also be the index of the first disabled channel
 */
void setNumChannels(uint32_t n);
void portExpanderSlowCode(void);
void portExpanderInit(void);

#endif /* INC_PORTEXPANDERMANAGER_H_ */
