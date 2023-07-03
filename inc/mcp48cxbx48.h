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
#ifndef INC_MCP48CXBX48_H_
#define INC_MCP48CXBX48_H_
//*************************************************
//Includes
//*************************************************
#include <mcp48cxbx48.h>
#include <stdbool.h>
#include <stdint.h>
#include <spi.h>



//*************************************************
//Defines
//*************************************************


//*************************************************
//Types
//*************************************************
typedef enum {
	VREF_BUF, //use vref pin with buffer
	VREF_UNBUF,//use vref pin without buffer
	BAND_GAP,//use internal bandgap
	BAND_GAPX2,//use internal bandgap with x2 gain on output
	VREF_BUFX2,//use vref pin with buffer and x2 gain on output
	VDD_REF,//use VDD as vref
	PD_HIZ,//power down and leave output disconnected
	PD_100K,//power down but have 100k pull down on output
	PD_1K//power down and have 1k pull down on output
} Mcp48cxbxChannelState;

//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************
/**
 * initializes this module
 * call at start of code
 */
void mcp48cxbxInit(void);

/**
 * Configures the module.
 * @param chip the chip of interest
 * @param spi the spi port that the adc is attached to. Part is disabled if set to SPINULL_DEV
 * @param cs the gpio pin that the chip select is attached to
 * @param drdy the gpio pin that the data ready pin is attached to.  If using polling approach, set to NULL_PIN
 */
void mcp48cxbxConfig(uint32_t chip, SpiDev spi, PortPin cs);

SpiDev getMcp48cxbxSpi(uint32_t chip);

PortPin getMcp48cxbxCsPin(uint32_t chip);
/**
 * This function triggers a DAC update
 * * It is intended to be called from SlowCode but it could be called from fast code for super fast sampling.
 */
void mcp48cxbxUpdate(uint32_t chip);
/**
 * sets the value of the specified channel
 * @param channel the channel to set
 * @param the value to set it to. Valid values are from 0.0f to 1.0f
 */
void setMcp48cxbxValue(uint32_t chip, uint32_t channel, float value);
/**
 * sets the state of the specified pin
 * VREF_BUF - use vref pin with buffer
 * VREF_UNBUF - use vref pin without buffer
 * BAND_GAP - use internal bandgap
 * BAND_GAPX2 - use internal bandgap with x2 gain on output
 * VREF_BUFX2 - use vref pin with buffer and x2 gain on output
 * VDD_REF - use VDD as vref
 * PD_HIZ - power down and leave output disconnected
 * PD_100K - power down but have 100k pull down on output
 * PD_1K - power down and have 1k pull down on output
 */
void setMcp48cxbxState(uint32_t chip, uint32_t channel, Mcp48cxbxChannelState state);
/**
 * gets the value of the specified channel
 * @param chip the chip of interest
 * @param channel the channel to set
 * @param the value to set it to. Valid values are from 0.0f to 1.0f
 */
float getMcp48cxbxValue(uint32_t chip, uint32_t channel);

#endif /* INC_MCP48CXBX48_H_ */
