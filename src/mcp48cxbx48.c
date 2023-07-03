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
#include <mcp48cxbx48.h>
#include <spi.h>
#include <math.h>




//*************************************************
//Defines
//*************************************************
#define CHIP_NUM 2
#define NUM_CHANNELS 8
#define NUM_REGS (NUM_CHANNELS + 3)

#define VREF_REG_ADDRESS 0x08
#define PD_REG_ADDRESS 0x09
#define GAIN_REG_ADDRESS 0x0a


//*************************************************
//Types
//*************************************************
typedef struct {
	SpiDev spi;
	PortPin cs;
	uint8_t txBytes[NUM_REGS*3];
	uint8_t rxBytes[NUM_REGS*3];
	uint32_t complete;
} Chip;


//*************************************************
//Variables
//*************************************************

static Chip m_chips[CHIP_NUM];

//*************************************************
//function prototypes
//*************************************************
static void setRegBits(uint32_t chip, uint32_t reg, uint32_t bits, uint32_t mask);

//*************************************************
//Code
//*************************************************

/**
 * initializes this module
 * call at start of code
 */
void mcp48cxbxInit(void){
	for(uint32_t c = 0; c < CHIP_NUM; ++c){

		for(uint32_t i = 0; i < NUM_CHANNELS; ++i){
			setMcp48cxbxValue(c, i, 0.0f);
			setMcp48cxbxState(c, i, PD_HIZ);
		}
	}
}
/**
 * Configures the module.
 * @param chip specifies which chip index we're interested in.
 * @param spi the spi port that the adc is attached to. Part is disabled if set to SPINULL_DEV
 * @param cs the gpio pin that the chip select is attached to
 * @param drdy the gpio pin that the data ready pin is attached to.  If using polling approach, set to NULL_PIN
 */
void mcp48cxbxConfig(uint32_t chip, SpiDev spi, PortPin cs){
	if(chip >= CHIP_NUM){
		return;
	}
	Chip* c = &m_chips[chip];
	c->spi = spi;
	c->cs = cs;
}
/**
 *
 * @param chip specifies which chip index we're interested in.
 * @return the spi port used by the specified chip
 */
SpiDev getMcp48cxbxSpi(uint32_t chip){
	SpiDev result = SPINULL_DEV;
	if(chip < CHIP_NUM){

		Chip* c = &m_chips[chip];
		result = c->spi;
	}
	return result;
}
/**
 *
 * @param chip specifies which chip index we're interested in.
 * @return the chip select pin used for the specified chip
 */
PortPin getMcp48cxbxCsPin(uint32_t chip){
	PortPin result = GPIO_NULL_PIN;
	if(chip < CHIP_NUM){

		Chip* c = &m_chips[chip];
		result = c->cs;
	}
	return result;
}
/**
 * This function triggers a DAC update
 * It is intended to be called from SlowCode but it could be called from fast code for super fast sampling.
 * @param chip specifies which chip index we're interested in.
 *
 */
void mcp48cxbxUpdate(uint32_t chip){
	if(chip >= CHIP_NUM){
		return;
	}
	Chip* c = &m_chips[chip];
	spiQueue8(c->spi, c->cs, c->txBytes, c->rxBytes, NUM_REGS*3, SPI_CPOL_0_CPHA_0, SPI_DIV_4, true, &c->complete);
}
/**
 * sets the value of the specified channel
 * @param chip specifies which chip index we're interested in.
 * @param channel the channel to set
 * @param the value to set it to. Valid values are from 0.0f to 1.0f
 */
void setMcp48cxbxValue(uint32_t chip, uint32_t channel, float value){
	if(channel >= NUM_CHANNELS || chip >= CHIP_NUM){
		return;
	}
	Chip* c = &m_chips[chip];

	uint32_t v = (uint32_t)(value * 4095.0f);

	c->txBytes[channel * 3] = (uint8_t)channel;
	c->txBytes[channel * 3 + 1] = (v >> 8) & 0xff;
	c->txBytes[channel * 3 + 2] = v & 0xff;
}

/**
 * gets the value of the specified channel
 * @param chip the chip of interest
 * @param channel the channel to set
 * @param the value to set it to. Valid values are from 0.0f to 1.0f
 */
float getMcp48cxbxValue(uint32_t chip, uint32_t channel){
	float result = NAN;
	if(channel < NUM_CHANNELS && chip < CHIP_NUM){

		Chip* c = &m_chips[chip];

		uint32_t v = 0;


		v |= c->txBytes[channel * 3 + 1] << 8;
		v |= c->txBytes[channel * 3 + 2];

		result = (float)v;
		result *= 1.0f/4095.0f;


	}
	return result;
}
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
 * @param chip specifies which chip index we're interested in.
 * @param channel the output channel of the specified chip
 * @param state the state to set the specified output to
 */
void setMcp48cxbxState(uint32_t chip, uint32_t channel, Mcp48cxbxChannelState state){
	if(channel >= NUM_CHANNELS || chip >= CHIP_NUM){
		return;
	}

	uint32_t c2 = channel * 2;
	uint32_t c8 = channel + 8;

	switch(state){
	case VREF_BUF: //use vref pin with buffer
		//PD to 00
		//VREF to 11
		//GAIN to 0
		setRegBits(chip, PD_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b11 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b0 << c8, 0b1 << c8);


		break;
	case VREF_UNBUF://use vref pin without buffer
		//PD to 00
		//VREF to 10
		//GAIN to 0
		setRegBits(chip, PD_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b10 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b0 << c8, 0b1 << c8);
		break;

	case BAND_GAP://use internal bandgap
		//PD to 00
		//VREF to 01
		//GAIN to 0
		setRegBits(chip, PD_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b01 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b0 << c8, 0b1 << c8);
		break;

	case BAND_GAPX2://use internal bandgap with x2 gain on output
		//PD to 00
		//VREF to 01
		//GAIN to 1
		setRegBits(chip, PD_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b01 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b1 << c8, 0b1 << c8);
		break;

	case VREF_BUFX2://use vref pin with buffer and x2 gain on output
		//PD to 00
		//VREF to 10
		//GAIN to 1
		setRegBits(chip, PD_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b10 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b1 << c8, 0b1 << c8);
		break;

	case VDD_REF://use VDD as vref
		//PD to 00
		//VREF to 00
		//GAIN to 0
		setRegBits(chip, PD_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b0 << c8, 0b1 << c8);
		break;

	case PD_HIZ://power down and leave output disconnected
		//PD to 11
		//VREF to 00
		//GAIN to 0
		setRegBits(chip, PD_REG_ADDRESS, 0b11 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b0 << c8, 0b1 << c8);
		break;

	case PD_100K://power down but have 100k pull down on output
		//PD to 10
		//VREF to 00
		//GAIN to 0
		setRegBits(chip, PD_REG_ADDRESS, 0b10 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b0 << c8, 0b1 << c8);
		break;

	case PD_1K://power down and have 1k pull down on output
		//PD to 01
		//VREF to 00
		//GAIN to 0
		setRegBits(chip, PD_REG_ADDRESS, 0b01 << c2, 0b11 << c2);
		setRegBits(chip, VREF_REG_ADDRESS, 0b00 << c2, 0b11 << c2);
		setRegBits(chip, GAIN_REG_ADDRESS, 0b0 << c8, 0b1 << c8);
		break;

	}
}
/**
 * sets bits in the specified register of the specified chip
 * @param chip the chip we want to set the registers of
 * @param reg the register address we're interested in
 * @param bits the bits we want to set
 * @param mask masks out only the bits we want to change. All zero bits in the mask mean those bits won't be affected.
 */
static void setRegBits(uint32_t chip, uint32_t reg, uint32_t bits, uint32_t mask){
	if(chip >= CHIP_NUM){
		return;
	}
	Chip* c = &m_chips[chip];
	c->txBytes[reg * 3] = reg;
	c->txBytes[reg * 3 + 1] &= ((~mask) >> 8) & 0xff;
	c->txBytes[reg * 3 + 2] &= ((~mask) >> 0) & 0xff;
	c->txBytes[reg * 3 + 1] |= ((bits) >> 8) & 0xff;
	c->txBytes[reg * 3 + 2] |= ((bits) >> 0) & 0xff;

}

