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
/*
 * This driver is intended for use with the honeywell HSC series pressure transducer, it requires an SPI connection
 * and seperate chip select lines for each driver. it isnt really SPI, more of a one way communication from the sensor.
 */
#ifndef INC_HSCPRESSURESENSOR_H_
#define INC_HSCPRESSURESENSOR_H_
//*************************************************
//Includes
//*************************************************

#include <sinkSource.h>
#include <ports.h>
#include <spi.h>


//*************************************************
//Defines
//*************************************************


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
 *
 */
void hscInit(void);
void hscSetSpi(uint32_t index, SpiDev spi, PortPin csPin);
SpiDev hscGetSpi(uint32_t index);
PortPin hscGetCsPin(uint32_t index);
void hscSetSinks(uint32_t index, SinkSource pressureSink, SinkSource temperatureSink);
SinkSource hscGetPressureSink(uint32_t index);
SinkSource hscGetTemperatureSink(uint32_t index);
void hscSetMinPressure(uint32_t index, float MinPressure);
void hscSetMaxPressure(uint32_t index, float MaxPressure);
float hscGetMinPressure(uint32_t index);
float hscGetMaxPressure(uint32_t index);

/**
 * This must be called regularly in slow code to ensure correct operation
 */
void hscSlowcode(void);


#endif /* INC_HSCPRESSURESENSOR_H_ */
