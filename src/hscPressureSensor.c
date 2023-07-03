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
#include <hscPressureSensor.h>
#include <fastcodeUtil.h>



//*************************************************
//Defines
//*************************************************
#define SENSOR_COUNT 4
#define SAMPLE_TIME (10*MILLISECONDS)
#define RX_BYTE_COUNT 4

//*************************************************
//Types
//*************************************************
typedef struct {
	SpiDev spiDev;
	PortPin csPin;
	SinkSource pressureSink;
	SinkSource temperatureSink;
	uint32_t spiStatus;
	uint8_t rxBytes[RX_BYTE_COUNT];

	float MaxPressure;
	float MinPressure;

} SensorConfig;

//*************************************************
//Variables
//*************************************************
static SensorConfig m_sensors[4];
static uint32_t m_timer = 0;
static uint8_t txBytes[RX_BYTE_COUNT];

//*************************************************
//function prototypes
//*************************************************


//*************************************************
//Code
//*************************************************
void hscInit(void){
	for(uint32_t i = 0; i < SENSOR_COUNT; ++i){
		SensorConfig* sc = &m_sensors[i];
		sc->spiDev = SPINULL_DEV;
	}

}
/**
 * if spi is SPINULL_DEV then this channel is considered disabled.
 */
void hscSetSpi(uint32_t index, SpiDev spi, PortPin csPin){
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		sc->spiDev = spi;
		sc->csPin = csPin;
	}
}
SpiDev hscGetSpi(uint32_t index){
	SpiDev result = SPINULL_DEV;
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		result = sc->spiDev;
	}
	return result;
}
PortPin hscGetCsPin(uint32_t index){
	PortPin result = GPIO_NULL_PIN;
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		result = sc->csPin;
	}
	return result;
}
void hscSetSinks(uint32_t index, SinkSource pressureSink, SinkSource temperatureSink){
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		sc->pressureSink = pressureSink;
		sc->temperatureSink = temperatureSink;
	}
}
SinkSource hscGetPressureSink(uint32_t index){
	SinkSource result = GPIO_NULL_PIN;
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		result = sc->pressureSink;
	}
	return result;
}
SinkSource hscGetTemperatureSink(uint32_t index){
	SinkSource result = GPIO_NULL_PIN;
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		result = sc->temperatureSink;
	}
	return result;
}
void hscSetMaxPressure(uint32_t index,float p){
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		sc->MaxPressure = p;
	}
}
float hscGetMaxPressure(uint32_t index){
	float result =GPIO_NULL_PIN;
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		result = sc->MaxPressure;
	}
	return result;
}
void hscSetMinPressure(uint32_t index, float p){
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		sc->MinPressure = p;
	}
}
float hscGetMinPressure(uint32_t index){
	 float result =GPIO_NULL_PIN;
	if(index < SENSOR_COUNT){
		SensorConfig* sc = &m_sensors[index];
		result = sc->MinPressure;
	}
	return result;
}
/**
 * This must be called regularly in slow code to ensure correct operation
 */
void hscSlowcode(void){
	if(slowTimer(&m_timer, SAMPLE_TIME)){
		for(uint32_t i = 0; i < SENSOR_COUNT; ++i){
			SensorConfig* sc = &m_sensors[i];
			if(sc->spiDev != SPINULL_DEV){
				//TODO: check rxBytes to see if the last spi transaction got some data
				//if so, pass it to the sinks
				uint32_t i = (uint32_t)sc->rxBytes[0];
				i <<= 8;
				i |= (uint32_t)sc->rxBytes[1];
				i &= 0x3fff;

				float p = (float)i;
				p = (((p  - sc->MinPressure)*(sc->MaxPressure -sc->MinPressure) ) / (16383 - 0)) + sc->MinPressure; // This calculates the pressure out of the sensor. 16383 = 2^14 -1, and is maximum # counts from sensor
				setSinkSource(sc->pressureSink, p);

				//and do the same for temperature
				uint32_t j = (uint32_t)sc->rxBytes[2];
								j <<= 8;
								j |= (uint32_t)sc->rxBytes[3];
								j >>= 5;
								float t = (float)j;
				t =((t/2047) * 200) -50; //calculated from RxBytes and datasheet transfer function
				setSinkSource(sc->temperatureSink, t);

				//TODO: how to scale? Maybe add a scale reg?

				spiQueue8(sc->spiDev, sc->csPin, txBytes, sc->rxBytes, RX_BYTE_COUNT, SPI_CPOL_0_CPHA_0, SPI_DIV_128, true, &(sc->spiStatus));

			}

		}
	}
}
