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
#ifndef INC_PERIPHERALMANAGER_H_
#define INC_PERIPHERALMANAGER_H_

//*************************************************
//notes
//*************************************************
//this module provides a generalized mechanism to configure a number of different peripherals
//the outputs (or inputs) of the peripherals can be specified to route to (or from) app data or a stream queue
//presumably each peripheral has zero or more values that they can generate,
//these should be routed to either app data or a stream queue, whichever is appropriate
//some peripherals will also have some data sinks.
//these should be plumbed into either app data or a queue
//each source (and maybe sink too) will likely need a gain and maybe an offset
//there will be other configuration stuff required, like i2c bus, chip select pin. That sort of thing
//*************************************************
//includes
//*************************************************
#include <stdint.h>
#include <stdbool.h>

//*************************************************
//defines
//*************************************************


//*************************************************
//Types
//*************************************************
typedef enum {
	PERIPHERAL_UNDEFINED = 0,
	PERIPHERAL_ADS131A0X = 1,
	PERIPHERAL_LSM6DS3 = 2,
	PERIPHERAL_MAX11629 = 3,
	PERIPHERAL_MAX31865 = 4,

} PeripheralType;

typedef union {
	uint32_t u;
	int32_t i;
	float f;
} Generic32;
//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************


#endif /* INC_PERIPHERALMANAGER_H_ */
