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

#ifndef INC_HARDWAREREV_H_
#define INC_HARDWAREREV_H_

//*************************************************
//notes
//*************************************************


//*************************************************
//includes
//*************************************************

#include <stdint.h>

//*************************************************
//defines
//*************************************************

#define HARDWARE_REV_UNDEFINED (0xffffffff)
#define HARDWARE_REV_NUCLEO (0xffffc1e0)
#define HARDWARE_REV_X1 (1)
#define HARDWARE_REV_X2 (2)
#define HARDWARE_REV_X3 (3)
#define HARDWARE_REV_X4 (4)
#define HARDWARE_REV_X5 (5)
#define HARDWARE_REV_X6 (6)
#define HARDWARE_REV_X7 (7)
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
 * Reads the hardware revision from OTP Flash
 * The result will be the last, non-zero value of the hardware rev section of OTP Flash
 */
uint32_t getHardwareRev(void);

/**
 * sets the hardware revision in OTP Flash.
 * If it has been set before then it will move ahead to the next word and set that
 */
void setHardwareRev(uint32_t rev);


#endif /* INC_HARDWAREREV_H_ */
