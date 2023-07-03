/*
Copyright (c) 2017-2018 STARFISH PRODUCT ENGINEERING INC.

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
//includes
//*************************************************
#include <fastcode.h>
#include <fastcodeUtil.h>
#include <packetReceiver.h>
#include <stepperMotor.h>
#include <stdbool.h>
#include <stdint.h>
#include "comms.h"
#include "ports.h"
#include "adcManager.h"
#include "spi.h"
#include "error.h"
#include "dac.h"
#include "timeSync.h"
#include "sigGen.h"
#include "ports.h"
#include "encoder.h"
#include "i2c.h"
#include "packetReceiver.h"
#include "brushlessMotor.h"
#include "softPwm.h"
#include <ads131m0x.h>


//*************************************************
//defines
//*************************************************

#define NUM_BRANCHES 2


//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************
static uint32_t branchNum = 0;

//*************************************************
//function prototypes
//*************************************************


//*************************************************
//code
//*************************************************



/** Main Interrupt for device, Fast Code,
 * this magically gets referenced from the interupt vector table by the linker
**/
void fastcode(void){







//	testFastcodeTimeFromHere();



	spiFastCode();






	timeSyncFastCode();




	commsFastCode();//6.2us





	softPwmFastCode();



	//12us


	adcManagerFastCode(); //6.8us

	//17us







	i2cFastcode();

	//23us





	encoderFastcode();

	//31us


	switch (branchNum){
	case 0:

//		packetReceiverFastcode();

		stepperFastCode();
		brushlessFastCode();


		break;
	case 1:
		dacFastCode();
		ads131m0xFastCode();

		powerOutputFastCode();
		sigGenFastCode();


		break;

//	case 2:
//
//
//
//		break;



	default:
		//branchNum = 0;
		break;
	}
	++branchNum;
	if (branchNum >= NUM_BRANCHES){
		branchNum = 0;
	}

	//47.9us

//	testFastcodeTimeToHere();

}
