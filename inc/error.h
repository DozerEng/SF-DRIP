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
#ifndef ERROR_H_
#define ERROR_H_

typedef enum {
	NULL_ERROR						=  0,//this is not an error
	TEST_ERROR						= 1,
	WATCHDOG_RESET_ERROR            = 2,
	SPI_QUEUE_OVERRUN_ERROR         = 3,
	SPI_SEND_FAILED_ERROR           = 4,
	SPI_CS_FAIL_ERROR				= 5,
	COMM_CRC_FAIL_ERROR				= 6,
	STEPPER_POSITION_FAILURE_ERROR  = 7,
	TX_BUFFER_OVERRUN_ERROR			= 8,
	TX_TIMEOUT_ERROR				= 9,
	RESET_ERROR						= 10,
	APPLICATION_ERROR				= 11,
	TIME_SYNC_ERROR                 = 12,

	//	Add additional errors here...

} ErrorType;
/**
 * can be called from anywhere to log an error
 */
void logError(ErrorType type, uint8_t subType);

/**
 * retrieves currently indexed error from the queue
 * result will be zero if no error
 */
uint32_t getCurrentError();



/**
 * removes the current error from the queue and switches to the next in line
 */
void clearCurrentError();


#endif /* ERROR_H_ */
