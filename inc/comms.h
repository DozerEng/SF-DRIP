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
#ifndef COMMS_H_
#define COMMS_H_

//#include "_processorGlobal.h"

#include "byteQ.h"

typedef enum {
	UARTNULL_DEV,
	USART1_DEV,
	USART2_DEV,
	USART3_DEV,
	UART4_DEV,
	UART5_DEV,
	USART6_DEV

} UartDev;



void initComms(UartDev dev, ByteQ* in, ByteQ* out, uint32_t baud, bool useDma);
/**

 */
void commsFastCode(void);

/**
 * @return the last port to receive a byte
 */
UartDev getLastPortToRecieveData(void);

void initBaudrateUpdater(UartDev u);
void updateBaudrate(uint32_t br);
uint32_t getBaudrateUpdate(void);

#endif /* COMMS_H_ */
