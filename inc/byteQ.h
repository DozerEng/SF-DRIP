/*
Copyright (c) 2017-2020 STARFISH PRODUCT ENGINEERING INC.

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
#ifndef BYTEQ_H_
#define BYTEQ_H_
#include <stdbool.h>
#include <stdint.h>

//
// one reader, one writer ONLY, otherwise mutexes required in code
//
typedef struct {
	uint32_t bufferSize;
	uint32_t back;//this is the index of where things are added
	uint32_t front;//this is the index of where things are taken off
	uint8_t *buffer;
} ByteQ;




/**
 * inits the queue with the specified buffer and buffer size
 */
void initByteQ( ByteQ* q, uint8_t* buffer, uint32_t bufferSize);
uint16_t getUsed( ByteQ* q);
/**
 * get and remove the item from the front of the queue
 * @return true if successful, false if queue is empty
 */
bool getByte( ByteQ* q, uint8_t* pc);
/**
 * add an item to the back of the queue
 * @return true if successful, false if queue is full and cannot be written to.
 */
bool putByte( ByteQ* q, uint8_t c);

/**
 * puts a bunch of bytes on the queue at once
 * @return the number of bytes successfully put on the queue
 */
uint32_t putBytes(ByteQ* q, uint8_t* bs, uint32_t n);
/**
 * Consider implementing this sometime for future UART DMA use.
 */
//uint16_t getLinearBuf( ByteQ* Q, uint8_t** rxb);
//void dropQ(  ByteQ* Q, uint16_t count);

#endif /* BYTEQ_H_ */
