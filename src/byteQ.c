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

//*************************************************
//includes
//*************************************************


#include "byteQ.h"
#include "queue.h"
#include <string.h>





//*************************************************
//defines
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


//*************************************************
//code
//*************************************************


/**
 * inits the queue with the specified buffer and buffer size
 */
void initByteQ( ByteQ* q, uint8_t* buffer, uint32_t bufferSize){
	q->front = 0;
	q->back = 0;
	q->bufferSize = bufferSize;
	q->buffer = buffer;
}
uint16_t getUsed( ByteQ* q){
	return getQueueItemCount(&(q->front), &(q->back), q->bufferSize);
}
/**
 * get and remove the item from the front of the queue
 */
bool getByte( ByteQ* q, uint8_t* pc){
	bool result = false;
	if(isQueueNotEmpty(&(q->front), &(q->back), q->bufferSize)){
		*pc = q->buffer[q->front];
		doneWithQueueFront(&(q->front), &(q->back), q->bufferSize);
		result = true;
	}
	return result;
}

uint32_t getBytes(ByteQ* q, uint8_t* pc, uint32_t count){
	uint32_t result = getQueueItemCount(&(q->front), &(q->back), q->bufferSize);
	//first decide how many to move: either the full amount (count) or the amount in queue if less
	if(count < result){
		result = count;
	}
	if(result > 0){
		//now move the amount up to the end of the buffer
		uint32_t nextFront = result + q->front;
		if(nextFront > q->bufferSize){
			nextFront = q->bufferSize - 1;
		}
		uint32_t num = nextFront - q->front;
		memcpy(&(q->buffer[q->front]), pc,  num);
		//if we didn't do the full amount up to the end of the buffer, then do the rest from the start of the buffer
		if(num < result){
			memcpy(q->buffer, pc + num,  result - num);
		}


		doneWithMultipleAtQueueFront(&(q->front), &(q->back), q->bufferSize, result);
	}
	return result;
}
/**
 * add an item to the back of the queue
 */
bool putByte( ByteQ* q, uint8_t c){
	bool result = false;
	if(isQueueNotFull(&(q->front), &(q->back), q->bufferSize)){
		q->buffer[q->back] = c;
		justAddedToQueueBack(&(q->front), &(q->back), q->bufferSize);
		result = true;
	}
	return result;
}

/**
 * puts a bunch of bytes on the queue at once
 * @return the number of bytes successfully put on the queue
 */
uint32_t putBytes(ByteQ* q, uint8_t* bs, uint32_t n){
	uint32_t num = getContiguousQueueRoom(&(q->front), &(q->back), q->bufferSize);
	if(num >= n){
		num = n;
	}

	memcpy(&(q->buffer[q->back]), bs,  num);
	justAddedMultipleToQueueBack(&(q->front), &(q->back), q->bufferSize, num);



	return num;
}




