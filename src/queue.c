/*
Copyright (c) 2020 STARFISH PRODUCT ENGINEERING INC.

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

#include "queue.h"

//*************************************************
//Notes
//*************************************************


//front is the location of the item currently being serviced in the queue. It's the item at the ticket counter to use a movie lineup as a metaphor
//back is the location of first free space in the queue. In other words it is one plus the location of the last added item.
//when front equals back then there are no items in the queue
//when (back + 1)%size == front then the queue is full

//I don't think you can ever completely fill the queue with this method. The best you can do is size - 1 elements


//Examples:
//if b == f then size is 0
//if b is 5 and f is 0 then there are valid items in locations: 0, 1, 2, 3, 4 so size is 4

//if b is size - 1 and f is 0 then expect result to be size - 1
//if f is size - 1 and b is 0 then expect result to be 1



//*************************************************
//Defines
//*************************************************

//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************

//*************************************************
//Code
//*************************************************



/**
 * indicates if the queue is NOT full, based on the front, back and size.
 * @param front the index of the next item to be removed from the queue
 * @param back the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @return true if there is still room in the queue for another item to be added
 *
 */
bool isQueueNotFull(uint32_t* front, uint32_t* back, uint32_t size){
	uint32_t newBack = *back + 1;
	if(newBack >= size){
		newBack = 0;
	}
	return newBack != *front;
}
/**
 * indicates that there are items in the queue
 * @param front the index of the next item to be removed from the queue
 * @param back the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @return true if there is at least one item in the queue
 *
 */
bool isQueueNotEmpty(uint32_t* front, uint32_t* back, uint32_t size){
	return *front != *back;
}

/**
 * call this after adding an item to the queue.
 * Don't add an item unless the queue is not full. (i.e. only add an item if isNotFull is true)
 * @param front the index of the next item to be removed from the queue
 * @param back the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @return true if the queue was not full and therefore this task completed successfully.
 *
 */
bool justAddedToQueueBack(uint32_t* front, uint32_t* back, uint32_t size){
	bool result = false;
	uint32_t newBack = *back + 1;
	if(newBack >= size){
		newBack = 0;
	}
	result = newBack != *front;
	if(result){
		*back = newBack;
	}
	return result;
}
/**
 * call this when you're done with the item at the front of the queue
 * Don't add an item unless the queue is not full. (i.e. only add an item if isNotFull is true)
 * @param front the index of the next item to be removed from the queue
 * @param back the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @return true if there was an item to remove and it was done successfully.
 *
 */
bool doneWithQueueFront(uint32_t* front, uint32_t* back, uint32_t size){
	bool result;
	if(isQueueNotEmpty(front, back, size)){
		uint32_t newFront = *front + 1;
		if(newFront >= size){
			newFront = 0;
		}
		*front = newFront;
		result = true;
	} else {
		result = false;
	}
	return result;



}
/**
 * call this when you're done with the item at the front of the queue
 * This assumes that you've already checked that there are more than count items in the queue
 * @param front the index of the next item to be removed from the queue
 * @param back the index of the location where the next item would be added to the queue
 * @param qSize the size of the backing array of this queue
 * @param count the number of items used
 * @return true if there was an item to remove and it was done successfully.
 *
 */
bool doneWithMultipleAtQueueFront(uint32_t* front, uint32_t* back, uint32_t qSize, uint32_t count){
	bool result;
	if(isQueueNotEmpty(front, back, qSize)){
		uint32_t newFront = *front + count;
		if(newFront >= qSize){
			newFront -= qSize;
		}
		if(newFront )
		*front = newFront;
		result = true;
	} else {
		result = false;
	}
	return result;



}

/**
 * @return the number of items currently in the queue
 */
uint32_t getQueueItemCount(uint32_t* front, uint32_t* back, uint32_t size){
	int32_t f = (int32_t)(*front);
	int32_t b = (int32_t)(*back);

	int32_t r = b - f;

	if(r < 0){
		r += size;
	}

	//result should now be positive
	return (uint32_t)r;
}

/**
 * @return the amount of room left to add items to the queue
 */
uint32_t getQueueRoom(uint32_t* front, uint32_t* back, uint32_t size){
	return size - getQueueItemCount(front, back, size) - 1;
}

/**
 * sometimes it's useful to index the item at the back of the queue
 * @return the index to the last item.
 */
uint32_t getLastAddedQueueItemIndex(uint32_t* front, uint32_t* back, uint32_t size){
    uint32_t result = *back;

    if(result == 0){
        result = size;
    }
    --result;

    return result;
}

/**
 * @return the number of elements of room in the queue that can be accessed in one contiguous array
 */
uint32_t getContiguousQueueRoom(uint32_t* front, uint32_t* back, uint32_t size){
	uint32_t r = getQueueRoom(front, back, size);

	uint32_t nextBack = *back + r;
	nextBack = (nextBack >= size) ? (size - 1) : nextBack;

	return nextBack - *back + 1;
}

/**
 * call this after adding multiple items to the queue.
 * Don't add an item unless the queue is not full. (i.e. only add an item if isNotFull is true)
 * @param front the index of the next item to be removed from the queue
 * @param back the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @param numAdded the number of items that were added. This assumes that there is room but will tell you if you added too much
 * @return true if the queue was not full and therefore this task completed successfully.
 *
 */
bool justAddedMultipleToQueueBack(uint32_t* front, uint32_t* back, uint32_t size, uint32_t numAdded){
	bool result = false;
	uint32_t newBack = *back + numAdded;

	uint32_t n = getQueueRoom(front, back, size);
	if(numAdded < n){


		if(newBack >= size){
			newBack -= size;
		}


		*back = newBack;
		result = true;
	}
	return result;
}




