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

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <stdbool.h>
#include <stdint.h>


/**
 * A handy-dandy module to manipulate a bounded queue: a first in first out data structure.
 * The queue is assumed to consist of an array of items and two index variables: back and front
 * The back is where stuff goes to line up
 * The front is the item being serviced right now.
 * When the front equals the back then there is nothing on the queue
 * When the next back location - not the current one but the one that would hypothetically exist after adding one more item - equals the front location then the queue is full
 * front and pack indeces will rotate around the queue size
 *
 *
 */



/**
 * indicates if the queue is NOT full, based on the front, back and size.
 * @param front the index of the next item to be removed from the queue
 * @param bck the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @return true if there is still room in the queue for another item to be added
 *
 */
bool isQueueNotFull(uint32_t* front, uint32_t* back, uint32_t size);
/**
 * indicates that there are items in the queue
 * @param front the index of the next item to be removed from the queue
 * @param bck the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @return true if there is at least one item in the queue
 *
 */
bool isQueueNotEmpty(uint32_t* front, uint32_t* back, uint32_t size);

/**
 * call this after adding an item to the queue.
 * Don't add an item unless the queue is not full. (i.e. only add an item if isNotFull is true)
 * @param front the index of the next item to be removed from the queue
 * @param bck the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @return true if the queue was not full and therefore this task completed successfully.
 *
 */
bool justAddedToQueueBack(uint32_t* front, uint32_t* back, uint32_t size);
/**
 * call this when you're done with the item at the front of the queue
 * Don't add an item unless the queue is not full. (i.e. only add an item if isNotFull is true)
 * @param front the index of the next item to be removed from the queue
 * @param bck the index of the location where the next item would be added to the queue
 * @param size the size of the backing array of this queue
 * @return true if there was an item to remove and it was done successfully.
 *
 */
bool doneWithQueueFront(uint32_t* front, uint32_t* back, uint32_t size);

/**
 * @return the number of items currently in the queue
 */
uint32_t getQueueItemCount(uint32_t* front, uint32_t* back, uint32_t size);

/**
 * @return the amount of room left to add items to the queue
 */
uint32_t getQueueRoom(uint32_t* front, uint32_t* back, uint32_t size);
/**
 * sometimes it's useful to index the item at the back of the queue
 * @return the index to the last item.
 */
uint32_t getLastAddedQueueItemIndex(uint32_t* front, uint32_t* back, uint32_t size);

/**
 * @return the number of elements of room in the queue that can be accessed in one contiguous array
 */
uint32_t getContiguousQueueRoom(uint32_t* front, uint32_t*back, uint32_t size);

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
bool justAddedMultipleToQueueBack(uint32_t* front, uint32_t* back, uint32_t size, uint32_t numAdded);

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
bool doneWithMultipleAtQueueFront(uint32_t* front, uint32_t* back, uint32_t qSize, uint32_t count);

#endif /* INC_QUEUE_H_ */
