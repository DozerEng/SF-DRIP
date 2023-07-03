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


#include <queue.h>
#include <tca9534.h>
#include <fastcodeUtil.h>

//*************************************************
//Defines
//*************************************************

#define QUEUE_SIZE 5

//*************************************************
//Types
//*************************************************

typedef struct {
	I2cDev dev;
	uint8_t address;
	uint8_t* rxInputs;
	uint8_t rxOutputs;
	uint8_t rxInverts;
	uint8_t rxConfig;
	uint8_t txData[7];
	TransactionStatus status[4];
	bool started;
} Tca9534State;
//*************************************************
//Variables
//*************************************************

static Tca9534State m_state[QUEUE_SIZE];
static uint32_t queueFront = 0;
static uint32_t queueBack = 0;
static uint32_t m_timeoutTimer;


//*************************************************
//function prototypes
//*************************************************

//*************************************************
//Code
//*************************************************


/**
 * @param i2c specifies which bus to use
 * @param address the i2c address offset of this port expander.
 * This corresponds to the 3 address select pins combined as binary digits into an integer
 * address  actual bus address
 *  0b000  0x38
 *  0b001  0x39
 *  0b010  0x3a
 *  0b011  0x3b
 *  0b100  0x3c
 *  0b101  0x3d
 *  0b110  0x3e
 *  0b111  0x3f
 *  @param dir the direction register mask, where a bit of 1 is an input and 0 is an output
 *  @param output the values to set the output pins to
 *  @param input a pointer to a variable where to put the read in pin values
 *  @return true if queue was successful
 */
bool queueTca9534(I2cDev dev, uint8_t address, uint8_t dir, uint8_t output, uint8_t* inputs){
	Tca9534State* s = &(m_state[queueBack]);
	//now add the new item
	bool result = false;
	if(isQueueNotFull(&queueFront, &queueBack, QUEUE_SIZE)){

		//now put request onto queue

		s->address = address + 0x38;
		s->dev = dev;
		s->rxInputs = inputs;
		s->txData[0] = 0x00;//this is the data for 1st transaction. It's the address of the input register.
		s->txData[1] = 0x01;//this is the 1st tx byte of 2nd transaction. It's the address of the output register.
		s->txData[2] = output;//this is 2nd tx byte of 2nd transaction. It's the value to load into the output register.
		s->txData[3] = 0x02;//this is the 1st tx byte of 3nd transaction. It's the address of the polarity register.
		s->txData[4] = 0x00;//this is 2nd tx byte of 2nd transaction. We'll set the polarity register to zero for now.
		s->txData[5] = 0x03;//this is the 1st tx byte of 3rd transaction. It's the address of the config registor.
		s->txData[6] = dir;//this is 2nd tx byte of 3rd transaction. We'll load the config reg with the direction of each pin.
		s->started = false;
		justAddedToQueueBack(&queueFront, &queueBack, QUEUE_SIZE);
		result = true;
	}

	return result;
}
uint32_t getTca9534QueueRoom(void){
	return getQueueRoom(&queueFront, &queueBack, QUEUE_SIZE);
}

void tca9534SlowCode(void){
	Tca9534State* s = &(m_state[queueFront]);


	//first check for done transactions and service the next new ones

	if(isQueueNotEmpty(&queueFront, &queueBack, QUEUE_SIZE) ){
		s = &(m_state[queueFront]);
		TransactionStatus ts = s->status[3];
		if(!(s->started)){
			resetSlowTimer(&m_timeoutTimer);
			if(getI2cQueueRoom(s->dev) >= 4){

				i2cQueue(s->dev, s->address<<1, &(s->txData[0]), 1, s->rxInputs, 1,  &(s->status[0]));
				i2cQueue(s->dev, s->address<<1, &(s->txData[1]), 2, &(s->rxOutputs), 1,  &(s->status[1]));
				i2cQueue(s->dev, s->address<<1, &(s->txData[3]), 2, &(s->rxInverts), 1,  &(s->status[2]));
				i2cQueue(s->dev, s->address<<1, &(s->txData[5]), 2, &(s->rxConfig), 1,  &(s->status[3]));
				s->started = true;
			}
		} else if(ts == TRANSACTION_COMPLETED_OK || ts == TRANSACTION_TIMED_OUT || ts == TRANSACTION_QUEUE_FULL){
			doneWithQueueFront(&queueFront, &queueBack, QUEUE_SIZE);
			resetSlowTimer(&m_timeoutTimer);
		}
		if(slowTimer(&m_timeoutTimer, 100000)){
			doneWithQueueFront(&queueFront, &queueBack, QUEUE_SIZE);
		}

	}
}

