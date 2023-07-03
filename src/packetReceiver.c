/*
Copyright (c) 2019 STARFISH PRODUCT ENGINEERING INC.

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


#include "packetReceiver.h"
#include "fastcodeUtil.h"
#include "ports.h"
#include "error.h"
#include "crc1021.h"
#include "timeSync.h"
//#include "queue.h"
#include "packet.h"






//*************************************************
//defines
//*************************************************



//this will be the maximum packet size in 4 byte words.
#define PACKET_BUFFER_LENGTH 1024
#define RECEIVE_LOOP_MAX_COUNT 100 //this is the max number of bytes that will be processed in the receive loop






//*************************************************
//Types
//*************************************************



//*************************************************
//Variables
//*************************************************

ByteQ* inQ = 0;








//queues are used to communicate from slow to fast code.

//Note that 10 bytes can be written or read by fast code in one cycle. This means any head-tail comparison may be off by 2.5 words (call it 10 to be safe)
//Rx queue written by fast code (head), read by slow code (tail)
//this is a slightly weird definition because the type Packet is actually just an int, not an array of ints.
//This works out because a pointer to a packet is really what we are working with and the assumption is that this points to an array of ints.
static Packet packetBuffer[PACKET_BUFFER_LENGTH];



//static uint32_t queueBack = 0;//the packet being received (written to the queue) at the moment by fast code
//static uint32_t queueFront = 0;//the packet being processed (read from the queue) at the moment by slow code




static uint32_t packetRxTime = 0;
static bool packetDecoded = false;


//*************************************************
//function prototypes
//*************************************************



//*************************************************
//code
//*************************************************











/**
 * checks for new packet
 * This should only be called when decoding of a previous packet is complete
 * @return true if packet available
 */
Packet* checkForPacket(void){
	//setup for decoding a new packet
	return packetDecoded ? packetBuffer : 0;
}
void finishedWithPacket(void){
	packetDecoded = false;
}




/**
 * handles receiving packets, , calcs CRC, doing initial decode and validation
 * This could be called in fastcode but may as well be called in slow code
 * TODO: consider making this process more than one byte at a time. In other words, the byteQ might have more than one byte in it when the loop is run.
 */
void packetReceiverLoop(){

	static uint32_t bIndex = 0;
	static uint16_t crcAcc = 0xffff;
	static uint32_t timeout = 0;

	if(inQ == 0 || packetDecoded){
		return;
		resetSlowTimer(&timeout);
	}

	Packet* rxPacket = packetBuffer;

	//keep a timeout timer running to reset the routine if things get stuck
	//there should never be a gap between bytes of greater than a millisecond
	if(slowTimer(&timeout, 1000)){
		bIndex = 0;
	}

	int32_t count = 0;

	//If there's a byte ready and we've done less than 15 bytes this time through
	uint8_t c;
	while(count < RECEIVE_LOOP_MAX_COUNT && getByte(inQ, &c)){
		resetSlowTimer(&timeout);
		++count;

		//if we've reached the end of the buffer then reset
		if(bIndex >= PACKET_BUFFER_LENGTH*4){
			bIndex = 0;
		}

		//setPacketByte(packetIndex, bIndex ^ 0b0011, c);//swap byte order
		setPacketByte(rxPacket, bIndex, c);

		uint32_t bi = bIndex;

		//increment index
		++bIndex;



		switch(bi){
		case 0:
			if(c != 0x55){
				bIndex = 0;//reset rxing because we got the wrong byte
			} else {
				packetRxTime = getLocalTimeMillis();
			}
			break;
		case 1:
			if(c != 0xaa){
				bIndex = 0;//reset rxing because we got the wrong byte
			}
			break;
		case 2:
			if(c != 0x55){
				bIndex = 0;//reset rxing because we got the wrong byte
			}
			break;
		case 3:
			if(c != 0xaa){
				bIndex = 0;//reset rxing because we got the wrong byte
			}
			break;
		case 4:
			//1st byte of packet->crc
			break;
		case 5:
			//2nd byte of packet->crc
			break;
		case 6:
			//1st byte of packet->length
			break;
		case 7:
			//2nd byte of packet->length
			crcAcc = 0xffff;
			break;
		default:


			//might want to consider spotting payload starts and recording them to save time later
			crcAcc = crc1021(crcAcc, c);

			uint32_t dataWordNum = getPacketLength(rxPacket) + 2;//this is the number of data words of the packet
			uint32_t dataWordCount = bIndex >> 2;//this is the number of data words received so far
			if(dataWordNum >= PACKET_BUFFER_LENGTH){
				//we're doomed. The length of this packet will be larger than our buffer.
				//we may as well stop
				bIndex = 0;
			} else if(dataWordCount >= dataWordNum){
				if(crcAcc == getPacketCrc(rxPacket)){

					packetDecoded = true;
				} else {
					//it's a bad packet (failed CRC) so don't register. The routine will reset after this and be ready for a new packet
				}

				bIndex = 0;
			}
			break;

		}
	}





}











uint32_t getTimeOfLastPacket(void){
	return packetRxTime;
}

void initPacketReciever(ByteQ* q){
	inQ = q;
}

