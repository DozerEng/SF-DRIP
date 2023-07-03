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

#include "packetBuilder.h"
#include "crc1021.h"
#include <ports.h>
#include <appData.h>
//#include "packet.c"

//********************************************************
//Defines
//********************************************************

#define PACKET_BUFFER_LENGTH 500



//********************************************************
//Variable Definitions
//********************************************************
static uint32_t packet[PACKET_BUFFER_LENGTH];
static uint32_t packetIndex = 0;
static uint32_t payloadStartIndex = 0;
static bool packetComplete = false;
static bool packetStarted = false;

//********************************************************



//********************************************************
//Function Prototypes
//********************************************************

/**
 * increases packet length by one
 * @return true if successful. False if packet length is maxed out
 */
static bool incrPacketLength(void);
static void incrPayloadLength(void);



//********************************************************



/**
 * reset routine in preparation to construct a new packet
 */
void startTxPacket(void){
	packetComplete = false;
	packetStarted = true;
	packet[0] = 0xaa55aa55;
	packet[1] = 0xffff0000;//crc,length
	packetIndex = 2;

}
/**
 * computes CRC in preparation for sending packet
 * This function does not send the packet
 */
void completeTxPacket(void){
	if(!packetComplete && packetStarted){
		uint8_t* bs = (uint8_t*)(packet);

		packetComplete = true;
		packetStarted = false;
		uint16_t crc = 0xffff;
//		for(uint32_t i = 8; i < packetIndex*4; ++i){
//			crc = crc1021(crc, bs[i]);
//		}

		crc = crc1021Loop(0xffff, &bs[8], packetIndex*4 - 8);

		packet[1] &= 0xffff;
		packet[1] |= ((uint32_t)crc) << 16;
	}
}
/**
 * send the packet if it is comlete
 */
void sendTxPacket(ByteQ* q){
	if(packetComplete){
		uint8_t* bs = (uint8_t*)(packet);
		uint32_t bNum = packetIndex*4;


		uint32_t n = putBytes(q, bs, bNum);
		if(n < bNum){
			n += putBytes(q, &(bs[n]), bNum - n);
		}

		if(n != bNum){
			asm("nop");
		}

//		for(uint32_t i = 0; i < packetIndex*4; ++i){
//			putByte(q, bs[i]);
//		}


	}

}
/**
 * starts a new payload of the specified type
 */
void startTxPayload(PayloadType type){
	int32_t ip = type;
	ip <<= 16;
	ip &= 0xffff0000;
	payloadStartIndex = packetIndex;
	packet[packetIndex] = ip;
	incrPacketLength();
}
/**
 * finishes the currently in-construction payload
 * Nothing to do here yet
 */
void endTxPayload(void){

}

/**
 * increases packet length by one
 * @return true if successful. False if packet length is maxed out
 */
static bool incrPacketLength(void){
	bool result = true;
	++packetIndex;
	uint32_t ph = packet[1];
	uint32_t pLen = (ph >> 0) & 0xffff;
	++pLen;
	if(pLen >= PACKET_BUFFER_LENGTH){
		--pLen;//pin packet length.
		result = false;
	}
	pLen &= 0xffff;
	ph &= 0xffff0000;
	ph |= pLen;
	packet[1] = ph;
	return result;
}
static void incrPayloadLength(void){
	uint32_t ph = packet[payloadStartIndex];
	uint32_t pLen = (ph >> 0) & 0xff;
	++pLen;
	pLen &= 0xff;
	ph &= 0xffffff00;
	ph |= pLen;
	packet[payloadStartIndex] = ph;
}


/**
 * adds an int to the current payload
 */
bool addTxPayloadData(uint32_t data){
	bool result = false;
	packet[packetIndex] = data;
	if(incrPacketLength()){
		incrPayloadLength();
		result = true;
	} else {
		asm("nop");
	}



	return result;
}
/**
 * adds a float to the current payload
 */
bool addTxPayloadFloat(float data){
	uint32_t* ip = (uint32_t*)(&data);
	return addTxPayloadData(*ip);
}


/**
 * indicates whether a packet has been started
 */
bool isPacketStarted(void){
	return packetStarted;
}
/**
 * discard the present started packet
 */
void cancelPacket(void){
	packetStarted = false;
}
