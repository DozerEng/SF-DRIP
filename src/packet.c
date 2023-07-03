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



//*************************************************
//includes
//*************************************************

#include "payload.h"
#include "packet.h"


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
//Payload* getPayload(Packet* packet, uint32_t payloadIndex);



//*************************************************
//code
//*************************************************


/**
 * gets the next available payload. increments the index until there are no more payloads.
 * When the packet is no longer needed, the packet should be finished. This function does not do that.
 * @param packet a pointer to a packet
 * @param index a pointer to an index for cycling through payloads of the specified packet. The first call with a packet, this should be zero
 * @return the payload or 0 if none left
 */
Payload* getNextPayload(Packet* packet, uint32_t* index){

	Payload* result = 0;
	//keeps track of the state of building the response packet

	uint32_t packetEnd = getPacketLength(packet)+2;
	if(*index == 0){
		*index =  2;
	}


	if(*index < packetEnd){
		//cast as pointer to payload and then index and get reference.
//		result = &(((Payload*)packet)[*index]);
		result = (Payload*)packet + *index;
	}

	*index += getPayloadLength(result) + 1;



	return result;
}



//Payload* getPayload(Packet* packet, uint32_t payloadIndex){
//
//	return (Payload*)(&(packet[payloadIndex]));
//}



uint32_t getPacketLength(Packet* packet){
	uint32_t result = packet[1] & 0xffff;
	return result;
}
uint32_t getPacketCrc(Packet* packet){
	uint32_t result = packet[1] >> 16;
	return result;
}

/**
 * Sets a byte in a packet
 * This is intended for building a packet from received bytes
 * It is assumed that the location being written to is big enough for the specified index
 * @param packet the location to write the packet byte
 * @param the index to write the byte. This must not over-reach the allocated space. This function does not check.
 * @byte the value of the byte
 */
void setPacketByte(Packet* packet, uint32_t byteIndex, uint8_t byte){

	uint8_t* bytes = (uint8_t*)packet;
	bytes[byteIndex] = byte;

}

