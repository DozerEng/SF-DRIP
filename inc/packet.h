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

#ifndef INC_PACKET_H_
#define INC_PACKET_H_


/**
 * This will likely only be used as a type for a pointer. It mostly only makes sense that way.
 */
typedef uint32_t Packet;

/**
 * gets the next available payload. increments the index until there are no more payloads.
 * When the packet is no longer needed, the packet should be finished. This function does not do that.
 * @param packet a pointer to a packet
 * @param index a pointer to an index for cycling through payloads of the specified packet. The first call with a packet, this should be zero
 * @return the payload or 0 if none left
 */
Payload* getNextPayload(Packet* packet, uint32_t* index);


uint32_t getPacketLength(Packet* packet);

void setPacketByte(Packet* packet, uint32_t byteIndex, uint8_t byte);
uint32_t getPacketCrc(Packet* packet);

#endif /* INC_PACKET_H_ */
