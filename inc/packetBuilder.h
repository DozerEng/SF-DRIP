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

#ifndef INC_PACKETBUILDER_H_
#define INC_PACKETBUILDER_H_

#include <stdint.h>
#include <stdbool.h>
#include "payload.h"
#include "byteQ.h"
/**
 * reset routine in preparation to construct a new packet
 */
void startTxPacket(void);
/**
 * computes CRC in preparation for sending packet
 * This function does not send the packet
 */
void completeTxPacket(void);
/**
 * finish off this packet and queue it
 */
void sendTxPacket(ByteQ* outQ);
/**
 * starts a new payload of the specified type
 */
void startTxPayload(PayloadType payloadType);
/**
 * finishes the currently in-construction payload
 */
void endTxPayload(void);
/**
 * adds an int to the current payload
 */
bool addTxPayloadData(uint32_t data);
/**
 * adds a float to the current payload
 */
bool addTxPayloadFloat(float data);

/**
 * indicates whether a packet has been started
 */
bool isPacketStarted(void);
/**
 * discard the present started packet
 */
void cancelPacket(void);

#endif /* INC_PACKETBUILDER_H_ */
