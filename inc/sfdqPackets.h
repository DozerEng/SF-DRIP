/*
Copyright (c) 2018 STARFISH PRODUCT ENGINEERING INC.

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

#ifndef SFDQPACKETS_H_
#define SFDQPACKETS_H_

#include <stdbool.h>
#include "byteQ.h"
#include "payload.h"



bool decodeSfdqPacket(void);
/**
 * Send the currently completely packet down the specified byte queue
 * This will quietly refuse if the queue is not empty
 */
void sendSfdqPacket(ByteQ* txQ);
bool isQueueEmpty(ByteQ* txQ);

bool wasPacketJustAQuery(void);
void makePayload(PayloadType type);
void  startPacketIfNecessary(bool includeId);


/**
 * indicates that a tx packet is currently being built
 */
bool isTxPacketInProgress(void);
/**
* sets whether all payloads will be accepted, regardless of whether the received ID matches this SDFQ
 */
void setAcceptAllPayloads(bool a);

#endif /* SFDQPACKETS_H_ */
