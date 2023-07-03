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

#include "payload.h"

uint32_t getPayloadWord(Payload* payload, uint8_t index){
	uint32_t len = getPayloadLength(payload) + 1;
	uint32_t result = 0;
	if(index < len){
		result = ((uint32_t*)payload)[index];
	}
	return result;
}
void setPayloadWord(Payload* payload, uint8_t index, uint32_t value){
	uint32_t len = getPayloadLength(payload);

	if(index < len){
		uint32_t* ph = ((uint32_t*)payload);
		*ph = value;
	}
}

uint32_t getPayloadLength(Payload* payload){
	uint32_t result = 0;
	if(payload != 0){
		uint32_t* ph = ((uint32_t*)payload);
		result = (*ph) & 0x0000ffff;
	}
	return result;
}



//void setPayloadLength(Payload payload, int32_t length){
//	uint32_t t = getPayloadWord(payload, 0);
//	t &= 0xffff0000;//clear the bottom two bytes
//	t |= (length << 0) & 0x0000ffff;//replace with new length
//	setPayloadWord(payload, 0, length);
//	//payload[0] = length;
//}
PayloadType getPayloadType(Payload* payload){
	uint32_t* ph = ((uint32_t*)payload);
	uint32_t pt = ((*ph)>>16) & 0xff;
	return (PayloadType)pt;
}
//void setPayloadType(Payload payload, PayloadType type){
//	uint32_t* ph = ((uint32_t*)payload);
//	uint32_t t = *ph;
//	t &= 0x0000ffff;//clear the top short
//	t |= (((uint32_t)type) << 16) & 0xffff0000;//replace with new crc
//	payload[0] = t;
//}
float getPayloadFloat(Payload* payload, uint8_t index){
	uint32_t ip = getPayloadWord(payload, index);
	float* fp = (float*)(&ip);
	return *fp;
}
void setPayloadFloat(Payload* payload, uint8_t index, float value){
	float* fp = &value;
	uint32_t* ip = (uint32_t*)fp;
	setPayloadWord(payload, index, *ip);
}


