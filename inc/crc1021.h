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

#ifndef INC_CRC1021_H_
#define INC_CRC1021_H_
#include <stdint.h>
/**
 * Updates the present CRC value with one new byte
 * Assumes the register initialized to 0xffff the first time
 * code copied from page that linked to this:
 * http://www.dattalo.com/technical/software/pic/crc_1021.asm
 * Link above seems broken
 * @param startCrc - the starting value, if this is to be run over the complete data the start should be set to 0xffff, otherwise it should be the previous, intermediate crc result
 * @param data the latest byte to cycle through the CRC computation
 * @return the updated CRC result after cycling through the new data
 */
uint16_t crc1021(uint16_t old_crc, uint8_t data);
/**
 * runs crc on a byte array. This might be more efficient than having an external loop call crc_1021 multiple times
 * Seems to be a bit faster, maybe 4 to 6 instruction cycles per byte?
 * @param startCrc - the starting value, if this is to be run over the complete data the start should be set to 0xffff
 * @param data the latest bytes to cycle through the CRC computation
 * @param count the number of bytes to compute over. That means it will execute from *data to *data + count - 1 inclusive.
 * @return the updated CRC result after cycling through the new data
 */
uint16_t crc1021Loop(uint16_t start, uint8_t* data, uint32_t count);

#endif /* INC_CRC1021_H_ */
