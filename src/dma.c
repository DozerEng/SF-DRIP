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
#include "dma.h"

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


//*************************************************
//code
//*************************************************

void clearDmaFlags(DMA_Stream_TypeDef* stream){
	if(stream == DMA2_Stream0 || stream == DMA1_Stream0){
		DMA_ClearFlag(stream, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TEIF0);
	} else if(stream == DMA2_Stream1 || stream == DMA1_Stream1){
		DMA_ClearFlag(stream, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1);
	} else if(stream == DMA2_Stream2 || stream == DMA1_Stream2){
		DMA_ClearFlag(stream, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TEIF2);
	} else if(stream == DMA2_Stream3 || stream == DMA1_Stream3){
		DMA_ClearFlag(stream, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TEIF3);
	} else if(stream == DMA2_Stream4 || stream == DMA1_Stream4){
		DMA_ClearFlag(stream, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TEIF4);
	} else if(stream == DMA2_Stream5 || stream == DMA1_Stream5){
		DMA_ClearFlag(stream, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5);
	} else if(stream == DMA2_Stream6 || stream == DMA1_Stream6){
		DMA_ClearFlag(stream, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6);
	} else if(stream == DMA2_Stream7 || stream == DMA1_Stream7){
		DMA_ClearFlag(stream, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_TEIF7);
	}
}
