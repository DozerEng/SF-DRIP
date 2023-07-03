/*
Copyright (c) 2017-2018 STARFISH PRODUCT ENGINEERING INC.

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

#include "stm32f4xx_usart.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "comms.h"
#include "fastcodeUtil.h"
#include "dma.h"
#include <clockSetup.h>



//*************************************************
//defines
//*************************************************
/*!< USART CR1 register clear Mask ((~(uint16_t)0xE9F3)) */
#define CR1_CLEAR_MASK            ((uint16_t)(USART_CR1_M | USART_CR1_PCE | \
                                              USART_CR1_PS | USART_CR1_TE | \
                                              USART_CR1_RE))

/*!< USART CR3 register clear Mask ((~(uint16_t)0xFCFF)) */
#define CR3_CLEAR_MASK            ((uint16_t)(USART_CR3_RTSE | USART_CR3_CTSE))
//*************************************************
//Types
//*************************************************

typedef struct {
	USART_TypeDef* port;
	DMA_Stream_TypeDef* rxStream;
	uint32_t rxChannel;
	DMA_Stream_TypeDef* txStream;
	uint32_t txChannel;
} PortDetails;

typedef struct {
	ByteQ* inQ;
	ByteQ* outQ;
	uint32_t baud;
	uint32_t txInProcess;//number of bytes currently in DMA transaction
	bool useDma;
} PortSetup;

//*************************************************
//Variables
//*************************************************

static const PortDetails portDetails[] = {
		{NULL,   0, 0, 0, 0},//UART_NULL
		{USART1, DMA2_Stream5, DMA_Channel_4, DMA2_Stream7, DMA_Channel_4},
		{USART2, DMA1_Stream5, DMA_Channel_4, DMA1_Stream6, DMA_Channel_4},
		{USART3, DMA1_Stream1, DMA_Channel_4, DMA1_Stream3, DMA_Channel_4},//DMA1_Stream3 is also used by SPI2
		{UART4,  DMA2_Stream2, DMA_Channel_4, DMA1_Stream4, DMA_Channel_4}, //DMA2_Stream2, DMA1_Stream4 shares DMA peripheral and stream with SPI1_RX, SPI2_TX respecively
		{UART5,  DMA1_Stream0, DMA_Channel_4, DMA1_Stream7, DMA_Channel_4}, //DMA1_Stream0 conflicts with SPI3_RX
		{USART6, DMA2_Stream1, DMA_Channel_5, DMA2_Stream6, DMA_Channel_5},
};

static PortSetup portSetup[] = {
		{NULL, NULL, 0, 0, false},//UARTNULL_DEV
		{NULL, NULL, 0, 0, false},
		{NULL, NULL, 0, 0, false},
		{NULL, NULL, 0, 0, false},
		{NULL, NULL, 0, 0, false},
		{NULL, NULL, 0, 0, false},
		{NULL, NULL, 0, 0, false},

};

static UartDev m_lastCommPortUsed = UARTNULL_DEV;

static uint32_t m_baudrateRequest = 115200;
static uint32_t m_timer = 0;
static UartDev m_uart = UARTNULL_DEV;
static uint8_t dataDump;

//*************************************************
//function prototypes
//*************************************************
void oneUartFastCode(UartDev dev);
void baudrateUpdaterFastCode(void);
static void setupRxDma(UartDev dev);
static void setupTxDma(UartDev dev);
static void setupUart(UartDev dev);
void initUsart(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);




//*************************************************
//code
//*************************************************





void initComms(UartDev dev, ByteQ* in, ByteQ* out, uint32_t baud, bool useDma){
	if(dev < 0 || dev > USART6_DEV){
		return;
	}
	portSetup[dev].inQ = in;
	portSetup[dev].outQ = out;
	portSetup[dev].baud = baud;
	portSetup[dev].useDma = useDma;

	setupUart(dev);

	if(useDma){
		setupRxDma(dev);
	}
}

void commsFastCode(void){
//	for(UartDev u = USART1_DEV; u <= USART6_DEV; ++u){
//		oneUartFastCode(u);
//	}
	oneUartFastCode(USART1_DEV);
	oneUartFastCode(USART2_DEV);
	oneUartFastCode(USART3_DEV);
	oneUartFastCode(USART6_DEV);
	baudrateUpdaterFastCode();
}

/** This is the main branch to call from fast code.
 * This implementation does not use DMA
 */
void oneUartFastCode(UartDev dev){
	if(dev < 0 || dev > USART6_DEV){
		return;
	}

	PortSetup ps = portSetup[dev];
	if(ps.inQ == NULL || ps.outQ == NULL || ps.baud == 0){
		return;
	}


	PortDetails pd = portDetails[dev];
	USART_TypeDef* u = pd.port;
	ByteQ* in = ps.inQ;
	ByteQ* out = ps.outQ;
	uint32_t status = pd.port->SR;


	if(ps.useDma){
		if((status & USART_FLAG_ORE) != 0){
			dataDump = (uint8_t)USART_ReceiveData(u);
	//		setupUart(dev);

		}

		int32_t diff = -(int32_t)(DMA_GetCurrDataCounter(pd.rxStream));
		diff += (int32_t)(in->bufferSize);
		in->back = (uint32_t)diff;

		setupTxDma(dev);
	} else {
		//don't use DMA



	// if there's a byte received then put in queue
		if((status & (USART_FLAG_RXNE | USART_FLAG_ORE)) != 0){
			putByte(in,(uint8_t)USART_ReceiveData(u));
			m_lastCommPortUsed = dev;

		}



	//also see if there's a byte to send
		if(getUsed(out) > 0 && USART_GetFlagStatus(u, USART_FLAG_TXE) == SET){
			uint8_t b;
			getByte(out, &b);
			USART_SendData(u, (uint16_t)b);
		}
	}


}
/**
 * @return the last port to receive a byte
 */
UartDev getLastPortToRecieveData(void){
	return m_lastCommPortUsed;
}

/**
 * @Identifies and sets up updater for the port used for host connection
 * Called after each UART is configured via initComms().
 */
void initBaudrateUpdater(UartDev u){
	if(u <= 0 || u > USART6_DEV){
		return;
	}
	m_uart = u;
	updateBaudrate(portSetup[m_uart].baud);

}

void updateBaudrate(uint32_t br){
	m_baudrateRequest = br;



}
uint32_t getBaudrateUpdate(void){
	return m_baudrateRequest;
}
void baudrateUpdaterFastCode(void){
	if(m_uart <= 0 || m_uart > USART6_DEV){
		return;
	}

//	USART_TypeDef* uart = portDetails[m_uart].port;
	PortSetup *ps = &portSetup[m_uart];

	if(( ps->baud != m_baudrateRequest )){
		if(slowTimer(&m_timer, 2000000)){
			ps->baud = m_baudrateRequest;

//
//			USART_InitTypeDef uitd;
////			USART_StructInit(&uitd);
//			uitd.USART_BaudRate = ps->baud;
//			uitd.USART_WordLength = USART_WordLength_8b;
//			uitd.USART_StopBits = USART_StopBits_1;
//			uitd.USART_Parity = USART_Parity_No;
//			uitd.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//TODO:is this correct?
//			uitd.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//
//			USART_Cmd(uart, DISABLE);
//			USART_Init(uart, &uitd);
//			USART_Cmd(uart, ENABLE);

			setupUart(m_uart);
		}
	} else {
		resetSlowTimer(&m_timer);
	}
}


static void setupRxDma(UartDev dev){
	if(dev < 0 || dev > USART6_DEV){
		return;
	}

	PortSetup* ps = &(portSetup[dev]);
	if(ps->inQ == NULL || ps->outQ == NULL || ps->baud == 0){
		return;
	}
	const PortDetails* pd = &(portDetails[dev]);
	USART_TypeDef* u = pd->port;
	ByteQ* q = ps->inQ;//for Rx this is the queue we care about



	//enable RX DMA
	USART_DMACmd(u, USART_DMAReq_Rx, ENABLE);

	//disable stream
	DMA_Cmd(pd->rxStream, DISABLE);

	//setup rx stream. This will take from the SPI data reg and put in memory

	DMA_InitTypeDef ditd;
//	DMA_StructInit(&ditd);
	ditd.DMA_Channel = pd->rxChannel;
	ditd.DMA_PeripheralBaseAddr = (uint32_t)(&(pd->port->DR));
	ditd.DMA_Memory0BaseAddr = (uint32_t)(q->buffer);
	ditd.DMA_DIR = DMA_DIR_PeripheralToMemory;
	ditd.DMA_BufferSize = q->bufferSize;
	ditd.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	ditd.DMA_MemoryInc = DMA_MemoryInc_Enable;
	ditd.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	ditd.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	ditd.DMA_Mode = DMA_Mode_Circular;
	ditd.DMA_Priority = DMA_Priority_VeryHigh;
	ditd.DMA_FIFOMode = DMA_FIFOMode_Disable;
	ditd.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	ditd.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	ditd.DMA_MemoryBurst = DMA_MemoryBurst_Single;


	DMA_Init(pd->rxStream, &ditd);

	//clear all event flags for stream
	clearDmaFlags(pd->rxStream);

	//enable streams
	DMA_Cmd(pd->rxStream, ENABLE);


}
static void setupTxDma(UartDev dev){
	if(dev < 0 || dev > USART6_DEV){
		return;
	}

	PortSetup* ps = &portSetup[dev];
	if(ps->inQ == NULL || ps->outQ == NULL || ps->baud == 0){
		return;
	}
	const PortDetails* pd = &(portDetails[dev]);

	USART_TypeDef* u = pd->port;
	ByteQ* q = ps->outQ;//for tx this is the queue we care about

	//if this stream is already enabled then don't proceed
	if(DMA_GetCmdStatus(pd->txStream) == ENABLE){
		return;
	}

	//move to the next elements in the queue front if there was a transaction in process
	if(ps->txInProcess > 0){
		uint32_t f = q->front;
		f += ps->txInProcess;
		if(f >= q->bufferSize){
			f -= q->bufferSize;
		}
		q->front = f;
		ps->txInProcess = 0;
	}




	//get the number of elements in the queue
	uint32_t n = getUsed(q);

	if(n <= 0){
		return;
	}

	//enable TX DMA
	USART_DMACmd(u, USART_DMAReq_Tx, ENABLE);

	//clear txc bit
	USART_ClearFlag(u, USART_FLAG_TC);

	//disable stream
	DMA_Cmd(pd->txStream, DISABLE);

	//pick the smallest of the number of elements in the queue vs the distance from the front to the queue size
	//that is the number of elements that can be sent out in this transaction.

	uint32_t m = q->bufferSize - q->front;
	if(n > m){
		n = m;
	}

	//record now many bytes we're about to send, so we can advance the queue next time.
	ps->txInProcess = n;
	//setup tx stream. This will take from the SPI data reg and put in memory

	DMA_InitTypeDef ditd;
//	DMA_StructInit(&ditd);
	ditd.DMA_Channel = pd->txChannel;
	ditd.DMA_PeripheralBaseAddr = (uint32_t)(&(pd->port->DR));
	ditd.DMA_Memory0BaseAddr = (uint32_t)(&(q->buffer[q->front]));
	ditd.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	ditd.DMA_BufferSize = n;
	ditd.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	ditd.DMA_MemoryInc = DMA_MemoryInc_Enable;
	ditd.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	ditd.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	ditd.DMA_Mode = DMA_Mode_Normal;
	ditd.DMA_Priority = DMA_Priority_VeryHigh;
	ditd.DMA_FIFOMode = DMA_FIFOMode_Disable;
	ditd.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	ditd.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	ditd.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(pd->txStream, &ditd);

	//clear all event flags for stream

	clearDmaFlags(pd->txStream);




	//enable streams
	DMA_Cmd(pd->txStream, ENABLE);


}
static void setupUart(UartDev dev){


	USART_TypeDef* uart = portDetails[dev].port;

	USART_Cmd(uart, DISABLE);

	USART_OverSampling8Cmd(uart, ENABLE);
//	/USART_SetPrescaler(uart, );


	USART_InitTypeDef uitd;
//	USART_StructInit(&uitd);

	uitd.USART_BaudRate = portSetup[dev].baud;
	uitd.USART_WordLength = USART_WordLength_8b;
	uitd.USART_StopBits = USART_StopBits_1;
	uitd.USART_Parity = USART_Parity_No;
	uitd.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//TODO:is this correct?
	uitd.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	initUsart(uart, &uitd);

	USART_ClockInitTypeDef ucitd;
//	USART_ClockStructInit(&ucitd);
	ucitd.USART_Clock = USART_Clock_Disable;
	ucitd.USART_CPOL = USART_CPOL_Low;
	ucitd.USART_CPHA = USART_CPHA_1Edge;
	ucitd.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(uart, &ucitd);

	USART_Cmd(uart, ENABLE);

}

/**
  * @brief  Initializes the USARTx peripheral according to the specified
  *         parameters in the USART_InitStruct .
  *         this was copied and modified from CMSIS code
  * @param  USARTx: where x can be 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or
  *         UART peripheral.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that contains
  *         the configuration information for the specified USART peripheral.
  * @retval None
  */
void initUsart(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct){
  uint32_t tmpreg = 0x00, apbclock = 0x00;
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;

  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_BAUDRATE(USART_InitStruct->USART_BaudRate));
  assert_param(IS_USART_WORD_LENGTH(USART_InitStruct->USART_WordLength));
  assert_param(IS_USART_STOPBITS(USART_InitStruct->USART_StopBits));
  assert_param(IS_USART_PARITY(USART_InitStruct->USART_Parity));
  assert_param(IS_USART_MODE(USART_InitStruct->USART_Mode));
  assert_param(IS_USART_HARDWARE_FLOW_CONTROL(USART_InitStruct->USART_HardwareFlowControl));

  /* The hardware flow control is available only for USART1, USART2, USART3 and USART6 */
  if (USART_InitStruct->USART_HardwareFlowControl != USART_HardwareFlowControl_None){
    assert_param(IS_USART_1236_PERIPH(USARTx));
  }

/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;

  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);

  /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit :
      Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;

  /* Write to USART CR2 */
  USARTx->CR2 = (uint16_t)tmpreg;

/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USARTx->CR1;

  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);

  /* Configure the USART Word Length, Parity and mode:
     Set the M bits according to USART_WordLength value
     Set PCE and PS bits according to USART_Parity value
     Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity |
            USART_InitStruct->USART_Mode;

  /* Write to USART CR1 */
  USARTx->CR1 = (uint16_t)tmpreg;

/*---------------------------- USART CR3 Configuration -----------------------*/
  tmpreg = USARTx->CR3;

  /* Clear CTSE and RTSE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR3_CLEAR_MASK);

  /* Configure the USART HFC :
      Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
  tmpreg |= USART_InitStruct->USART_HardwareFlowControl;

  /* Write to USART CR3 */
  USARTx->CR3 = (uint16_t)tmpreg;

/*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate */

  if ((USARTx == USART1) || (USARTx == USART6)) {
    apbclock = (uint32_t)getApb2ClockHz();
  } else {
    apbclock = (uint32_t)getApb1ClockHz();
  }


  /* Determine the integer part */
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0) {
    /* Integer part computing in case Oversampling mode is 8 Samples */
    integerdivider = ((25 * apbclock) / (2 * (USART_InitStruct->USART_BaudRate)));
  } else {
    /* Integer part computing in case Oversampling mode is 16 Samples */
    integerdivider = ((25 * apbclock) / (4 * (USART_InitStruct->USART_BaudRate)));
  }
  tmpreg = (integerdivider / 100) << 4;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0){
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  } else {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }

  /* Write to USART BRR register */
  USARTx->BRR = (uint16_t)tmpreg;
}


