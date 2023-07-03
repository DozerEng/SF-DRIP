/*
Copyright (c) 2012-2018 STARFISH PRODUCT ENGINEERING INC.

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

 

#include <i2c.h>
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <queue.h>
#include <ports.h>
#include <fastcodeUtil.h>
#include <clockSetup.h>

//********************************************************************************
//defines
//********************************************************************************

#define NUM_I2C_DEVICES 3
#define CODEC_I2C_FREQ 20000     // fast mode (400kHz) requires a multiple of 10MHz for the peripheral clock - which is 42MHz, so we can't use that
#define I2C_TIMEOUT  2500  //microseconds. sufficient time to complete a single I2C operation
#define I2C_QUEUE_LENGTH 200 //this is the number of elements in each I2C bus queue


#define CR1_CLEAR_MASK    ((uint16_t)0xFBF5)      /*<! I2C registers Masks */
#define FLAG_MASK         ((uint32_t)0x00FFFFFF)  /*<! I2C FLAG mask */
#define ITEN_MASK         ((uint32_t)0x07000000)  /*<! I2C Interrupt Enable mask */


//********************************************************************************
//scructs and enums
//********************************************************************************



// the states which are used within this file (driver) to control the intricacies of I2C communication
typedef enum{
	IDLE,
	TX_WAIT_FOR_BUS_NOT_BUSY,
	TX_WAIT_FOR_CONTROLLER_MODE_SELECTED,
	TX_WAIT_FOR_CONTROLLER_TX_MODE_SELECTED,
	TX_SEND_DATA,
	TX_WAIT_FOR_CONTROLLER_BYTE_TRANSMITTED,

	RX_WAIT_FOR_BUS_NOT_BUSY,
	RX_WAIT_FOR_CONTROLLER_MODE_SELECTED,
	RX_WAIT_FOR_CONTROLLER_RX_MODE_SELECTED,
	RX_RECEIVE_DATA,
	RX_WAIT_FOR_CONTROLLER_BYTE_RECEIVED,
	SW_RESET,
	BAIL_SEND_CLOCKS,

} I2cState;

// a structure and array of variables for holding the states of the I2C transactions
typedef struct {
	uint8_t     i2cAddr;//this is the 7 bit address, shifted left one bit
	uint8_t     *tx_buff;
	uint32_t        num_bytes_to_tx;
	uint32_t        num_bytes_txd;
	uint8_t    *rx_buff;
	uint32_t        num_bytes_to_rx;
	uint32_t        num_bytes_rxd;
	TransactionStatus *transaction_status;               // a simplified set of states (plus timeout), used to publish the transaction state
} I2cTransaction;

typedef struct {
	I2cTransaction i2c_transaction[I2C_QUEUE_LENGTH];
	I2cState state;
	uint32_t queueBack;
	uint32_t queueFront;
	uint32_t timeoutCounter;
	uint32_t resetTimesCounter;
	uint32_t busyTimeoutTimesCounter;
} I2cDevState;

//********************************************************************************
//variables
//********************************************************************************


static I2C_TypeDef *i2c_devices[NUM_I2C_DEVICES] = {
	I2C1,
	I2C2,
	I2C3
};


//static I2c_Transaction_t i2c_transaction[NUM_I2C_DEVICES][I2C_QUEUE_LENGTH];
//static I2c_State_t i2cState[NUM_I2C_DEVICES];
//static uint32_t queueBack[NUM_I2C_DEVICES];
//static uint32_t queueFront[NUM_I2C_DEVICES];
//static uint32_t timeout_cntr[NUM_I2C_DEVICES];
//static uint32_t resetTimesCounter[NUM_I2C_DEVICES];
//static uint32_t busyTimeoutTimesCounter[NUM_I2C_DEVICES];

static I2cDevState i2cDevState[NUM_I2C_DEVICES];



//********************************************************************************
//function prototypes
//********************************************************************************


static void setStatus(I2cTransaction *t, TransactionStatus s);
static void setI2cState(I2cDev dev, I2cState s);
static void oneBusFastCode(I2cDev dev);
static void setupI2c(I2cDev dev);
static void finishTransaction(I2cDev dev, I2cTransaction *t, TransactionStatus status);
/**
 * copied from st libraries to remove HSE_VALUE reference
 */
void initI2c(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);


//********************************************************************************



static void setupI2c(I2cDev dev){
	I2C_InitTypeDef          i2c_init_struct;
	i2c_init_struct.I2C_ClockSpeed          = CODEC_I2C_FREQ;
	i2c_init_struct.I2C_Mode                = I2C_Mode_I2C;
	i2c_init_struct.I2C_DutyCycle           = I2C_DutyCycle_2;    // 50% duty cycle - not using fast mode, so this is not relevant
	i2c_init_struct.I2C_OwnAddress1         = 0;                  // device is operating in controller-mode, so own-address is not relevant
	i2c_init_struct.I2C_Ack                 = I2C_Ack_Disable;
	i2c_init_struct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// Write to timer registers
	I2C_TypeDef* i2c = i2c_devices[dev];


//	I2C_SoftwareResetCmd(i2c, ENABLE);
//TODO: this uses HSE_VALUE inside. This should be scrubbed.
	initI2c(i2c, &i2c_init_struct);

	// I2C enable
	I2C_Cmd(i2c, ENABLE);

}

void i2cInit(I2cDev dev){
	setupI2c(dev);

	uint32_t i;

	for (i = 0; i < NUM_I2C_DEVICES; i++){
		I2cDevState *s = &(i2cDevState[dev]);
		s->state = IDLE;
		s->queueBack = 0;
		s->queueFront = 0;
		s->resetTimesCounter = 0;
		s->busyTimeoutTimesCounter = 0;
//		i2cState[i] = IDLE;
//		queueBack[i] = 0;
//		queueFront[i] = 0;
//		resetTimesCounter[i] = 0;
//		busyTimeoutTimesCounter[i] = 0;
	}
}




TransactionStatus i2cGetStatus(I2cDev dev){
	if(dev == I2CNULL_DEV){
		return NO_TRANSACTION_YET;
	}
	I2cDevState *s = &(i2cDevState[dev]);
	return *(s->i2c_transaction[s->queueFront].transaction_status);
}


bool i2cQueue(I2cDev dev, uint8_t slaveAddrShiftOne, uint8_t *txData, uint32_t numBytesToWrite, uint8_t *rxData, uint32_t numBytesToRead, TransactionStatus *status){
	if(dev == I2CNULL_DEV){
		*status = TRANSACTION_QUEUE_FULL;
		return false;
	}
	bool result = false;
	I2cDevState *s = &(i2cDevState[dev]);

	if(isQueueNotFull(&(s->queueFront), &(s->queueBack), I2C_QUEUE_LENGTH)){//check to be sure we haven't over-run the queue
		result = true;
		I2cDevState *s = &(i2cDevState[dev]);
		I2cTransaction *t = &(s->i2c_transaction[s->queueBack]);



		// set up the transaction
		t->i2cAddr         = slaveAddrShiftOne;
		t->tx_buff            = txData;
		t->num_bytes_to_tx    = numBytesToWrite;
		t->num_bytes_txd      = 0;
		t->rx_buff            = rxData;
		t->num_bytes_to_rx    = numBytesToRead;
		t->num_bytes_rxd      = 0;
		t->transaction_status = status;
		*status = TRANSACTION_QUEUED;

		justAddedToQueueBack(&(s->queueFront), &(s->queueBack), I2C_QUEUE_LENGTH);
	}
	return result;
}


void i2cFastcode(){
//	for(I2cDev i = I2C1_DEV; i <= I2C3_DEV; ++i){
//		oneBusFastCode(i);
//	}


	//33.3us

	oneBusFastCode(I2C1_DEV);

	//34.9us


	oneBusFastCode(I2C2_DEV);

	//38.7us

	oneBusFastCode(I2C3_DEV);

	//42.5



}



// call at a periodic interval, preferably faster than 1ms intervals for all active i2c devices (and definitely faster than 2ms, or the timeout will trip for no good reason)
//must be called for each i2c dev
static void oneBusFastCode(I2cDev dev){

	if(dev == I2CNULL_DEV || dev > I2C3_DEV){
		return;
	}
	I2cTransaction *transac;
	I2C_TypeDef       *i2c_dev;
	I2cDevState *s = &(i2cDevState[dev]);

	transac = &(s->i2c_transaction[s->queueFront]);
	i2c_dev     = i2c_devices[dev];

	I2cState state = s->state;

	switch (state){
	case IDLE:
		if (I2C_GetFlagStatus(i2c_dev, I2C_FLAG_BUSY)){
			//for some reason the bus is busy. Let's keep an eye on it and timeout if it persists

		} else if(isQueueNotEmpty(&(s->queueFront), &(s->queueBack), I2C_QUEUE_LENGTH)){
			I2C_ClearFlag(i2c_dev, I2C_FLAG_AF);
			//if the queue is not empty then we're already pointing at the transaction to process
			setStatus(transac, TRANSACTION_IN_PROGRESS);
//			setI2cState(dev, TX_WAIT_FOR_BUS_NOT_BUSY);

			if(transac->num_bytes_to_tx > 0){
				//we need to transmit before receiving
				I2C_GenerateSTART(i2c_dev, ENABLE);    // send the I2C start condition
				setI2cState(dev, TX_WAIT_FOR_CONTROLLER_MODE_SELECTED);
			} else if(transac->num_bytes_to_rx > 0){
				// we need to receive right away
				I2C_GenerateSTART(i2c_dev, ENABLE);    // send the I2C start condition
				setI2cState(dev, RX_WAIT_FOR_CONTROLLER_MODE_SELECTED); // move on to Rx
			} else {
				//oddly, there's nothing to do
				finishTransaction(dev, transac, TRANSACTION_COMPLETED_OK);
			}
			resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
		} else {
			resetSlowTimer(&(s->timeoutCounter));//reset timer because we're just idle and should not timeout
		}


		break;


	case TX_WAIT_FOR_CONTROLLER_MODE_SELECTED:
		//check that the start was sent. This should not take much time at all. We could use a shorter timeout here
		if (I2C_CheckEvent(i2c_dev, I2C_EVENT_MASTER_MODE_SELECT)){
			//the start was successfully transmitted so send the i2c address
			I2C_Send7bitAddress(i2c_dev, transac->i2cAddr, I2C_Direction_Transmitter);
			setI2cState(dev, TX_WAIT_FOR_CONTROLLER_TX_MODE_SELECTED);
			resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
		}else {
			//not sure why we would ever get here and timeout
		}
		break;
	case TX_WAIT_FOR_CONTROLLER_TX_MODE_SELECTED:
		//check for an ack after the address is sent. This will take the time of 9 bits being sent
		//for 100kHz this will be 90us. For 10kHz this would be 900us.
		if (I2C_CheckEvent(i2c_dev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
			//the address was ACKed so transition to next state
			setI2cState(dev, TX_SEND_DATA);
			resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
		} else if(SET == I2C_GetFlagStatus(i2c_dev, I2C_FLAG_AF)){
			//the address was NACKed so end the transaction
			I2C_GenerateSTOP(i2c_dev, ENABLE);         // send the I2C stop condition
			finishTransaction(dev, transac, TRANSACTION_TX_ADDRESS_NACK);
		} else {
			//not sure why we would ever get here and timeout
		}
		break;
	case TX_SEND_DATA:
		//this state never needs a timeout. It will always transition to another state right away
		if (transac->num_bytes_to_tx > 0){
			//there are bytes to transmit so send one
			I2C_SendData(i2c_dev, transac->tx_buff[transac->num_bytes_txd]);
			transac->num_bytes_to_tx--;
			transac->num_bytes_txd++;
			setI2cState(dev, TX_WAIT_FOR_CONTROLLER_BYTE_TRANSMITTED);
			resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
		} else {
			if (transac->num_bytes_to_rx == 0){
				I2C_GenerateSTOP(i2c_dev, ENABLE);     // send the I2C stop condition
				setI2cState(dev, IDLE);     // nothing more to send, return to idle
				setStatus(transac, TRANSACTION_COMPLETED_OK);
				resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
				doneWithQueueFront(&(s->queueFront), &(s->queueBack), I2C_QUEUE_LENGTH);
			} else {
				// we need to receive byte(s) now
				I2C_GenerateSTART(i2c_dev, ENABLE);    // send the I2C start condition
				setI2cState(dev, RX_WAIT_FOR_CONTROLLER_MODE_SELECTED); // move on to Rx
			}
		}
		break;
	case TX_WAIT_FOR_CONTROLLER_BYTE_TRANSMITTED:
		//we're waiting for the last transmitted byte to complete. This should take 9 clocks.
		if (I2C_CheckEvent(i2c_dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
			setI2cState(dev, TX_SEND_DATA);
			resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
		} else if(SET == I2C_GetFlagStatus(i2c_dev, I2C_FLAG_AF)){
			//that last transmission was NACKed so stop transmitting
			I2C_GenerateSTOP(i2c_dev, ENABLE);         // send the I2C stop condition
			finishTransaction(dev, transac, TRANSACTION_TX_DATA_NACK);
		} else {
			//not sure why we would ever get here and timeout
		}
		break;

	// Rx -----------------------------------------------------------------
//	case RX_WAIT_FOR_BUS_NOT_BUSY: //Do we need to do this check?
//		if (!I2C_GetFlagStatus(i2c_dev, I2C_FLAG_BUSY)){
//			I2C_GenerateSTART(i2c_dev, ENABLE);        // send the I2C start condition
//			setI2cState(dev, RX_WAIT_FOR_MASTER_MODE_SELECTED);
//			resetSlowTimer(&(timeout_cntr[dev]));//reset timer because we've successfully completed this step
//		}
//		break;
	case RX_WAIT_FOR_CONTROLLER_MODE_SELECTED:
		//here we are waiting for a start to complete. This should take no time at all.
		if (I2C_CheckEvent(i2c_dev, I2C_EVENT_MASTER_MODE_SELECT)){
			I2C_Send7bitAddress(i2c_dev, transac->i2cAddr, I2C_Direction_Receiver);
			setI2cState(dev, RX_WAIT_FOR_CONTROLLER_RX_MODE_SELECTED);
			resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
		} else {
			//not sure why we would ever get here and timeout
		}
		break;
	case RX_WAIT_FOR_CONTROLLER_RX_MODE_SELECTED:
		//this should take 9 bits or so. Let's say 900us at most at 10kHz
		if (I2C_CheckEvent(i2c_dev, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){//check for an ack after the address is sent
			setI2cState(dev, RX_RECEIVE_DATA);
			resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
		} else if(SET == I2C_GetFlagStatus(i2c_dev, I2C_FLAG_AF)){
			//we were NACKed so fail end this transaction right away
			I2C_GenerateSTOP(i2c_dev, ENABLE);         // send the I2C stop condition
			finishTransaction(dev, transac, TRANSACTION_RX_ADDRESS_NACK);

		} else {
			//not sure why we would ever get here and timeout
		}
		break;
	case RX_RECEIVE_DATA:
		if (transac->num_bytes_to_rx){
			if (transac->num_bytes_to_rx > 1){
				I2C_AcknowledgeConfig(i2c_dev, ENABLE);  // ack  (more bytes expected)
				setI2cState(dev, RX_WAIT_FOR_CONTROLLER_BYTE_RECEIVED);
			} else {
				I2C_AcknowledgeConfig(i2c_dev, DISABLE); // nack (last byte)
				setI2cState(dev, RX_WAIT_FOR_CONTROLLER_BYTE_RECEIVED);
				resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
			}
		} else {
			I2C_GenerateSTOP(i2c_dev, ENABLE);         // send the I2C stop condition
			finishTransaction(dev, transac, TRANSACTION_COMPLETED_OK);
		}
		break;
	case RX_WAIT_FOR_CONTROLLER_BYTE_RECEIVED:
			if (I2C_CheckEvent(i2c_dev, I2C_EVENT_MASTER_BYTE_RECEIVED)){
				if(transac->rx_buff != NULL){//don't write here if it's null!
					transac->rx_buff[transac->num_bytes_rxd] = I2C_ReceiveData(i2c_dev);
				}
				transac->num_bytes_to_rx--;
				transac->num_bytes_rxd++;
				setI2cState(dev, RX_RECEIVE_DATA);
				resetSlowTimer(&(s->timeoutCounter));//reset timer because we've successfully completed this step
			} else if(SET == I2C_GetFlagStatus(i2c_dev, I2C_FLAG_AF)){
				I2C_GenerateSTOP(i2c_dev, ENABLE);         // send the I2C stop condition
				finishTransaction(dev, transac, TRANSACTION_RX_DATA_NACK);
			} else if(SET == I2C_GetFlagStatus(i2c_dev, I2C_FLAG_BERR)){

//				I2C_ClearFlag(i2c_dev, I2C_FLAG_BERR);
//				I2C_GenerateSTOP(i2c_dev, ENABLE);         // send the I2C stop condition
//				finishTransaction(dev, transac, TRANSACTION_RX_BUS_ERROR);

				//TODO let's just see what happens when I assume it's all good!
				if(transac->rx_buff != NULL){//don't write here if it's null!
					transac->rx_buff[transac->num_bytes_rxd] = I2C_ReceiveData(i2c_dev);
				}
				transac->num_bytes_to_rx--;
				transac->num_bytes_rxd++;
				setI2cState(dev, RX_RECEIVE_DATA);
			} else {
				//not sure why we would ever get here and timeout
			}
		break;
	case SW_RESET:
		I2C_SoftwareResetCmd(i2c_dev, DISABLE);
		setupI2c(dev);
		setI2cState(dev, IDLE);
		break;


	case BAIL_SEND_CLOCKS:

		if(I2C_GetFlagStatus(i2c_dev, I2C_FLAG_BUSY)){
			setI2cState(dev, SW_RESET);
			I2C_SoftwareResetCmd(i2c_dev, ENABLE);
		} else {
			setI2cState(dev, IDLE);
		}
		break;
	default:                                           // Note: might want to do something more severe than this
		finishTransaction(dev, transac, TRANSACTION_COMPLETED_OK);
		break;
	}

	if (slowTimer(&(s->timeoutCounter), I2C_TIMEOUT)){
		//we could get here by being idle but stuck in a busy state, or
		// we are stuck in a non-idle state for too long - so:
		//   Reset the I2C port - Note: Leave this for future development.
		//       It requires significant testing and ensuring that resetting any used I2C port doesn't break the software using it.
		if(state == IDLE){
			//we must be busy because every other path through IDLE state resets the timout timer


			setI2cState(dev, BAIL_SEND_CLOCKS);

			++(s->busyTimeoutTimesCounter);

		} else {
			//we're stuck in some non-idle state
			setI2cState(dev, SW_RESET);
			I2C_SoftwareResetCmd(i2c_dev, ENABLE);
//			I2C_GenerateSTOP(i2c_dev, ENABLE);
			setStatus(transac, TRANSACTION_TIMED_OUT);
			doneWithQueueFront(&(s->queueFront), &(s->queueBack), I2C_QUEUE_LENGTH);

			++(s->resetTimesCounter);
		}

	}
}

static void finishTransaction(I2cDev dev, I2cTransaction *t, TransactionStatus status){
	setStatus(t, status);
	setI2cState(dev, IDLE);
	I2cDevState *s = &(i2cDevState[dev]);
	doneWithQueueFront(&(s->queueFront), &(s->queueBack), I2C_QUEUE_LENGTH);
}

static void setStatus(I2cTransaction *t, TransactionStatus status){
	TransactionStatus* ts = t->transaction_status;

	*ts = status;
}

/**
 * sets the state for the current bus transaction
 * dev must not be null
 */
static void setI2cState(I2cDev dev, I2cState newState){
//	if(dev == I2CNULL_DEV){
//		return;
//	}
	I2cDevState *s = &(i2cDevState[dev]);
	s->state = newState;
	resetSlowTimer(&(s->timeoutCounter));
}

/**
 * queries the amount of room left in the queue.
 * It is advised to call this prior to adding to the queue to ensure there's room
 */
uint32_t getI2cQueueRoom(I2cDev dev){
	uint32_t result = 0;
	if(dev >= I2C1_DEV && dev <= I2C3_DEV){
		I2cDevState *s = &(i2cDevState[dev]);
		result = getQueueRoom(&(s->queueFront), &(s->queueBack), I2C_QUEUE_LENGTH);
	}
	return result;
}

uint32_t getI2cResetTimeoutCount(I2cDev dev){
	uint32_t result = 0;
	if(dev >= I2C1_DEV && dev <= I2C3_DEV){
		I2cDevState *s = &(i2cDevState[dev]);
		result = s->resetTimesCounter;
	}
	return result;
}
uint32_t getI2cBusyTimeoutCount(I2cDev dev){
	uint32_t result = 0;
	if(dev >= I2C1_DEV && dev <= I2C3_DEV){
		I2cDevState *s = &(i2cDevState[dev]);
		result = s->busyTimeoutTimesCounter;
	}
	return result;
}

/**
  * @brief  Initializes the I2Cx peripheral according to the specified
  *         parameters in the I2C_InitStruct.
  *
  *copied from ST libraries to remove reference to HSE_VALUE
  * @note   To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency
  *         (I2C peripheral input clock) must be a multiple of 10 MHz.
  *
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_InitStruct: pointer to a I2C_InitTypeDef structure that contains
  *         the configuration information for the specified I2C peripheral.
  * @retval None
  */
void initI2c(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct){
  uint16_t tmpreg = 0, freqrange = 0;
  uint16_t result = 0x04;
  uint32_t pclk1 = 8000000;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_CLOCK_SPEED(I2C_InitStruct->I2C_ClockSpeed));
  assert_param(IS_I2C_MODE(I2C_InitStruct->I2C_Mode));
  assert_param(IS_I2C_DUTY_CYCLE(I2C_InitStruct->I2C_DutyCycle));
  assert_param(IS_I2C_OWN_ADDRESS1(I2C_InitStruct->I2C_OwnAddress1));
  assert_param(IS_I2C_ACK_STATE(I2C_InitStruct->I2C_Ack));
  assert_param(IS_I2C_ACKNOWLEDGE_ADDRESS(I2C_InitStruct->I2C_AcknowledgedAddress));

/*---------------------------- I2Cx CR2 Configuration ------------------------*/
  /* Get the I2Cx CR2 value */
  tmpreg = I2Cx->CR2;
  /* Clear frequency FREQ[5:0] bits */
  tmpreg &= (uint16_t)~((uint16_t)I2C_CR2_FREQ);


  pclk1 = (uint32_t)getApb2ClockHz();
  /* Set frequency bits depending on pclk1 value */
  freqrange = (uint16_t)(pclk1 / 1000000);
  tmpreg |= freqrange;
  /* Write to I2Cx CR2 */
  I2Cx->CR2 = tmpreg;

/*---------------------------- I2Cx CCR Configuration ------------------------*/
  /* Disable the selected I2C peripheral to configure TRISE */
  I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_PE);
  /* Reset tmpreg value */
  /* Clear F/S, DUTY and CCR[11:0] bits */
  tmpreg = 0;

  /* Configure speed in standard mode */
  if (I2C_InitStruct->I2C_ClockSpeed <= 100000){
    /* Standard mode speed calculate */
    result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed << 1));
    /* Test if CCR value is under 0x4*/
    if (result < 0x04){
      /* Set minimum allowed value */
      result = 0x04;
    }
    /* Set speed value for standard mode */
    tmpreg |= result;
    /* Set Maximum Rise Time for standard mode */
    I2Cx->TRISE = freqrange + 1;

  /* Configure speed in fast mode */
  /* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
     input clock) must be a multiple of 10 MHz */
  } else {/*(I2C_InitStruct->I2C_ClockSpeed <= 400000)*/

    if (I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_2){
      /* Fast mode speed calculate: Tlow/Thigh = 2 */
      result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 3));
    } else {/*I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_16_9*/

      /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
      result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 25));
      /* Set DUTY bit */
      result |= I2C_DutyCycle_16_9;
    }

    /* Test if CCR value is under 0x1*/
    if ((result & I2C_CCR_CCR) == 0){

      /* Set minimum allowed value */
      result |= (uint16_t)0x0001;
    }
    /* Set speed value and set F/S bit for fast mode */
    tmpreg |= (uint16_t)(result | I2C_CCR_FS);
    /* Set Maximum Rise Time for fast mode */
    I2Cx->TRISE = (uint16_t)(((freqrange * (uint16_t)300) / (uint16_t)1000) + (uint16_t)1);
  }

  /* Write to I2Cx CCR */
  I2Cx->CCR = tmpreg;
  /* Enable the selected I2C peripheral */
  I2Cx->CR1 |= I2C_CR1_PE;

/*---------------------------- I2Cx CR1 Configuration ------------------------*/
  /* Get the I2Cx CR1 value */
  tmpreg = I2Cx->CR1;
  /* Clear ACK, SMBTYPE and  SMBUS bits */
  tmpreg &= CR1_CLEAR_MASK;
  /* Configure I2Cx: mode and acknowledgement */
  /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
  /* Set ACK bit according to I2C_Ack value */
  tmpreg |= (uint16_t)((uint32_t)I2C_InitStruct->I2C_Mode | I2C_InitStruct->I2C_Ack);
  /* Write to I2Cx CR1 */
  I2Cx->CR1 = tmpreg;

/*---------------------------- I2Cx OAR1 Configuration -----------------------*/
  /* Set I2Cx Own Address1 and acknowledged address */
  I2Cx->OAR1 = (I2C_InitStruct->I2C_AcknowledgedAddress | I2C_InitStruct->I2C_OwnAddress1);
}
