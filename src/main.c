


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
#include <math.h>
#include <ad7172.h>
#include <adcManager.h>
#include <bootloader.h>
#include <byteQ.h>
//#include <cat24C256.h>
#include <comms.h>
#include <dac.h>
#include <encoder.h>
#include <error.h>
#include <fastcode.h>
#include <fastcodeUtil.h>
#include <feedbackControl.h>
#include <i2c.h>
#include <pac1710.h>
#include <packetReceiver.h>
#include <portExpanderManager.h>
#include <ports.h>
#include <powerOutputs.h>
#include <pwm.h>
#include <sfdqPackets.h>
#include <sfm3019.h>
#include <sigGen.h>
#include <spi.h>
#include <stdbool.h>
#include <stepperMotor.h>
#include <stm32f4xx.h>
#include <stm32f4xx_iwdg.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx.h>
#include <sys/_stdint.h>
#include <system_stm32f4xx.h>
#include <thermistors.h>
#include <vRailMonitor.h>
#include <backupRam.h>
#include <appData.h>
#include <lsm6ds3.h>
#include <eeprom.h>
#include <statusLeds.h>
#include <streamingManager.h>
//#include <queueManager.h>


#include <max31865.h>
#include <amt22.h>
#include <hardwareRev.h>
#include <sinkSource.h>
#include <hardwareEncoder.h>
#include <ads131m0x.h>
#include <clockSetup.h>
#include <crc1021.h>
#include <hscPressureSensor.h>
//#include <ledStripControl.h>


//*************************************************
//defines
//*************************************************
#define APP_DATA_LENGTH 16
#define SFDQ_BAUD 115200
#define UART_BYTE_BUFFER_LENGTH 2000
#define EEPROM_RAM_SIZE 1556
#define STREAM_MANAGER_QUEUE_LENGTH (100)

//Clock config stuff
#define HSE_VALUE_MHZ 8 //change this to 8 for Nucleo
#define CLOCK_FREQ_MHZ 180




//*************************************************
//Types
//*************************************************


//*************************************************
//SFDQ Variables
//*************************************************

static ByteQ hostInQ;	//note: by default a ByteQ is 128 bytes long
static ByteQ hostOutQ;
static uint8_t hostInBuffer[UART_BYTE_BUFFER_LENGTH];
static uint8_t hostOutBuffer[UART_BYTE_BUFFER_LENGTH];

static ByteQ debugInQ;
static ByteQ debugOutQ;
static uint8_t debugInBuffer[UART_BYTE_BUFFER_LENGTH];
static uint8_t debugOutBuffer[UART_BYTE_BUFFER_LENGTH];


static uint32_t wdtTimerReg = 0;

static uint32_t feedbackTimerReg = 0;


static float m_queueData0[STREAM_MANAGER_QUEUE_LENGTH];
static float m_queueData1[STREAM_MANAGER_QUEUE_LENGTH];
static float m_queueData2[STREAM_MANAGER_QUEUE_LENGTH];
static float m_queueData3[STREAM_MANAGER_QUEUE_LENGTH];
static float m_queueData4[STREAM_MANAGER_QUEUE_LENGTH];
static float m_queueData5[STREAM_MANAGER_QUEUE_LENGTH];
static float m_queueData6[STREAM_MANAGER_QUEUE_LENGTH];
static float m_queueData7[STREAM_MANAGER_QUEUE_LENGTH];

//static uint32_t currentSenseTimerReg = 0;
//
//static uint8_t currentSenseAddress[4] = {0x4c, 0x4d, 0x4e, 0x4f};
//static float currentSenseValue[4];
//static float voltageSenseValue[4];
static uint32_t currentSenseIndex = 0;


static uint32_t m_eepromShadowRam[EEPROM_RAM_SIZE];




//*************************************************
//Application Specific Variables
//*************************************************

//this section is only for use when the SFDQ firmware is customized for a specific project
static bool commOverride = false;//used to make the SFDQ comm protocol over-ride the custom code
static uint32_t commOverrideTimerReg = 0;
static uint32_t commTimeoutTimerReg = 0;

static float m_txAppData[APP_DATA_LENGTH];//data to send to the host
static float m_rxAppData[APP_DATA_LENGTH];//data sent to us from host

//static uint32_t cat24TimerReg = 0;
//static uint32_t cat24DataReg = 0;

static uint32_t m_hardwareRev;

//static uint8_t m_ledStripBytes[LED_STRIP_BYTES_FROM_COUNT_RGB(60)];


//*************************************************
//function prototypes
//*************************************************

static void print(char* msg);


/**
 * This is the specific code to the TNC project.
 * It will only run when comms is not present.
 */
static void doAppSpecificCode(void);





//*************************************************
//code
//*************************************************


int main(void){


	// DO THIS FIRST
	//check if WDT reset occurred

//	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET){
//		//a wdt reset occurred. Log it
//		logError(WATCHDOG_RESET, 0);
//		RCC_ClearFlag();
//
//	}
	// DO THIS SECOND
	// (ST library initialization)
	// System / clock setup (ST library calls)
	initSystem();

	setupSystemClock(CLOCK_FREQ_MHZ);
 	systemCoreClockUpdate();
 	enablePeripheralClocks();




	initPins();

	//init new byte queues.
	initByteQ(&hostInQ, hostInBuffer, UART_BYTE_BUFFER_LENGTH);
	initByteQ(&hostOutQ, hostOutBuffer, UART_BYTE_BUFFER_LENGTH);
	initByteQ(&debugInQ, debugInBuffer, UART_BYTE_BUFFER_LENGTH);
	initByteQ(&debugOutQ, debugOutBuffer, UART_BYTE_BUFFER_LENGTH);

	//use these next two lines for the old wiring of uarts.
	//This is NOT compatible with using the bootloader



	m_hardwareRev = getHardwareRev();

//	m_hardwareRev = HARDWARE_REV_X5;

//	m_hardwareRev = HARDWARE_REV_NUCLEO;//TODO:force it for now

	switch(m_hardwareRev){
	case HARDWARE_REV_X7:
	case HARDWARE_REV_X6:
	case HARDWARE_REV_X5:
		initComms(USART3_DEV,  &debugInQ, &debugOutQ, 115200, false);//DMA collision with SPI2!!!
		initComms(USART1_DEV, &hostInQ, &hostOutQ, SFDQ_BAUD, true);
		initBaudrateUpdater(USART1_DEV);

		powerOutputInit(0, PWM_TIMER8, PWM_CH1);
		powerOutputInit(1, PWM_TIMER8, PWM_CH2);
		powerOutputInit(2, PWM_TIMER8, PWM_CH3);
		powerOutputInit(3, PWM_TIMER8, PWM_CH4);
		setPinRawAf(GPIO_C6_PIN, GPIO_AF_TIM8);
		setPinRawAf(GPIO_C7_PIN, GPIO_AF_TIM8);
		setPinRawAf(GPIO_C8_PIN, GPIO_AF_TIM8);
		setPinRawAf(GPIO_C9_PIN, GPIO_AF_TIM8);
		break;
	case HARDWARE_REV_X3:
	case HARDWARE_REV_X2:
		initComms(USART1_DEV, &debugInQ, &debugOutQ, 115200, true);
		initComms(USART6_DEV, &hostInQ, &hostOutQ, SFDQ_BAUD, true);
		initBaudrateUpdater(USART6_DEV);

		powerOutputInit(0, PWM_TIMER3, PWM_CH4);
		powerOutputInit(1, PWM_TIMER3, PWM_CH1);
		powerOutputInit(2, PWM_TIMER2, PWM_CH4);
		powerOutputInit(3, PWM_TIMER3, PWM_CH3);


		setPinDirection(GPIO_C8_PIN, PIN_AF1);
		setPinDirection(GPIO_B4_PIN, PIN_AF1);
		setPinDirection(GPIO_B1_PIN, PIN_AF1);
		setPinDirection(GPIO_B2_PIN, PIN_AF1);

		setPinDirection(GPIO_C6_PIN, PIN_AF2);//usart6
		setPinDirection(GPIO_C7_PIN, PIN_AF2);//usart6
		setPinDirection(GPIO_A6_PIN, PIN_OUT);//led100
		setPinDirection(GPIO_A7_PIN, PIN_OUT);//led101
		//these enable the PWM outputs
		setPin(GPIO_C9_PIN, true);	//PWM0
		setPin(GPIO_C14_PIN, true); //PWM1
		setPin(GPIO_C15_PIN, true); //PWM2
		setPin(GPIO_C13_PIN, true); //PWM3
		break;
	case HARDWARE_REV_NUCLEO:
		initComms(USART3_DEV,  &debugInQ, &debugOutQ, 115200, false);//DMA collision with SPI2!!!
		initComms(USART1_DEV, &hostInQ, &hostOutQ, 115200, true); //SFDQ_BAUD, true);
		initBaudrateUpdater(USART1_DEV);

//		setPinDirection(GPIO_A2_PIN, PIN_AF2);
//		setPinDirection(GPIO_A3_PIN, PIN_AF2);
//		initComms(USART2_DEV, &hostInQ, &hostOutQ, 115200, true); //SFDQ_BAUD, true);
//		initBaudrateUpdater(USART2_DEV);

		break;
	default:
		if(getRoundedHse() == 8.0e6f){
			initComms(USART3_DEV,  &debugInQ, &debugOutQ, 115200, false);//DMA collision with SPI2!!!
			initComms(USART1_DEV, &hostInQ, &hostOutQ, SFDQ_BAUD, true);
			initBaudrateUpdater(USART1_DEV);
		} else {
			//note both uarts recieve into the same queue because different hardware has different uarts mapped to comms
			//TODO: this likely does not work
			initComms(USART1_DEV, &hostInQ, &hostOutQ, SFDQ_BAUD, true);
//			initComms(USART6_DEV, &hostInQ, &debugOutQ, SFDQ_BAUD, true);
		}
		break;


	}



	initPacketReciever(&hostInQ);



	initThermistors();
	initSteppers();
	initFeedback();




	spiInit();
	dacInit();

	if(SET == RCC_GetFlagStatus(RCC_FLAG_IWDGRST)){
		logError(WATCHDOG_RESET_ERROR, 0);
	} else {
		logError(RESET_ERROR, 0);
	}





	sigGenInit();
	i2cInit(I2C1_DEV);
	i2cInit(I2C2_DEV);
	i2cInit(I2C3_DEV);
	initialiseFastCode(20000.0f);
	pac1710Init(I2C1_DEV);

	initEncoder();
	vRailMonitorInit();
	adcManagerInit();

	volatile uint32_t i = 0; //Slow code
//	logError(TEST_ERROR);



//	amt22Config(1, SPI2_DEV, GPIO_B10_PIN, true, 42, true, 1, 0);



	//setup I2C pins
	setPinDirection(GPIO_B9_PIN, PIN_AF1);
	setPinDirection(GPIO_C12_PIN, PIN_AF1);

	initStreaming();
	setupStreamQueue(0, m_queueData0, STREAM_MANAGER_QUEUE_LENGTH);//this is where we config the queues
	setupStreamQueue(1, m_queueData1, STREAM_MANAGER_QUEUE_LENGTH);//this is where we config the queues
	setupStreamQueue(2, m_queueData2, STREAM_MANAGER_QUEUE_LENGTH);//this is where we config the queues
	setupStreamQueue(3, m_queueData3, STREAM_MANAGER_QUEUE_LENGTH);//this is where we config the queues
	setupStreamQueue(4, m_queueData4, STREAM_MANAGER_QUEUE_LENGTH);//this is where we config the queues
	setupStreamQueue(5, m_queueData5, STREAM_MANAGER_QUEUE_LENGTH);//this is where we config the queues
	setupStreamQueue(6, m_queueData6, STREAM_MANAGER_QUEUE_LENGTH);//this is where we config the queues
	setupStreamQueue(7, m_queueData7, STREAM_MANAGER_QUEUE_LENGTH);//this is where we config the queues



//	cat24TestData = 0x31415926;
//	queueCat24WriteWords(12, &cat24TestData, &cat24Status);

//	setupEepromAsCat24(m_eepromShadowRam, EEPROM_RAM_SIZE, I2C2_DEV, 0);
	setupEepromAsNothing(EEPROM_RAM_SIZE, m_eepromShadowRam);

	initAppData(m_txAppData, APP_DATA_LENGTH, m_rxAppData, APP_DATA_LENGTH);


	ads131m0xInit();
	hscInit();



	while(true){
		slowCodeTimeCheck();

		switch(m_hardwareRev){
		case HARDWARE_REV_X7:
		case HARDWARE_REV_X6:
			configStatusLeds(GPIO_D2_PIN, GPIO_B2_PIN);
			break;
		case HARDWARE_REV_X5:
			configStatusLeds(GPIO_B10_PIN, GPIO_B2_PIN);
			break;
		case HARDWARE_REV_NUCLEO:
			configStatusLeds(GPIO_A5_PIN, GPIO_A5_PIN);
			setPinDirection(GPIO_A5_PIN, PIN_OUT);
			break;
		default:
			configStatusLeds(GPIO_A6_PIN, GPIO_A7_PIN);
			break;
		}




		++i;
		if(slowTimer(&wdtTimerReg, 1000)){

			IWDG_ReloadCounter();
			//setup app data frequently
			initAppData(m_txAppData, APP_DATA_LENGTH, m_rxAppData, APP_DATA_LENGTH);

		}


		if(slowTimer(&feedbackTimerReg, getFeedbackSampleTimeUs())){
			calcFeedbackOutputs();
		}

		statusLedSlowcode();

		//init dac because there's no harm in it
		dacInit();


		packetReceiverLoop();

		if(decodeSfdqPacket()){

			//there was a new packet

			if(!wasPacketJustAQuery()){
				resetSlowTimer(&commOverrideTimerReg);
				commOverride = true;
			}

//			resetSlowTimer(&commTimeoutTimerReg);
//			resetSlowTimer(&commBlinkTimerReg);
			statusLedPacketReceived();
			sendSfdqPacket(&hostOutQ);


		}

//if no comms for 5 seconds then make sure we're at 115200 baud
		if(slowTimer(&commTimeoutTimerReg, 5000000)){
			if(getBaudrateUpdate() != 115200){
				updateBaudrate(115200);
			}
		}


		if(slowTimer(&commOverrideTimerReg, 1000000)){
			commOverride = false;
		}

		if(!commOverride){
			//if comms is not ongoing then control the system autonomously
			doAppSpecificCode();
		} else {

//			setTxAppData(0, getFastloopMaxTime()*1e6);
//			setTxAppData(1, (float)getTimeInMicroSeconds());
//			setTxAppData(1, getSfm3019Temperature());
//			setTxAppData(2, getSfm3019Volume());
//			setTxAppData(3, getI2cResetTimeoutCount(I2C1_DEV)+getI2cResetTimeoutCount(I2C2_DEV));
//			setTxAppData(4, getI2cBusyTimeoutCount(I2C1_DEV)+getI2cBusyTimeoutCount(I2C2_DEV));
//			setTxAppData(5, getSfm3019Status());
//			setTxAppData(6, M_PI);

		}

//		if(slowTimer(&cat24TimerReg, 100000)){
//			queueCat24ReadWords(I2C2_DEV, 0b10100000, 12, &cat24DataReg, 0);
////			updateTca9534a(I2C1_DEV, 0x39, 0b11010100, 0xff, &tca9534Inputs, &tca9534Status1);
//
//
//
//		}
		portExpanderSlowCode();
//		tca9534SlowCode();
//		cat24C256SlowCode();
		eepromSlowCode();
//		sfm3019SlowCode(I2C1_DEV, 20000, 0.0f, GPIO_A11_PIN);

//		if(ledStripGetWaypointCount() == 0){
//			ledStripSetCycleTime(5.0f);
//			ledStripSetPhasePerLed(0.01f);
//			ledStripWaypointAdd(1.0f, 0.0f, 0.0f, 1.0f, 0.2f);//red
//			ledStripWaypointAdd(1.0f, 0.6f, 0.0f, 1.0f, 0.2f);//orange
//			ledStripWaypointAdd(1.0f, 1.0f, 0.0f, 1.0f, 0.2f);//yellow
//			ledStripWaypointAdd(0.0f, 1.0f, 0.0f, 1.0f, 0.2f);//green
//			ledStripWaypointAdd(0.0f, 0.0f, 1.0f, 1.0f, 0.2f);//blue
//			ledStripWaypointAdd(0.6f, 0.0f, 1.0f, 1.0f, 0.2f);//violet
//			setPinDirection(GPIO_C12_PIN, PIN_AF2);
//		}
//
//
//		ledStripConfig(SPI3_DEV, LED_STRIP_BYTES_FROM_COUNT_RGB(60), m_ledStripBytes, true);
//		ledStripSlowcode();


//		ad7172SlowCode(SPI2_DEV, GPIO_B12_PIN);
		lsm6ds3SlowCode(0.5);
		currentSenseIndex = 0;
		adcManagerSlowCode();

//		float a, v;
//		pac1710SlowCode(0, &m_pac1710Timer, &a, &v);
//		setTxAppData(4, v);
		vRailMonitorSlowCode();
		bootloaderSlowCode();
		hardwareEncoderSlowcode();
//		ads131a0xConfig(true, SPI2_DEV, GPIO_B12_PIN, GPIO_A11_PIN);
//		ads131a0xControl(1000000);
		ads131m0xSlowCode();

//		max31865SlowCode(0.5);
//		amt22SlowCode(0.01);
		streamManagerSlowCode();
		hscSlowcode();



		//these lines setup the hardware encoder
//		setHardwareEncoderDistancePerCount(HW_ENCODER_TIMER2, 1.0f);
//		setupHardwareEncoder(HW_ENCODER_TIMER2, txAppDataSink(1));
//		setHardwareEncoderZeroSink(HW_ENCODER_TIMER2, streamSink(0));
//		configStreaming(0, FB_INPUT_STREAM_QUEUE, 0, 0.001, false);


		//check the current sensor. If we get a reading then increment to the next chip
//		if(pac1710SlowCode(currentSenseAddress[currentSenseIndex], &currentSenseTimerReg, &(currentSenseValue[currentSenseIndex]), &(voltageSenseValue[currentSenseIndex]))){
//			++currentSenseIndex;
//			if(currentSenseIndex >= 4){
//				currentSenseIndex = 0;
//			}
//		}
	}
}

// Debug print
static void print(char* msg){
	if(msg){
		while(*msg){
			putByte(&debugOutQ, *msg);
			msg++;
		}
	}
}

/**
 * This is the specific code to the application project.
 * It will only run when comms is not present.
 */
static void doAppSpecificCode(void){

}

void SysTick_Handler(void){
	dontPutAnyFastCodeBeforeThisFunction();
	fastcode();
	dontPutAnyFastCodeAfterThisFunction();

}



