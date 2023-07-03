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

//*************************************************
//includes
//*************************************************
#include <thermistors.h>
#include <math.h>

#include <sfdqPackets.h>
#include <_processorGlobal.h>
#include <uid.h>
#include <comms.h>
#include <error.h>
#include <timeSync.h>
#include <ports.h>
#include <ad7172.h>
#include <sigGen.h>
#include <lsm6ds3.h>
#include <packetReceiver.h>
#include <packetBuilder.h>
#include <adcManager.h>
#include <temperatureControl.h>
#include <encoder.h>
#include <feedbackControl.h>
#include <brushlessMotor.h>
#include <fastcodeUtil.h>
#include <vRailMonitor.h>
#include <version.h>
#include <dac.h>
#include <bootloader.h>
#include <portExpanderManager.h>
#include <stepperMotor.h>
#include <softPwm.h>
#include <appData.h>
#include <eeprom.h>
#include <streamingManager.h>
#include <hardwareRev.h>
#include <registerConfig.h>


//*************************************************
//Notes
//*************************************************


//*************************************************
//Defines
//*************************************************

//*************************************************
//Types
//*************************************************


typedef enum {
	INPUT_0 = 0,
	INPUT_1 = 1,
	OUTPUT_0 = 2,
	OUTPUT_1 = 3,
	ANALOG = 4,
	ALT_FUNC_1 = 5,
	ALT_FUNC_2 = 6,
	OUTPUT_X = 7,
	NO_EFFECT = 8,
	INPUT_PULLUP_0 = 9,
	INPUT_PULLUP_1 = 10,
} SfdqPinConfig;

//*************************************************
//Variables
//*************************************************

static uint32_t hostTime;
static bool pwmPayloadDone = false;
static uint32_t lastPayloadHeader = 0;
static bool packetOnlyContainedQueries = true;//packetOnlyContainedQueries = true;
static uint32_t baudrateRequest = 0;
static uint32_t baudrateCount = 0;
static bool acceptAllPayloads = false;//if this is set then all payloads are accepted, even if ID is not correct
static uint32_t m_eepromWordIndex = 0;
static uint32_t m_eepromWordCount = 0;
static uint32_t m_streamingChannel = 0;
static Payload* m_registerConfigPayload = NULL;
static Payload* m_registerRequestPayload = NULL;



//*************************************************
//function prototypes
//*************************************************



static uint32_t makePinConfigNum(PortPin pinNum);



//*************************************************
//Code
//*************************************************




bool decodeSfdqPacket(void){
	bool result = false;

	pwmPayloadDone = false;
	bool onlyQueries = true;

	//now decode packets
	Packet* packet = checkForPacket();
	if(packet != 0){
		result = true;




		//process each payload of the new packet
		bool ignore = false;

		Payload* payload = 0;

		uint32_t i = 0;
		while((payload = getNextPayload(packet, &i)) != 0){
			PayloadType pt = getPayloadType(payload);
			lastPayloadHeader = getPayloadWord(payload, 0);
			uint32_t pLen = getPayloadLength(payload);
			if(ignore && pt != ID_PAYLOAD){
				//don't process packet because it's not for us
			} else {
				PortPin pin;
				uint32_t i;
				switch(pt){

				case ID_PAYLOAD:
					if(pLen > 0){//only interpret if there's data.
						uint32_t id = getPayloadWord(payload, 1);

						if(acceptAllPayloads){
							ignore = false;
						} else if(id != readUid32()){
							ignore = true;
						} else {
							ignore = false;
						}
					}
//					if(!ignore){
//						makePayload(ID_PAYLOAD);
//					}
					break;
				case IMU_RESPONSE_PAYLOAD:
					makePayload(IMU_RESPONSE_PAYLOAD);
					break;
				case TIME_PAYLOAD:
					;//an empty statement to prevent a silly error

					hostTime = getPayloadWord(payload, 1);
					uint32_t packetTime = getTimeOfLastPacket();
					updateTimeSync(hostTime, packetTime);
					//setup the response payload
					makePayload(TIME_PAYLOAD);


					break;
				case PORT_EXPANDER_PAYLOAD:
					i = 1;
					uint32_t k = 0;
					while(i <= pLen){
						uint32_t pe = getPayloadWord(payload,i);
						++i;
						I2cDev bus = I2CNULL_DEV;
						switch(pe & (0b11 << 30)){
						default:
						case (0 << 30):
							bus = I2CNULL_DEV;
							break;
						case (1 << 30):
							bus = I2C1_DEV;
							break;
						case (2 << 30):
							bus = I2C2_DEV;
							break;
						case (3 << 30):
							bus = I2C3_DEV;
							break;
						}
						IcType type = TCA9534;
						switch(pe & (0b111 << 27)){
						case (0 << 27):
							type = TCA9534;
							break;
						default:
							break;
						}
						//I think this should be faster than 24 right shifts
						uint32_t address = 0;
						if(pe & 0b1 << 26){
							address |= 0b100;
						}
						if(pe & 0b1 << 25){
							address |= 0b10;
						}
						if(pe & 0b1 << 24){
							address |= 1;
						}
						setupPortExpander(k, bus, address, type);

						PinConfig pc = INPUT_LO;
						uint8_t mask = 0b1;
						for(uint32_t j = 0; j < 8; ++j){
							uint32_t pem = pe & 0b111;
							switch(pem){
							default:
							case 0:
								pc = INPUT_LO;
								break;
							case 1:
								pc = INPUT_HI;
								break;
							case 2:
								pc = OUTPUT_CLEAR;
								break;
							case 3:
								pc = OUTPUT_SET;
								break;
							case 4:
								pc = OUTPUT_NO_CHANGE;
								break;

							}
							setupPortExpanderPin(k, mask, pc);

							mask <<= 1;
							pe >>= 3;
						}
						++k;

					}
					//this will ensure that any channels beyond what was just configured will be disabled
					setNumChannels(k);
					makePayload(PORT_EXPANDER_PAYLOAD);
					break;
				case EEPROM_WRITE_REQUEST_PAYLOAD:
					;//an empty statement to prevent a silly error

					uint32_t addrNum = getPayloadWord(payload, 1);
					m_eepromWordIndex = addrNum >> 16;
					m_eepromWordCount = addrNum & 0x00ff;//it can't be more than 256

					writeEepromValues(m_eepromWordIndex, m_eepromWordCount, payload + 2);//passing payload as a pointer is ok because the variables will have been copied during this function call
					makePayload(EEPROM_RESPONSE_PAYLOAD);
//					makePayload(ACK_PAYLOAD);
					break;
				case EEPROM_READ_REQUEST_PAYLOAD:
					;//an empty statement to prevent a silly error

					addrNum = getPayloadWord(payload, 1);
					m_eepromWordIndex = addrNum >> 16;
					m_eepromWordCount = addrNum & 0x00ff;//it can't be more than 256
					makePayload(EEPROM_RESPONSE_PAYLOAD);
					break;
				case IO_CONTROL_PAYLOAD:
					;//an empty statement to prevent a silly error


//					packetOnlyContainedQueries = false;//indicate that this payload is not just a query
//
//					//now set outputs
//					uint32_t ioOut = getPayloadWord(p, 1);
//					uint32_t dirMask = getPayloadWord(p, 2);
//					uint32_t adcMask = getPayloadWord(p, 3);//this is adc mode
//					uint32_t dacValue = getPayloadWord(p,4);
//					uint16_t dac1 = (uint16_t)((dacValue >> 0) & 0xffff);
//					uint16_t dac2 = (uint16_t)((dacValue >> 16) & 0xffff);
//					setDac1Value(dac1);
//					setDac2Value(dac2);
//
//					setGpioDirMask((uint64_t)dirMask, (uint64_t)ioOut, (uint64_t)adcMask);
//
//					setAdcChannelMask(adcMask);
//
//					//setup the response payload
//
//					uint32_t i = 5;
//					uint32_t j = 0;
//					uint32_t mask = adcMask;
//					while(mask != 0 && i <= pLen){
//						if(mask & 0b1){
//							float k = getPayloadFloat(p,i);
//							setAdcTimeConstant(j, k);
//							++i;
//						}
//						mask >>= 1;
//
//						++j;
//					}
//
//					makePayload(IO_RESPONSE_PAYLOAD);

					break;
				case GPIO_CONTROL_PAYLOAD:
					;//an empty statement to prevent a silly error


					onlyQueries = false;//indicate that this payload is not just a query


					pin = GPIO_A0_PIN;
					uint32_t pinConfig = 0;
					for(uint32_t i = 1; i <= 6; ++i){
						if(i > pLen){
							break;
						}

						uint32_t pc = getPayloadWord(payload, i);
						for(uint32_t j = 0; j < 8; ++j){

//							if(pin == GPIO_A11_PIN){
//								setPin(GPIO_A11_PIN, true);
//								asm("nop");
//							} else {
//							}
							pinConfig = pc & 0b1111;



							if(isPinChangeAllowed(pin)){


								switch((SfdqPinConfig)pinConfig){
								default:
								case NO_EFFECT:
									break;
								case INPUT_0:
									setPinDirection(pin,PIN_IN);
									setPinPullup(pin, PULL_NONE);
									break;
								case INPUT_1:
									setPinDirection(pin,PIN_IN);
									setPinPullup(pin, PULL_NONE);
									break;
								case OUTPUT_0:
									setPinDirection(pin,PIN_OUT_LO);
									setPin(pin, false);
									break;
								case OUTPUT_1:
									setPinDirection(pin,PIN_OUT_HI);
									setPin(pin, true);
									break;
								case ANALOG:
									setPinDirection(pin,PIN_AN);
									break;
								case ALT_FUNC_1:
									setPinDirection(pin,PIN_AF1);
									break;
								case ALT_FUNC_2:
									setPinDirection(pin,PIN_AF2);
									break;
								case OUTPUT_X:
									setPinDirection(pin,PIN_OUT);
									break;
								case INPUT_PULLUP_0:
									setPinDirection(pin,PIN_IN);
									setPinPullup(pin, PULL_UP);

									break;
								case INPUT_PULLUP_1:
									setPinDirection(pin,PIN_IN);
									setPinPullup(pin, PULL_UP);
									break;
								}

							}

							pc >>= 4;
							++pin;
						}
					}





					makePayload(GPIO_RESPONSE_PAYLOAD);

					break;
				case GPIO_REQUEST_PAYLOAD:
					makePayload(GPIO_RESPONSE_PAYLOAD);
					makePayload(ANALOG_RESPONSE_PAYLOAD);

					break;
				case ANALOG_CONTROL_PAYLOAD:
					;//an empty statement to prevent a silly error
					i = 1;
					if(pLen > i){
						onlyQueries = false;//indicate that this payload is not just a query

						uint32_t adcEnable = getPayloadWord(payload,i);

						setAdcChannelMask(adcEnable & 0xffff);
						setDacEnable(0, (adcEnable & (1<<16)) != 0);
						setDacEnable(1, (adcEnable & (1<<17)) != 0);
						bool dac0ManualEn = (adcEnable & (1<<18)) != 0;
						bool dac1ManualEn = (adcEnable & (1<<19)) != 0;
						++i;
						uint32_t dacValue = getPayloadWord(payload,i);
						++i;
						uint16_t dac1 = (uint16_t)((dacValue >> 0) & 0xffff);
						uint16_t dac2 = (uint16_t)((dacValue >> 16) & 0xffff);
						if(dac0ManualEn){
							setDac1Value(dac1);
						}
						if(dac1ManualEn){
							setDac2Value(dac2);
						}
						uint32_t mask = getAdcChannelMask();


						uint32_t j = 0;
						while(i <= pLen){
							if(mask & 0b1){
								float tc = getPayloadFloat(payload,i);
								++i;
								float scale = getPayloadFloat(payload, i);
								++i;
								float offset = getPayloadFloat(payload, i);
								++i;

								setAdcParams(j, tc, scale, offset);
							}
							mask >>= 1;
							++j;
						}
					}
					makePayload(ANALOG_RESPONSE_PAYLOAD);
					break;
				case IO_REQUEST_PAYLOAD:


					//setup the response payload
					makePayload(IO_RESPONSE_PAYLOAD);


					break;
				case APP_DATA_PAYLOAD://these two cases are treated the same
				case APP_DATA_REQUEST_PAYLOAD:
					i = 1;
					while(i <= pLen){
						float d = getPayloadFloat(payload, i);

						setRxAppData(i - 1, d);


						++i;
					}
					makePayload(APP_DATA_PAYLOAD);
					break;
				case STEPPER_CONFIG_PAYLOAD:
					;//an empty statement to prevent a silly error
					onlyQueries = false;//indicate that this payload is not just a query
					i = 1;

					while(i <= pLen){
						uint32_t it = getPayloadWord(payload, i);
						uint32_t stepperIndex = it >> 24;
						StepperType stepperType = (StepperType)((it >> 16) & 0xff);
						uint16_t stepperConfig = it & 0xffff;


						++i;
						uint32_t pins = getPayloadWord(payload, i);

						PortPin stepperOut1Pin = ((pins >> 8) & 0x7f) + GPIO_A0_PIN;
						PortPin stepperOut2Pin = ((pins >> 0) & 0x7f) + GPIO_A0_PIN;
						++i;
						float stepperDistancePerStep = getPayloadFloat(payload, i);
						++i;


						float stepperPwmFraction = getPayloadFloat(payload, i);
						++i;
						if(stepperType == STEPPER_PWM){
							setSigGenType(0, STEPPER_SIG);
							setSigGenType(1, STEPPER_SIG);
							setSigGenType(2, STEPPER_SIG);
							setSigGenType(3, STEPPER_SIG);
						}
						setStepperOnIdle(stepperIndex, (StepperOnIdle)(stepperConfig & 0b11));
						setStepperZeroing(stepperIndex, (stepperConfig & 0b100) != 0);
						configStepper(stepperIndex, stepperType, stepperDistancePerStep, stepperPwmFraction);
						configStepperOutputPins(stepperIndex, stepperOut1Pin, stepperOut2Pin);

					}

					makePayload(STEPPER_RESPONSE_PAYLOAD);

					break;
				case STEPPER_CONTROL_PAYLOAD:


					break;
				case FIRMWARE_REQUEST_PAYLOAD:
					if(pLen >= 1){
						setHardwareRev(getPayloadWord(payload, 1));
						NVIC_SystemReset(); //TODO: Consider doing this to reboot after setting the hardware rev
					}
					makePayload(FIRMWARE_RESPONSE_PAYLOAD);


					break;
				case FIRMWARE_PROGRAM_PAYLOAD:
					triggerBootloader();
					makePayload(FIRMWARE_PROGRAM_ACK_PAYLOAD);

					break;
				case BAUDRATE_PAYLOAD:
					;//an empty statement to prevent a silly error
					uint32_t br = getPayloadWord(payload, 1);
					if(br == baudrateRequest){

						++baudrateCount;
						if(baudrateCount >= 4){
							updateBaudrate(br);
						}
					} else {
						baudrateCount = 0;
					}
					baudrateRequest = br;
					makePayload(BAUDRATE_PAYLOAD);
					break;
				case ERROR_ACK_PAYLOAD:
					;//an empty statement to prevent a silly error

					uint32_t e = getCurrentError();
					if(e != 0 && e == getPayloadWord(payload, 1)){
						clearCurrentError();
					}
					break;
				case THERMAL_CONTROL_PAYLOAD:
					;//an empty statement to prevent a silly error
					onlyQueries = false;//indicate that this payload is not just a query
					float setpointT = getPayloadFloat(payload, 1);
					uint32_t channels = getPayloadWord(payload, 2);
					uint32_t outChan = (channels >> 24) & 0xff;
					uint32_t adcChan = (channels >> 16) & 0xff;
					float pCoeff = getPayloadFloat(payload, 3);
					float iCoeff = getPayloadFloat(payload, 4);
					float dCoeff = getPayloadFloat(payload, 5);
					float maxPwm = getPayloadFloat(payload, 6);

					sigGenTemperatureControl(outChan, adcChan, setpointT, pCoeff, iCoeff, dCoeff, maxPwm);

					makePayload(THERMAL_RESPONSE_PAYLOAD);
					break;
				case THERMAL_REQUEST_PAYLOAD:
					makePayload(THERMAL_RESPONSE_PAYLOAD);
					break;
				case PWM_CONTROL_PAYLOAD:


					;//an empty statement to prevent a silly error

					if(pLen > 1){//if this payload has no data then it's just a request for a payload response
						onlyQueries = false;//indicate that this payload is not just a query

						uint32_t c = getPayloadWord(payload, 1);
						SigType st = lookupType(c & 0xff);
						c >>= 8;
						c &= 0b11;
						float pwm = getPayloadFloat(payload, 2);
						float freq = getPayloadFloat(payload, 3);



						sigGenConfig(c, st, freq, pwm);
					}
					if(!pwmPayloadDone){
						makePayload(PWM_RESPONSE_PAYLOAD);
						pwmPayloadDone = true;
					}


					break;

				case HADC_CONTROL_PAYLOAD:
					;//an empty statement to prevent a silly error


					i = 1;
					while(i <= pLen){
						onlyQueries = false;//indicate that this payload is not just a query
						uint32_t w1 = getPayloadWord(payload, i);
						++i;
						bool adcEn = (w1 >> 24) != 0;
						uint32_t adcChan = (w1 >> 16) & 0b11;
						uint32_t adcPosInputType = (w1 >> 8) & 0b1111;
						uint32_t adcNegInputType = (w1 >> 0) & 0b1111;
						ad7172Config(adcChan, adcPosInputType, adcNegInputType, adcEn);

					}


					makePayload(HADC_RESPONSE_PAYLOAD);
					break;
				case HADC_REQUEST_PAYLOAD:
					makePayload(HADC_RESPONSE_PAYLOAD);
					break;


				case TEMPERATURE_CONFIG_PAYLOAD:
					;//an empty statement to prevent a silly error


					i = 1;
					while(i <= pLen){
						onlyQueries = false;//indicate that this payload is not just a query
						uint32_t w1 = getPayloadWord(payload, i);
						++i;
						uint32_t index = (w1 >> 24) & 0xff;
						uint32_t thermistorChannel = (w1 >> 16) & 0xff;

						float c0 = getPayloadFloat(payload, i);
						++i;
						float c1 = getPayloadFloat(payload, i);
						++i;
						float c2 = getPayloadFloat(payload, i);
						++i;
						float c3 = getPayloadFloat(payload, i);
						++i;
						setupThermistor(index, thermistorChannel, c0, c1, c2, c3);

					}
					makePayload(TEMPERATURE_RESPONSE_PAYLOAD);
					break;

				case TEMPERATURE_REQUEST_PAYLOAD:
					makePayload(TEMPERATURE_RESPONSE_PAYLOAD);
					break;
				case ENCODER_CONFIG_PAYLOAD:
					;//an empty statement to prevent a silly error


					i = 1;
					while(i <= pLen){
						onlyQueries = false;//indicate that this payload is not just a query
						uint32_t w1 = getPayloadWord(payload, i);
						++i;
						uint32_t index = (w1 >> 24) & 0xff;
						PortPin channelA = ((w1 >> 16) & 0xff) + GPIO_A0_PIN;
						PortPin channelB = ((w1 >> 8) & 0xff) + GPIO_A0_PIN;

						float scale = getPayloadFloat(payload, i);
						++i;
						setupEncoder(index, channelA, channelB, NULL_PIN, scale, 0);//TODO: debouncing is essentially disabled here
					}
					makePayload(ENCODER_RESPONSE_PAYLOAD);
					break;
				case FEEDBACK_CONFIG_PAYLOAD:
					;//an empty statement to prevent a silly error


					i = 1;
					while(i <= pLen){
						onlyQueries = false;//indicate that this payload is not just a query
						uint32_t w1 = getPayloadWord(payload, i);
						++i;
						uint32_t channel = (w1 >> 28) & 0xf;
						FeedbackInputType inputType = (w1 >> 24) & 0xf;
						uint32_t inputIndex = (w1 >> 16) & 0xff;
						FeedbackOutputType outputType = (w1 >> 8) & 0xff;
						uint32_t outputIndex = (w1 >> 0) & 0xff;

						float c0 = getPayloadFloat(payload, i);
						++i;
						float c1 = getPayloadFloat(payload, i);
						++i;
						float c2 = getPayloadFloat(payload, i);
						++i;
						float d1 = getPayloadFloat(payload, i);
						++i;
						float d2 = getPayloadFloat(payload, i);
						++i;
						float minOut = getPayloadFloat(payload, i);
						++i;
						float maxOut = getPayloadFloat(payload, i);
						++i;
						float minIn = getPayloadFloat(payload, i);
						++i;
						float maxIn = getPayloadFloat(payload, i);
						++i;
						uint32_t pins = getPayloadWord(payload, i);
						++i;
						PortPin minLimitPin = ((pins >> 8) & 0xff) + GPIO_A0_PIN;
						if(minLimitPin != NULL_PIN){
							minLimitPin &= 0x7f;
						}
						bool minLimitPinInvert = (pins & (0x80 << 8)) != 0;
						PortPin maxLimitPin = ((pins >> 0) & 0xff) + GPIO_A0_PIN;
						if(maxLimitPin != NULL_PIN){
							maxLimitPin &= 0x7f;
						}
						bool maxLimitPinInvert = (pins & (0x80 << 0)) != 0;
						//now check config bits
						uint8_t configBits = (pins >> 16) & 0xff;

						bool passThrough = (configBits & 0b1) != 0;
						uint8_t unitsVal = ((configBits>>1) & 0b11);

						FeedbackCircularUnits units;
						switch(unitsVal){
						default:
						case 0b00:
							units = FB_LINEAR;
							break;
						case 0b01:
							units = FB_DEGREES;
							break;
						case 0b10:
							units = FB_RADIANS;
							break;
						}


						setupFeedback(channel, inputType, inputIndex, outputType, outputIndex, minOut, maxOut, minIn, maxIn);
						setFeedbackBiquad(channel,  c0, c1, c2, d1, d2);
						configFeedbackTravelLimitPins(channel, minLimitPin, minLimitPinInvert, maxLimitPin, maxLimitPinInvert);
						setFeedbackPassThrough(channel, passThrough);
						setFeedbackUnits(channel, units);
						//ensure output is set too
						if(outputType == FB_OUTPUT_HI_POWER_UNIPOLAR || outputType == FB_OUTPUT_HI_POWER_BIPOLAR){
							setSigGenType(outputIndex, FEEDBACK_SIG);

						}
					}
					makePayload(FEEDBACK_RESPONSE_PAYLOAD);
					//send traj payload if necessary
					if(anyTrajectoriesConfigured()){
						makePayload(TRAJECTORY_RESPONSE_PAYLOAD)	;
					}

					break;

				case BRUSHLESS_MOTOR_CONFIG_PAYLOAD:
					;//an empty statement to prevent a silly error
					onlyQueries = false;//indicate that this payload is not just a query

					uint32_t w1 = getPayloadWord(payload, 1);
					PortPin sensorA = ((w1 >> 24) & 0xff) + GPIO_A0_PIN;
					PortPin sensorB = ((w1 >> 16) & 0xff) + GPIO_A0_PIN;
					PortPin sensorC = ((w1 >> 8) & 0xff) + GPIO_A0_PIN;
					//uint8_t phasesPerRev = (w1 >> 0) & 0xff;
//					float voltage = getPayloadFloat(p, 2);
					float phaseCyclesPerRev = getPayloadFloat(payload, 2);
					float phaseOffset = getPayloadFloat(payload, 3);
					float phaseAdvance = getPayloadFloat(payload, 4);
					setupBrushlessMotor(sensorA, sensorB, sensorC, phaseCyclesPerRev);
					setBrushlessPhaseOffset(phaseOffset, phaseAdvance);
					setSigGenType(0, BRUSHLESS_MOTOR_SIG);
					setSigGenType(1, BRUSHLESS_MOTOR_SIG);
					setSigGenType(2, BRUSHLESS_MOTOR_SIG);


					makePayload(BRUSHLESS_RESPONSE_PAYLOAD);
					break;
				case TRAJECTORY_PAYLOAD:
					;//an empty statement to prevent a silly error


					uint32_t chan;
					float pos = NAN;
					float speed = NAN;
					uint32_t time = 0;
//					bool didAtLeastOne = false;
					i = 1;
					while(i <= pLen){
						onlyQueries = false;//indicate that this payload is not just a query
						chan = getPayloadWord(payload, i);
						++i;
						pos = getPayloadFloat(payload, i);
						++i;
						speed = getPayloadFloat(payload, i);
						++i;
						time = getPayloadWord(payload, i);
						++i;
						trajectoryAddWaypoint(chan, pos, speed, time);
//						didAtLeastOne = true;

					}
//					//will see if I need to send a traj response elsewhere
//					if(didAtLeastOne){
//						makePayload(TRAJECTORY_RESPONSE);
//					}
					break;
				case SOFT_PWM_CONFIG_PAYLOAD:
					i = 1;
					while(i <= pLen){
						uint32_t w1 = getPayloadWord(payload, i);
						++i;
						float f2 = getPayloadFloat(payload, i);
						++i;

						uint32_t channel = (w1 >> 28) & 0xf;
						uint32_t type = (w1 >> 24) & 0xf;
						uint32_t param = (w1 >> 20) & 0xf;
						uint32_t outPin = (w1 >> 8) & 0xff;
						uint32_t triggerPin = (w1 >> 0) & 0xff;
						bool invertTrig = (triggerPin & 0x80) != 0;
						triggerPin &= 0x7f;

						configSoftPwm(channel, (PortPin)outPin);

						switch(type){
						case 0://unconfigured
							deconfigureSoftPwm(channel);
							break;
						case 1://pwm
							setupSoftPwmAsPwm(channel, f2);
							break;
						case 2://one shot
							setupSoftPwmAsOneShot(channel, f2, (PortPin)triggerPin, invertTrig);
							break;
						case 3://follower
							setupSoftPwmAsFollower(channel, param, f2);
							break;

						}
					}
					makePayload(SOFT_PWM_RESPONSE_PAYLOAD);
					break;

				case PERIPHERAL_CONFIG_PAYLOAD:
					break;
				case STREAMING_CONFIG_PAYLOAD:{

						uint32_t w1 = getPayloadWord(payload, 1);
						float period = getPayloadFloat(payload, 2);
						m_streamingChannel = w1 >> 24;
						uint32_t inputIndex = (w1 >> 16) & 0xff;
						FeedbackInputType type = (w1 >> 8) & 0xff;
						if(type != FB_INPUT_QUERY){

							configStreaming(m_streamingChannel, type, inputIndex, period, true);//TODO: probably should use microseconds
						}
						//don't bother making a payload if there's no data
						if(isNextStreamSampleAvailable(m_streamingChannel)){
							makePayload(STREAMING_RESPONSE_PAYLOAD);
						}
					}
					break;

				case REGISTER_CONFIG_PAYLOAD:{
						//add new braces so I can define more variables in their own scope


						i = 1;
						bool sendResponse = false;
						while(i <= pLen){
							sendResponse = true;
							uint32_t id = getPayloadWord(payload, i);
							ModuleId mid = (id >> 24);
							uint32_t rid = (id >> 16) & 0xff;
							uint32_t ch = (id >> 8) & 0xff;
//							uint32_t rnum = (id) & 0xff;
							++i;
							uint32_t wn = getPayloadWord(payload, i);
							++i;
							configRegister(mid, rid, ch, wn);



						}

						if(sendResponse){
							m_registerConfigPayload = payload;
							makePayload(REGISTER_RESPONSE_PAYLOAD);
						}

					}
					break;

				//the following payloads will not be interpreted by a SFDQ
				case SOFT_PWM_RESPONSE_PAYLOAD:
				case TEMPERATURE_RESPONSE_PAYLOAD:
				case ENCODER_RESPONSE_PAYLOAD:
				case FIRMWARE_RESPONSE_PAYLOAD:
				case FIRMWARE_PROGRAM_ACK_PAYLOAD:
				case ERROR_PAYLOAD:
				case ACK_PAYLOAD:
				case NACK_PAYLOAD:
				case IO_RESPONSE_PAYLOAD:
				case STEPPER_RESPONSE_PAYLOAD:
				case PWM_RESPONSE_PAYLOAD:
				case THERMAL_RESPONSE_PAYLOAD:
				case HADC_RESPONSE_PAYLOAD:
				case TRAJECTORY_RESPONSE_PAYLOAD:
				case GPIO_RESPONSE_PAYLOAD:
				case ANALOG_RESPONSE_PAYLOAD:
				case REGISTER_RESPONSE_PAYLOAD:
				default:
					break;
				}
			}
		}

		packetOnlyContainedQueries = onlyQueries;

		finishedWithPacket();

	}

	return result;
}




bool isQueueEmpty(ByteQ* txQ){
	return (getUsed(txQ) == 0);
}
/**
 * Send the currently completely packet down the specified byte queue
 * This will quietly refuse if the queue is not empty
 */
void sendSfdqPacket(ByteQ* txQ){
	if(isQueueEmpty(txQ)){

		completeTxPacket();
		sendTxPacket(txQ);
	} else {
		cancelPacket();
	}
}
/**
 * indicates that a tx packet is currently being built
 */
bool isTxPacketInProgress(void){
	return isPacketStarted();
}

bool wasPacketJustAQuery(void){
	return packetOnlyContainedQueries;
}

/**
 * start a packet and include this sfdq's id
 */
void startPacketIfNecessary(bool includeId){
	if(!isPacketStarted()){
		//setup the builder for a response packet
		startTxPacket();
		if(includeId){
			//send a id packet
			makePayload(ID_PAYLOAD);

		}

		if(getCurrentError() != 0){
			makePayload(ERROR_PAYLOAD);
		}


	}
}
void makePayload(PayloadType type){
	startPacketIfNecessary(true);
	startTxPayload(type);
	switch(type){
	case ID_PAYLOAD:
		addTxPayloadData(readUid32());
		break;
	case TIME_PAYLOAD:
		addTxPayloadData(hostTime);
		addTxPayloadData(getTimeOfLastPacket());
		break;
	case FIRMWARE_RESPONSE_PAYLOAD:
		addTxPayloadData(getSfdqVersion());
		addTxPayloadData(getAppEdmsId());
		addTxPayloadData(getAppVersion());
		addTxPayloadData(getHardwareRev());
		break;
	case BAUDRATE_PAYLOAD:
		addTxPayloadData(getBaudrateUpdate());
		break;
	case FIRMWARE_PROGRAM_ACK_PAYLOAD:
		addTxPayloadData(isBootloaderTriggered() ? 1 : 0);
		break;
	case ERROR_PAYLOAD:
		addTxPayloadData(getCurrentError());
		break;
	case ACK_PAYLOAD:
		addTxPayloadData(lastPayloadHeader);
		break;
	case NACK_PAYLOAD:
		addTxPayloadData(lastPayloadHeader);
		break;
	case IO_RESPONSE_PAYLOAD:
//		addTxPayloadData((uint32_t)getGpioInputs());//ioIn
//		addTxPayloadData((uint32_t)getGpioDirMask());
//		addTxPayloadData((uint32_t)getGpioAdcMask());
//		for(uint32_t i = 0; i < getAdcCount(); ++i){
//			addTxPayloadFloat(getAdcValue(i));
//		}
		break;
	case PORT_EXPANDER_PAYLOAD:
		for(uint32_t i = 0; i < getPortExpanderCount(); ++i){
			if(!isPortExpanderEnabled(i)){
				break;
			} else {
				uint32_t address = getPortExpanderAddress(i);
				uint32_t bus;
				switch(getPortExpanderBus(i)){
				default:
				case I2CNULL_DEV:
					bus = 0 << 6;
					break;
				case I2C1_DEV:
					bus = 1 << 6;
					break;
				case I2C2_DEV:
					bus = 2 << 6;
					break;
				case I2C3_DEV:
					bus = 3 << 6;
					break;
				}
				uint32_t type;
				switch(getPortExpanderType(i)){
				default:
				case TCA9534:
					type = 0 << 3;
					break;
				}


				uint32_t pew = 0;
				pew |= bus;
				pew |= type;
				pew |= address & 0b111;
				uint8_t mask = 0b10000000;
				for(uint32_t j = 0; j < 8; ++j){
					PinConfig config = getPortPinConfig(i, mask);
					mask >>= 1;
					pew <<= 3;
					switch(config){
					case INPUT_LO:
						pew |= 0b000;
						break;
					case INPUT_HI:
						pew |= 0b001;
						break;
					case OUTPUT_CLEAR:
						pew |= 0b010;
						break;
					case OUTPUT_SET:
						pew |= 0b011;
						break;
					case OUTPUT_NO_CHANGE:
						pew |= 0b100;
						break;
					}
				}
				addTxPayloadData(pew);
			}

		}
		break;
	case GPIO_RESPONSE_PAYLOAD:
		;//this is here to stop a silly error
		PortPin p = GPIO_A0_PIN;
		uint32_t pinVal;
		uint32_t w;
		uint32_t i;
		for( w = 0; w < 6; ++w){
			pinVal = 0;
			for(i = 0; i < 8; ++i){
				pinVal >>= 4;
				pinVal |= makePinConfigNum(p) << 28;
				++p;
			}
			addTxPayloadData((uint32_t)pinVal);
		}



		break;
	case ANALOG_RESPONSE_PAYLOAD:
		;//this is here to stop a silly error
		uint32_t enMask = getAdcChannelMask();
		enMask |= isDacEnable(0) ? (1<<16) : 0;
		enMask |= isDacEnable(0) ? (1<<17) : 0;
		addTxPayloadData((uint32_t)enMask);
		uint32_t j = 0;
		for(uint32_t i = 0; i < getAdcCount(); ++i){

			if(isAdcEnabled(i)){//skip channels that are not enabled
				float a = getAdcValue(i);
				addTxPayloadFloat(a);
			}
			++j;
		}
		break;
	case STEPPER_RESPONSE_PAYLOAD:
		for(uint32_t i = 0; i < STEPPER_CHANNELS; ++i){

			if(isStepperConfigured(i)){
				uint32_t t = i << 24;
				t |= getStepperState(i) << 16;
				addTxPayloadData(t);
				float pos = getStepperPosition(i);
				float vel = getStepperVelocity(i);
				addTxPayloadFloat(pos);
				addTxPayloadFloat(vel);

			}

		}

		break;

	case THERMAL_RESPONSE_PAYLOAD:
		//add all thermal channels to packet
		for(int i = 0; i < OUTPUT_CHANNEL_NUM; ++i){
			if(getSigGenType(i) == THERMAL_SIG){
				TempControlState* tcs = getTempControlState(i);
				uint32_t chan = i << 24;
				chan |= (tcs->thermistorNum)<< 16;
				addTxPayloadData(chan);
				addTxPayloadFloat(tcs->temperature);
				addTxPayloadFloat(getSigGenOutputs()[i]);
			}
		}
		break;
	case PWM_RESPONSE_PAYLOAD:
		//add all thermal channels to packet
		for(uint32_t i = 0; i < OUTPUT_CHANNEL_NUM; ++i){
//			if(getSigGenType(i) != THERMAL_SIG && getSigGenType(i) != STEPPER_SIG){
			addTxPayloadData((i << 8) | getSigGenType(i));
			addTxPayloadFloat(getSigGenOutputs()[i]);
//			}
		}
		float v = getVRail();
		addTxPayloadFloat(v);
		break;
	case HADC_RESPONSE_PAYLOAD:
		for(uint32_t i = 0; i < 4; ++i){
			//send channel word (byte packed MSB to LSB): undef, channel, pos input type, neg input type
			uint32_t cw = i << 16;

			cw |= ad7172GetInput(i, true) << 8;
			cw |= ad7172GetInput(i, false) << 0;
			addTxPayloadData(cw);
			addTxPayloadFloat(ad7172GetValue(i));
		}
		break;
	case IMU_RESPONSE_PAYLOAD:
		addTxPayloadFloat(getLsm6ds3AccX());
		addTxPayloadFloat(getLsm6ds3AccY());
		addTxPayloadFloat(getLsm6ds3AccZ());
		break;

	case TEMPERATURE_RESPONSE_PAYLOAD:
		for(uint32_t i = 0; i < getThermistorCount(); ++i){
			addTxPayloadData(i << 24);
			addTxPayloadFloat(getThermistorTemperature(i));
		}
		break;
	case APP_DATA_PAYLOAD:
		for(uint32_t i = 0; i < getTxAppDataLength(); ++i){
			addTxPayloadFloat(getTxAppData(i));
		}
		break;
	case ENCODER_RESPONSE_PAYLOAD:
		for(uint32_t i = 0; i < getEncoderCount(); ++i){
			if(isEncoderEnabled(i)){
				addTxPayloadData(i << 24);
				addTxPayloadFloat(getEncoderDistance(i));
			}
		}
		break;
	case TRAJECTORY_RESPONSE_PAYLOAD:
		//respond with current setpoint and end time of last trajectory for each fb channel
		for(uint32_t i = 0; i < NUM_FEEDBACK_CHANNELS; ++i){

			if(isConfigured(i)){
				addTxPayloadData(i);
				addTxPayloadFloat(getCurrentSetpoint(i));
				addTxPayloadData(getTrajectoryQueueLength(i));
				addTxPayloadData(getLastEndTime(i));
			}
		}

		break;
	case FEEDBACK_RESPONSE_PAYLOAD:
		addTxPayloadFloat(getFeedbackSampleTime());
		for(uint32_t i = 0; i < NUM_FEEDBACK_CHANNELS; ++i){
			uint32_t chan = i << 24;
			addTxPayloadData(chan);
			addTxPayloadFloat(getFeedbackInput(i));
			addTxPayloadFloat(getFeedbackOutput(i));
		}

		break;
	case BRUSHLESS_RESPONSE_PAYLOAD:
		addTxPayloadFloat(getBrushlessRevPerSec());
		addTxPayloadFloat(getBrushlessPosition());
		addTxPayloadFloat(getBrushlessDrive());
		break;
	case EEPROM_RESPONSE_PAYLOAD:
//		m_eepromWordIndex
//		m_eepromWordCount
		;
		uint32_t startIndex = m_eepromWordIndex;
		uint32_t totalNum = m_eepromWordCount;
		//don't send data beyond the eeprom size
		if(startIndex >= getEepromSize()){
			//if the start index is beyond the end of the EEPROM then send the size
			startIndex = getEepromSize();
			totalNum = 0;
		} else if(startIndex + totalNum >= getEepromSize()){
			totalNum = getEepromSize() - startIndex;
		}
		addTxPayloadData((startIndex << 16) | (totalNum & 0xffff));
		for(uint32_t a = startIndex; a < (startIndex + totalNum); ++a){
			addTxPayloadData(getEepromValue(a));
		}
		break;

	case SOFT_PWM_RESPONSE_PAYLOAD:
		for(uint32_t i = 0; i < getSoftPwmCount(); ++i){
			if(isSoftPwmConfigured(i)){
				uint32_t softChan = i << 24;
				addTxPayloadData(softChan);
				addTxPayloadFloat(getSoftPwmSetpoint(i));
			}
		}
		break;
	case STREAMING_RESPONSE_PAYLOAD:{

			uint32_t streamingWord = m_streamingChannel << 24;
			streamingWord |= isStreamSecondsNotHostTime(m_streamingChannel) ? 0b01 : 0b00;
//			streamingWord |= (getStreamInputIndex(m_streamingChannel) & 0xff) << 16;
//			streamingWord |= (getStreamInputType(m_streamingChannel) & 0xff)<< 8;
			addTxPayloadData(streamingWord);
			addTxPayloadFloat(getStreamSamplePeriod(m_streamingChannel));
			if(isStreamSecondsNotHostTime(m_streamingChannel)){
				addTxPayloadFloat(getStreamFirstSeconds(m_streamingChannel));
			} else {
				addTxPayloadData(getStreamFirstSampleTime(m_streamingChannel));
			}
			bool didNotAddStuff = true;
			while(isNextStreamSampleAvailable(m_streamingChannel)){
				didNotAddStuff = false;
				addTxPayloadFloat(pullNextStreamSample(m_streamingChannel));
			}
			if(didNotAddStuff){
				asm("nop");
			}
		}
		break;

	case REGISTER_RESPONSE_PAYLOAD:{
			uint32_t incr = 0;
			Payload* p = NULL;
			//are we responding to a config or request payload
			if(m_registerConfigPayload != NULL){
				p = m_registerConfigPayload;
				m_registerConfigPayload = NULL;
				incr = 2;
			} else if(m_registerRequestPayload != NULL){
				p = m_registerRequestPayload;
				m_registerRequestPayload = NULL;
				incr = 1;
			}
			if(p != NULL){

				//assuming we are responding to something and not just mistaken, then step through received payload in order to contruct tx payload
				uint32_t pLen = getPayloadLength(p);


				uint32_t i = 1;
				while(i <= pLen){
					uint32_t id = getPayloadWord(p, i);
					ModuleId mid = (id >> 24);
					uint32_t rid = (id >> 16) & 0xff;
					uint32_t ch = (id >> 8) & 0xff;

					i += incr;
					addTxPayloadData(id);
					addTxPayloadData(getRegsterWord(mid, rid, ch));


				}
			}

		}

		break;

	//now a whole bunch of config payloads that the SFDQ (which is a peripheral) will never send
	case REGISTER_CONFIG_PAYLOAD:
	case PERIPHERAL_CONFIG_PAYLOAD:
	case STREAMING_CONFIG_PAYLOAD:
	case SOFT_PWM_CONFIG_PAYLOAD:
	case THERMAL_CONTROL_PAYLOAD:
	case THERMAL_REQUEST_PAYLOAD:
	case PWM_CONTROL_PAYLOAD:
	case HADC_CONTROL_PAYLOAD:
	case HADC_REQUEST_PAYLOAD:
	case ERROR_ACK_PAYLOAD:
	case FIRMWARE_REQUEST_PAYLOAD:
	case STEPPER_CONTROL_PAYLOAD:
	case STEPPER_CONFIG_PAYLOAD:
	//case IO_REQUEST_PAYLOAD:
	case IO_CONTROL_PAYLOAD:
	case TEMPERATURE_CONFIG_PAYLOAD:
	case ENCODER_CONFIG_PAYLOAD:
	case BRUSHLESS_MOTOR_CONFIG_PAYLOAD:
	case FEEDBACK_CONFIG_PAYLOAD:
	case TRAJECTORY_PAYLOAD:
	case IO_REQUEST_PAYLOAD:
	case TEMPERATURE_REQUEST_PAYLOAD:
	case FIRMWARE_PROGRAM_PAYLOAD:
	case APP_DATA_REQUEST_PAYLOAD:
	case EEPROM_WRITE_REQUEST_PAYLOAD:
	case EEPROM_READ_REQUEST_PAYLOAD:
	case GPIO_CONTROL_PAYLOAD:
	case ANALOG_CONTROL_PAYLOAD:
	case GPIO_REQUEST_PAYLOAD:

		break;
	}
	endTxPayload();
}
static uint32_t makePinConfigNum(PortPin pinNum){
	uint32_t result = 0;
	PinDir d = getPinDirection(pinNum);
	switch(d){
	case PIN_IN:
		if(getPinPullup(pinNum) == PULL_UP){
			result |= (isPinInputSet(pinNum) ? 9 : 10);
		} else {
			result |= (isPinInputSet(pinNum) ? 1 : 0);
		}
		break;
	case PIN_OUT_LO:
	case PIN_OUT_HI:
	case PIN_OUT:
		result |= (isPinInputSet(pinNum) ? 3 : 2);
		break;
	case PIN_AN:
		result |= 0b100;
		break;
	case PIN_AF1:
		result |= 0b101;
		break;
	case PIN_AF2:
		result |= 0b110;
		break;
	}
	result &= 0xf;
	return result;
}
/**
* sets whether all payloads will be accepted, regardless of whether the received ID matches this SDFQ
 */
void setAcceptAllPayloads(bool a){
	acceptAllPayloads = a;
}
