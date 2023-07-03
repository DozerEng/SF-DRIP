/*
Copyright (c) 2021 STARFISH PRODUCT ENGINEERING INC.
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
//notes
//*************************************************
//*************************************************
//includes
//*************************************************
#include <registerConfig.h>
#include <ad7172.h>
#include <hardwareEncoder.h>
#include <adcManager.h>
#include <brushlessMotor.h>
#include <feedbackControl.h>
#include <fastcodeUtil.h>
#include <ads131m0x.h>
#include <hscPressureSensor.h>
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
static uint32_t fromFloat(float f);
static float toFloat(uint32_t u);

//*************************************************
//code
//*************************************************
/**
 * configures one register with the 32 bit value provided.
 * @param moduleId the id of the module whose register is desired to be changed
 * @param channel the modules channel whose register is desired to be changed
 * @param registerId the id of the register desired to be changed
 * @param value the value to change it to. Although this is passes as a uint, it may be used as a float or an int, depending on the register
 */
void configRegister(uint8_t moduleId, uint8_t registerId, uint8_t channel, uint32_t value){

	RegisterId r = (((uint32_t)moduleId) << 8) | ((uint32_t)registerId);
	switch(r){
	case GPIO_MODE_REGISTER:
		break;
	case GPIO_OUTPUT_TYPE_REGISTER:
		break;
	case GPIO_OUTPUT_SPEED_REGISTER:
		break;
	case GPIO_PULLUP_PULLDOWN_REGISTER:
		break;
	case GPIO_OUTPUT_DATA_REGISTER:
		break;
	case GPIO_SET_RESET_REGISTER:
		break;

	case FEEDBACK_PLUMBING_REGISTER:
		setFeedbackSink(channel, 0, (value >>  24) & 0xff);
		setFeedbackSink(channel, 1, (value >>  16) & 0xff);
		setFeedbackSink(channel, 2, (value >>   8) & 0xff);
		setFeedbackSink(channel, 3, (value >>   0) & 0xff);
		break;
	case FEEDBACK_C0_REGISTER:
		break;
	case FEEDBACK_C1_REGISTER:
		break;
	case FEEDBACK_C2_REGISTER:
		break;
	case FEEDBACK_C3_REGISTER:
		break;
	case FEEDBACK_D1_REGISTER:
		break;
	case FEEDBACK_D2_REGISTER:
		break;
	case FEEDBACK_MIN_OUT_REGISTER:
		break;
	case FEEDBACK_MAX_OUT_REGISTER:
		break;
	case FEEDBACK_MIN_IN_REGISTER:
		break;
	case FEEDBACK_MAX_IN_REGISTER:
		break;
	case FEEDBACK_CONFIG_REGISTER:
		break;

	case BRUSHLESS_SETUP_REGISTER:
		break;
	case BRUSHLESS_PHASE_PER_REV_REGISTER:
		break;
	case BRUSHLESS_PHASE_OFFSET_REGISTER:
		break;
	case BRUSHLESS_PHASE_ADVANCE_REGISTER:
		break;
	case BRUSHLESS_DRIVE_ABC_REGISTER:
		break;
	case BRUSHLESS_STATE_SINKS_REGISTER:{

			setBrushlessSink(0, (value >>  24) & 0xff);
			setBrushlessSink(1, (value >>  16) & 0xff);
			setBrushlessSink(2, (value >>   8) & 0xff);
			setBrushlessSink(3, (value >>   0) & 0xff);

		}
		break;

	case ADC_SINK_REGISTER:
		setAdcSink(channel, 0xff & value);
		break;
	case ADC_OFFSET_REGISTER:
		setAdcOffset(channel, toFloat(value));
		break;
	case ADC_GAIN_REGISTER:
		setAdcGain(channel, toFloat(value));
		break;
	case ADC_TC_REGISTER:
		setAdcTimeConstant(channel, toFloat(value));
		break;

	case PACKETS_STREAMING_SOURCE_REGISTER:
		break;

	case DAC_SOURCE_REGISTER:
		break;

	case SOFT_PWM_CONFIG_REGISTER:
		break;
	case SOFT_PWM_PERIOD_REGISTER:
		break;

	case THERMISTOR_CONFIG_REGISTER:
		break;
	case THERMISTOR_C0_REGISTER:
		break;
	case THERMISTOR_C1_REGISTER:
		break;
	case THERMISTOR_C2_REGISTER:
		break;
	case THERMISTOR_C3_REGISTER:
		break;

	case STREAMING_CONFIG_REGISTER:
		break;
	case STREAMING_PERIOD_REGISTER:
		break;

	case STEPPER_CONFIG_REGISTER:
		break;
	case STEPPER_DISTANCE_PER_STEP_REGISTER:
		break;
	case STEPPER_PWM_FRACTION_REGISTER:
		break;
	case AD7172_SINK_REGISTER:
		ad7172SetSink(channel, value & 0xff);
		break;
	case AD7172_OFFSET_REGISTER:
		ad7172SetOffset(channel, toFloat(value));
		break;
	case AD7172_GAIN_REGISTER:
		ad7172SetGain(channel, toFloat(value));
		break;
	case AD7172_SETUP_REGISTER:{
			bool e = (value & (0xff << 16)) != 0;
			Ad7172Inputs pos = (value & (0xff << 8));
			Ad7172Inputs neg = (value & (0xff << 0));
			ad7172Config(channel, pos, neg, e);
		}
		break;
	case HW_ENCODER_CONFIG_REGISTER:
		setupHardwareEncoder(channel, value & 0xff);
		break;
	case HW_ENCODER_OFFSET_REGISTER:
		setHardwareEncoderOffset(channel, toFloat(value));
		break;
	case HW_ENCODER_SCALE_REGISTER:
		setHardwareEncoderDistancePerCount(channel, toFloat(value));
		break;
	case HW_ENCODER_COUNTS_PER_REV_REGISTER:
		setHardwareEncoderCountsPerRev(channel, toFloat(value));
		break;
	case FASTCODE_MONITOR_SINK_REGISTER:
		setFastcodeSink(0, (value >> 0) & 0xff);
		setFastcodeSink(1, (value >> 8) & 0xff);
		setFastcodeSink(2, (value >> 16) & 0xff);
		setFastcodeSink(3, (value >> 24) & 0xff);
		break;
	case ADS131M0X_SINKS1_REGISTER:
		setAds131m0xSink(0, (value >> 0) & 0xff);
		setAds131m0xSink(1, (value >> 8) & 0xff);
		setAds131m0xSink(2, (value >> 16) & 0xff);
		setAds131m0xSink(3, (value >> 24) & 0xff);
		break;
	case ADS131M0X_SINKS2_REGISTER:
		setAds131m0xSink(4, (value >> 0) & 0xff);
		setAds131m0xSink(5, (value >> 8) & 0xff);
		setAds131m0xSink(6, (value >> 16) & 0xff);
		setAds131m0xSink(7, (value >> 24) & 0xff);
		break;
	case ADS131M0X_CONFIG_REGISTER:
		;
		PortPin drdy = (value >> 0) & 0xff;
		PortPin cs = (value >> 8) & 0xff;
		uint8_t numChans = (value >> 16) & 0xf;
		uint8_t freq = (value >> 20) & 0xf;
		uint8_t spi = (value >> 24) & 0xf;
		ads131m0xConfig((SpiDev)(spi - 1), (ADS131M0xSampleRate)freq, (uint32_t)numChans, cs, drdy);
		break;

	case HSC_SINKS_REGISTER:
		hscSetSinks(channel, (value >> 8) & 0xff, (value >> 0) & 0xff);
		break;
	case HSC_CONFIG_REGISTER:
		hscSetSpi(channel, (value >> 8) & 0xf, (value >> 0) & 0xff);
		break;
	case HSC_MINPRESSURE_REGISTER:
		hscSetMinPressure(channel, toFloat(value));
		break;
	case HSC_MAXPRESSURE_REGISTER:
		hscSetMaxPressure(channel, toFloat(value));
		break;
	default:
		//do nothing. the module ID doesn't match anything
		break;
	}
}
/**
 * gets the value of one register as a 32 bit integer.
 * @param moduleId the id of the module whose register is desired to be changed
 * @param channel the modules channel whose register is desired to be changed
 * @param registerId the id of the register desired to be changed
 */
uint32_t getRegsterWord(uint8_t moduleId, uint8_t registerId, uint8_t channel){
	uint32_t result = -1;
	RegisterId r = (((uint32_t)moduleId) << 8) | ((uint32_t)registerId);
	switch(r){
	case GPIO_MODE_REGISTER:
		break;
	case GPIO_OUTPUT_TYPE_REGISTER:
		break;
	case GPIO_OUTPUT_SPEED_REGISTER:
		break;
	case GPIO_PULLUP_PULLDOWN_REGISTER:
		break;
	case GPIO_OUTPUT_DATA_REGISTER:
		break;
	case GPIO_SET_RESET_REGISTER:
		break;

	case FEEDBACK_PLUMBING_REGISTER:
		result =  ((uint32_t)getFeedbackSink(channel, 0) & 0xff) << 24;
		result |= ((uint32_t)getFeedbackSink(channel, 1) & 0xff) << 16;
		result |= ((uint32_t)getFeedbackSink(channel, 2) & 0xff) << 8;
		result |= ((uint32_t)getFeedbackSink(channel, 3) & 0xff) << 0;
		break;
	case FEEDBACK_C0_REGISTER:
		break;
	case FEEDBACK_C1_REGISTER:
		break;
	case FEEDBACK_C2_REGISTER:
		break;
	case FEEDBACK_C3_REGISTER:
		break;
	case FEEDBACK_D1_REGISTER:
		break;
	case FEEDBACK_D2_REGISTER:
		break;
	case FEEDBACK_MIN_OUT_REGISTER:
		break;
	case FEEDBACK_MAX_OUT_REGISTER:
		break;
	case FEEDBACK_MIN_IN_REGISTER:
		break;
	case FEEDBACK_MAX_IN_REGISTER:
		break;
	case FEEDBACK_CONFIG_REGISTER:
		break;

	case BRUSHLESS_SETUP_REGISTER:
		break;
	case BRUSHLESS_PHASE_PER_REV_REGISTER:
		break;
	case BRUSHLESS_PHASE_OFFSET_REGISTER:
		break;
	case BRUSHLESS_PHASE_ADVANCE_REGISTER:
		break;
	case BRUSHLESS_DRIVE_ABC_REGISTER:
		break;
	case BRUSHLESS_STATE_SINKS_REGISTER:
		result =  ((uint32_t)getBrushlessSink(0) & 0xff) << 24;
		result |= ((uint32_t)getBrushlessSink(1) & 0xff) << 16;
		result |= ((uint32_t)getBrushlessSink(2) & 0xff) << 8;
		result |= ((uint32_t)getBrushlessSink(3) & 0xff) << 0;

		break;

	case ADC_SINK_REGISTER:
		result = getAdcSink(channel);
		break;
	case ADC_OFFSET_REGISTER:
		result = fromFloat(getAdcOffset(channel));
		break;
	case ADC_GAIN_REGISTER:
		result = fromFloat(getAdcGain(channel));
		break;
	case ADC_TC_REGISTER:
		result = fromFloat(getAdcTimeConstant(channel));
		break;

	case PACKETS_STREAMING_SOURCE_REGISTER:
		break;

	case DAC_SOURCE_REGISTER:
		break;

	case SOFT_PWM_CONFIG_REGISTER:
		break;
	case SOFT_PWM_PERIOD_REGISTER:
		break;

	case THERMISTOR_CONFIG_REGISTER:
		break;
	case THERMISTOR_C0_REGISTER:
		break;
	case THERMISTOR_C1_REGISTER:
		break;
	case THERMISTOR_C2_REGISTER:
		break;
	case THERMISTOR_C3_REGISTER:
		break;

	case STREAMING_CONFIG_REGISTER:
		break;
	case STREAMING_PERIOD_REGISTER:
		break;

	case STEPPER_CONFIG_REGISTER:
		break;
	case STEPPER_DISTANCE_PER_STEP_REGISTER:
		break;
	case STEPPER_PWM_FRACTION_REGISTER:
		break;
	case AD7172_SINK_REGISTER:
		break;
	case AD7172_OFFSET_REGISTER:
		break;
	case AD7172_GAIN_REGISTER:

		break;
	case AD7172_SETUP_REGISTER:{
//			bool e = (value & (0xff << 16)) != 0;
//			Ad7172Inputs pos = (value & (0xff << 8));
//			Ad7172Inputs neg = (value & (0xff << 0));
//			ad7172Config(channel, pos, neg, e);
		}
		break;
	case HW_ENCODER_CONFIG_REGISTER:
		result = getHardwareEncoderSink(channel) & 0xff;
		break;
	case HW_ENCODER_OFFSET_REGISTER:
		result = fromFloat(getHardwareEncoderOffset(channel));
		break;
	case HW_ENCODER_SCALE_REGISTER:
		result = fromFloat(getHardwareEncoderDistancePerCount(channel));
		break;
	case HW_ENCODER_COUNTS_PER_REV_REGISTER:
		result = fromFloat(getHardwareEncoderCountsPerRev(channel));
		break;
	case FASTCODE_MONITOR_SINK_REGISTER:
		result =  ((uint32_t)getFastcodeSink(0) & 0xff) << 0;
		result |= ((uint32_t)getFastcodeSink(1) & 0xff) << 8;
		result |= ((uint32_t)getFastcodeSink(2) & 0xff) << 16;
		result |= ((uint32_t)getFastcodeSink(3) & 0xff) << 24;
		break;
	case ADS131M0X_SINKS1_REGISTER:
		result =  ((uint32_t)getAds131m0xSink(0) & 0xff) << 0;
		result |= ((uint32_t)getAds131m0xSink(1) & 0xff) << 8;
		result |= ((uint32_t)getAds131m0xSink(2) & 0xff) << 16;
		result |= ((uint32_t)getAds131m0xSink(3) & 0xff) << 24;
		break;
	case ADS131M0X_SINKS2_REGISTER:
		result =  ((uint32_t)getAds131m0xSink(4) & 0xff) << 0;
		result |= ((uint32_t)getAds131m0xSink(5) & 0xff) << 8;
		result |= ((uint32_t)getAds131m0xSink(6) & 0xff) << 16;
		result |= ((uint32_t)getAds131m0xSink(7) & 0xff) << 24;
		break;
	case ADS131M0X_CONFIG_REGISTER:
		;

		uint32_t drdy = (uint32_t)getAds131m0xDrdyPin();
		uint32_t cs = (uint32_t)getAds131m0xCsPin();
		uint32_t numChans = getAds131m0xNumChannels();
		uint32_t freq = getAds131m0xSampleRate();
		uint32_t spi = (uint32_t)(getAds131m0xSpi() + 1);

		result = drdy;
		result |= cs << 8;
		result |= numChans << 16;
		result |= freq << 20;
		result |= spi << 24;

		break;
	case HSC_SINKS_REGISTER:
		result = (hscGetPressureSink(channel) << 8) | hscGetPressureSink(channel);
		break;

	case HSC_CONFIG_REGISTER:
		result = (hscGetSpi(channel) << 8) | (hscGetCsPin(channel));
		break;
	case HSC_MAXPRESSURE_REGISTER:
		result = hscGetMaxPressure(channel);
		break;
	case HSC_MINPRESSURE_REGISTER:
		result = hscGetMinPressure(channel);
		break;
	}

	return result;
}
/**
 * casts a 32 bit float to a 32 bit unsigned int
 * by dereferencing a pointer to it
 */
static uint32_t fromFloat(float f){
	return *((uint32_t*)(&f));
}
/**
 * casts a 32 bit unsigned int to a 32 bit float
 * by dereferencing a pointer to it
 */
static float toFloat(uint32_t u){
	return *((float*)(&u));
}
