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

#ifndef INC_REGISTERCONFIG_H_
#define INC_REGISTERCONFIG_H_

//*************************************************
//notes
//*************************************************
//this module sets the registers of other modules according to the comm protocol

//*************************************************
//includes
//*************************************************
#include <stdbool.h>
#include <stdint.h>

//*************************************************
//defines
//*************************************************
#define REGISTER_MODULE_SHIFT_NUM 8

//*************************************************
//Types
//*************************************************

typedef enum {
	MODULE_ID_GPIO                   =  0,
	MODULE_ID_FEEDBACKCONTROL        =  1,
	MODULE_ID_BRUSHLESSMOTOR         =  2,
	MODULE_ID_ADCMANAGER             =  3,
	MODULE_ID_SFDQPACKETS            =  4,
	MODULE_ID_DAC                    =  5,
	MODULE_ID_SOFTPWM                =  6,
	MODULE_ID_THERMISTORS            =  7,
	MODULE_ID_STREAMINGMANAGER       =  8,
	MODULE_ID_STEPPERMOTOR           =  9,
	MODULE_ID_ENCODER                = 10,
	MODULE_ID_ADS131A0X              = 11,
	MODULE_ID_AD7172                 = 12,
	MODULE_ID_SFM3000                = 13,
	MODULE_ID_SFM3019                = 14,
	MODULE_ID_PORTEXPANDERMANAGER    = 15,
	MODULE_ID_AMT22                  = 16,
	MODULE_ID_AS5048A                = 17,
	MODULE_ID_PAC1710                = 18,
	MODULE_ID_MAX31865               = 19,
	MODULE_ID_HW_ENCODER             = 20,
	MODULE_ID_FASTCODE_MONITOR       = 21,
	MODULE_ID_ADS131M0X				 = 22,
	MODULE_ID_HSC_PRESSURE_SENSOR    = 23,



} ModuleId;

/**
 * identifies both the module and registerId
 */
typedef enum {
	GPIO_MODE_REGISTER                    = (MODULE_ID_GPIO << REGISTER_MODULE_SHIFT_NUM) | 0,
	GPIO_OUTPUT_TYPE_REGISTER             = (MODULE_ID_GPIO << REGISTER_MODULE_SHIFT_NUM) | 1,
	GPIO_OUTPUT_SPEED_REGISTER            = (MODULE_ID_GPIO << REGISTER_MODULE_SHIFT_NUM) | 2,
	GPIO_PULLUP_PULLDOWN_REGISTER         = (MODULE_ID_GPIO << REGISTER_MODULE_SHIFT_NUM) | 3,
	GPIO_OUTPUT_DATA_REGISTER             = (MODULE_ID_GPIO << REGISTER_MODULE_SHIFT_NUM) | 4,
	GPIO_SET_RESET_REGISTER               = (MODULE_ID_GPIO << REGISTER_MODULE_SHIFT_NUM) | 5,

	FEEDBACK_PLUMBING_REGISTER            = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 0,
	FEEDBACK_C0_REGISTER                  = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 1,
	FEEDBACK_C1_REGISTER                  = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 2,
	FEEDBACK_C2_REGISTER                  = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 3,
	FEEDBACK_C3_REGISTER                  = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 4,
	FEEDBACK_D1_REGISTER                  = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 5,
	FEEDBACK_D2_REGISTER                  = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 6,
	FEEDBACK_MIN_OUT_REGISTER             = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 7,
	FEEDBACK_MAX_OUT_REGISTER             = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 8,
	FEEDBACK_MIN_IN_REGISTER              = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 9,
	FEEDBACK_MAX_IN_REGISTER             = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 10,
	FEEDBACK_CONFIG_REGISTER              = (MODULE_ID_FEEDBACKCONTROL << REGISTER_MODULE_SHIFT_NUM) | 11,

	BRUSHLESS_SETUP_REGISTER              = (MODULE_ID_BRUSHLESSMOTOR << REGISTER_MODULE_SHIFT_NUM) | 0,
	BRUSHLESS_PHASE_PER_REV_REGISTER      = (MODULE_ID_BRUSHLESSMOTOR << REGISTER_MODULE_SHIFT_NUM) | 1,
	BRUSHLESS_PHASE_OFFSET_REGISTER       = (MODULE_ID_BRUSHLESSMOTOR << REGISTER_MODULE_SHIFT_NUM) | 2,
	BRUSHLESS_PHASE_ADVANCE_REGISTER      = (MODULE_ID_BRUSHLESSMOTOR << REGISTER_MODULE_SHIFT_NUM) | 3,
	BRUSHLESS_DRIVE_ABC_REGISTER          = (MODULE_ID_BRUSHLESSMOTOR << REGISTER_MODULE_SHIFT_NUM) | 4,
	BRUSHLESS_STATE_SINKS_REGISTER        = (MODULE_ID_BRUSHLESSMOTOR << REGISTER_MODULE_SHIFT_NUM) | 5,

	ADC_SINK_REGISTER                     = (MODULE_ID_ADCMANAGER << REGISTER_MODULE_SHIFT_NUM) | 0,
	ADC_OFFSET_REGISTER                   = (MODULE_ID_ADCMANAGER << REGISTER_MODULE_SHIFT_NUM) | 1,
	ADC_GAIN_REGISTER                     = (MODULE_ID_ADCMANAGER << REGISTER_MODULE_SHIFT_NUM) | 2,
	ADC_TC_REGISTER                       = (MODULE_ID_ADCMANAGER << REGISTER_MODULE_SHIFT_NUM) | 3,

	PACKETS_STREAMING_SOURCE_REGISTER     = (MODULE_ID_SFDQPACKETS << REGISTER_MODULE_SHIFT_NUM) | 0,

	DAC_SOURCE_REGISTER                   = (MODULE_ID_DAC << REGISTER_MODULE_SHIFT_NUM) | 0,

	SOFT_PWM_CONFIG_REGISTER              = (MODULE_ID_SOFTPWM << REGISTER_MODULE_SHIFT_NUM) | 0,
	SOFT_PWM_PERIOD_REGISTER              = (MODULE_ID_SOFTPWM << REGISTER_MODULE_SHIFT_NUM) | 1,

	THERMISTOR_CONFIG_REGISTER            = (MODULE_ID_THERMISTORS << REGISTER_MODULE_SHIFT_NUM) | 0,
	THERMISTOR_C0_REGISTER                = (MODULE_ID_THERMISTORS << REGISTER_MODULE_SHIFT_NUM) | 1,
	THERMISTOR_C1_REGISTER                = (MODULE_ID_THERMISTORS << REGISTER_MODULE_SHIFT_NUM) | 2,
	THERMISTOR_C2_REGISTER                = (MODULE_ID_THERMISTORS << REGISTER_MODULE_SHIFT_NUM) | 3,
	THERMISTOR_C3_REGISTER                = (MODULE_ID_THERMISTORS << REGISTER_MODULE_SHIFT_NUM) | 4,

	STREAMING_CONFIG_REGISTER             = (MODULE_ID_STREAMINGMANAGER << REGISTER_MODULE_SHIFT_NUM) | 0,
	STREAMING_PERIOD_REGISTER             = (MODULE_ID_STREAMINGMANAGER << REGISTER_MODULE_SHIFT_NUM) | 1,
   	STEPPER_CONFIG_REGISTER               = (MODULE_ID_STEPPERMOTOR << REGISTER_MODULE_SHIFT_NUM) | 0,
	STEPPER_DISTANCE_PER_STEP_REGISTER    = (MODULE_ID_STEPPERMOTOR << REGISTER_MODULE_SHIFT_NUM) | 1,
	STEPPER_PWM_FRACTION_REGISTER         = (MODULE_ID_STEPPERMOTOR << REGISTER_MODULE_SHIFT_NUM) | 2,

	AD7172_SINK_REGISTER                  = (MODULE_ID_AD7172 << REGISTER_MODULE_SHIFT_NUM) | 0,
	AD7172_OFFSET_REGISTER                = (MODULE_ID_AD7172 << REGISTER_MODULE_SHIFT_NUM) | 1,
	AD7172_GAIN_REGISTER                  = (MODULE_ID_AD7172 << REGISTER_MODULE_SHIFT_NUM) | 2,
	AD7172_SETUP_REGISTER                 = (MODULE_ID_AD7172 << REGISTER_MODULE_SHIFT_NUM) | 3,

	HW_ENCODER_CONFIG_REGISTER            = (MODULE_ID_HW_ENCODER << REGISTER_MODULE_SHIFT_NUM) | 0,
	HW_ENCODER_OFFSET_REGISTER            = (MODULE_ID_HW_ENCODER << REGISTER_MODULE_SHIFT_NUM) | 1,
	HW_ENCODER_SCALE_REGISTER             = (MODULE_ID_HW_ENCODER << REGISTER_MODULE_SHIFT_NUM) | 2,
	HW_ENCODER_COUNTS_PER_REV_REGISTER    = (MODULE_ID_HW_ENCODER << REGISTER_MODULE_SHIFT_NUM) | 3,

	FASTCODE_MONITOR_SINK_REGISTER        = (MODULE_ID_FASTCODE_MONITOR << REGISTER_MODULE_SHIFT_NUM) | 0,

	ADS131M0X_SINKS1_REGISTER             = (MODULE_ID_ADS131M0X <<  REGISTER_MODULE_SHIFT_NUM) | 0,
	ADS131M0X_SINKS2_REGISTER             = (MODULE_ID_ADS131M0X <<  REGISTER_MODULE_SHIFT_NUM) | 1,
	ADS131M0X_CONFIG_REGISTER             = (MODULE_ID_ADS131M0X <<  REGISTER_MODULE_SHIFT_NUM) | 2,

	HSC_SINKS_REGISTER                    = (MODULE_ID_HSC_PRESSURE_SENSOR << REGISTER_MODULE_SHIFT_NUM) | 0,
	HSC_CONFIG_REGISTER                   = (MODULE_ID_HSC_PRESSURE_SENSOR << REGISTER_MODULE_SHIFT_NUM) | 1,
	HSC_MINPRESSURE_REGISTER              = (MODULE_ID_HSC_PRESSURE_SENSOR << REGISTER_MODULE_SHIFT_NUM) | 2,
	HSC_MAXPRESSURE_REGISTER              = (MODULE_ID_HSC_PRESSURE_SENSOR << REGISTER_MODULE_SHIFT_NUM) | 3,

} RegisterId;

//typedef union {
//	uint32_t u;
//	int32_t i;
//	float f;
//} Generic32;

//*************************************************
//Variables
//*************************************************


//*************************************************
//function prototypes
//*************************************************

/**
 * configures one register with the 32 bit value provided.
 * @param moduleId the id of the module whose register is desired to be changed
 * @param channel the modules channel whose register is desired to be changed
 * @param registerId the id of the register desired to be changed
 * @param value the value to change it to. Although this is passes as a uint, it may be used as a float or an int, depending on the register
 */
void configRegister(uint8_t moduleId, uint8_t registerId, uint8_t channel, uint32_t value);
/**
 * gets the value of one register as a 32 bit integer.
 * @param channel the modules channel whose register is desired to be changed
 * @param registerId the id of the register desired to be changed
 */
uint32_t getRegsterWord(uint8_t moduleId, uint8_t registerId, uint8_t channel);
#endif /* INC_REGISTERCONFIG_H_ */
