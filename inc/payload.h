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

#ifndef INC_PAYLOAD_H_
#define INC_PAYLOAD_H_

#include <stdint.h>

typedef enum {

	ACK_PAYLOAD 				   = 0x02,
	ANALOG_CONTROL_PAYLOAD         = 0x2e,
	ANALOG_RESPONSE_PAYLOAD        = 0x2f,
	APP_DATA_PAYLOAD               = 0x24,
	APP_DATA_REQUEST_PAYLOAD       = 0x23,
	BAUDRATE_PAYLOAD               = 0x31,
	BRUSHLESS_MOTOR_CONFIG_PAYLOAD = 0x1f,
	BRUSHLESS_RESPONSE_PAYLOAD     = 0x2b,
	EEPROM_READ_REQUEST_PAYLOAD    = 0x28,
	EEPROM_RESPONSE_PAYLOAD        = 0x29,
	EEPROM_WRITE_REQUEST_PAYLOAD   = 0x27,
	ENCODER_CONFIG_PAYLOAD		   = 0x1c,
	ENCODER_RESPONSE_PAYLOAD	   = 0x1d,
	ERROR_ACK_PAYLOAD 			   = 0x0d,
	ERROR_PAYLOAD 				   = 0x0c,
	FEEDBACK_CONFIG_PAYLOAD		   = 0x1e,
	FEEDBACK_RESPONSE_PAYLOAD      = 0x2a,
	FIRMWARE_PROGRAM_ACK_PAYLOAD   = 0x26,
	FIRMWARE_PROGRAM_PAYLOAD       = 0x25,
	FIRMWARE_REQUEST_PAYLOAD 	   = 0x0a,
	FIRMWARE_RESPONSE_PAYLOAD 	   = 0x0b,
	GPIO_CONTROL_PAYLOAD           = 0x2c,
	GPIO_REQUEST_PAYLOAD           = 0x30,
	GPIO_RESPONSE_PAYLOAD          = 0x2d,
	HADC_CONTROL_PAYLOAD		   = 0x16,
	HADC_REQUEST_PAYLOAD		   = 0x17,
	HADC_RESPONSE_PAYLOAD		   = 0x18,
	ID_PAYLOAD 			  		   = 0x00,
	IMU_RESPONSE_PAYLOAD		   = 0x19,
	IO_CONTROL_PAYLOAD 			   = 0x04,
	IO_REQUEST_PAYLOAD 			   = 0x05,
	IO_RESPONSE_PAYLOAD 		   = 0x06,
	NACK_PAYLOAD 				   = 0x03,
	PERIPHERAL_CONFIG_PAYLOAD      = 0x35,
	PORT_EXPANDER_PAYLOAD          = 0x32,
	PWM_CONTROL_PAYLOAD			   = 0x11,
	PWM_RESPONSE_PAYLOAD		   = 0x12,
	REGISTER_CONFIG_PAYLOAD        = 0x38,
	REGISTER_RESPONSE_PAYLOAD      = 0x39,
	SOFT_PWM_CONFIG_PAYLOAD        = 0x33,
	SOFT_PWM_RESPONSE_PAYLOAD      = 0x34,
	STEPPER_CONFIG_PAYLOAD 		   = 0x07,
	STEPPER_CONTROL_PAYLOAD 	   = 0x08,
	STEPPER_RESPONSE_PAYLOAD 	   = 0x09,
	STREAMING_CONFIG_PAYLOAD       = 0x36,
	STREAMING_RESPONSE_PAYLOAD     = 0x37,
	TEMPERATURE_CONFIG_PAYLOAD	   = 0x1a,
	TEMPERATURE_REQUEST_PAYLOAD	   = 0x22,
	TEMPERATURE_RESPONSE_PAYLOAD   = 0x1b,
	THERMAL_CONTROL_PAYLOAD		   = 0x0e,
	THERMAL_REQUEST_PAYLOAD		   = 0x10,
	THERMAL_RESPONSE_PAYLOAD	   = 0x0f,
	TIME_PAYLOAD 				   = 0x01,
	TRAJECTORY_PAYLOAD             = 0x20,
	TRAJECTORY_RESPONSE_PAYLOAD	   = 0x21,


} PayloadType;

typedef uint32_t Payload;

PayloadType getPayloadType(Payload* payload);
void setPayloadType(Payload* payload, PayloadType type);
uint32_t getPayloadLength(Payload* payload);
void setPayloadLength(Payload* payload, int32_t length);
uint32_t getPayloadWord(Payload* payload, uint8_t index);
void setPayloadWord(Payload* payload, uint8_t index, uint32_t value);
float getPayloadFloat(Payload* payload, uint8_t index);
void setPayloadFloat(Payload* payload, uint8_t index, float value);


#endif /* INC_PAYLOAD_H_ */
