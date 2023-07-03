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

#ifndef INC_BRUSHLESSMOTOR_H_
#define INC_BRUSHLESSMOTOR_H_

#include <stdint.h>
#include <ports.h>
#include <sinkSource.h>

/**
 * inits this module. Must be run at start
 */
void initBrushlessMotor(void);

/**
 * setups up a motor. This will use high power output channels 0, 1, 2
 * @param phaseCyclesPerRev indicates how many times the windings see a full phase cycle, per revolution of the motor shaft. If zero then the motor is considered disabled.
 */
void setupBrushlessMotor(PortPin sensorA, PortPin sensorB, PortPin sensorC, float phaseCyclesPerRev);

/**
 * @param winding phase the phase we're interested in. Valid values: 0, 1, 2.
 * @return the drive value for the specified phase
 */
float getBrushlessPhaseValue(uint8_t index);
/**
 * computes the present phase of the motor based on the sensor inputs
 */
void brushlessFastCode(void);

/**
 * sets the present drive. Negative values reverse direction.
 */
void setBrushlessDrive(float drive);
float getBrushlessDrive(void);
float getBrushlessAngle(void);
float getBrushlessRevPerSec(void);
float getBrushlessPosition(void);
/**
 * sets the phase offset and phase advance coefficient for the motor
 * @param offset the phase adjustment in degrees to offset the phase from the hall sensors
 * @param advance the amount to advance the phase per speed in degrees per Hz.
 */
void setBrushlessPhaseOffset(float offset, float advance);

/**
 * Sets some sinks for useful brushless motor info
 * @param index 0,1,2,3 = position, velocity, output phase, sensor phase.
 * @param sink the sink
 *
 */
void setBrushlessSink(uint32_t index, SinkSource sink);

/**
 * gets some sinks for useful brushless motor info
 * @param index 0,1,2,3 = position, velocity, output phase, sensor phase.
 * @return sink the sink
 *
 */
SinkSource getBrushlessSink(uint32_t index);

#endif /* INC_BRUSHLESSMOTOR_H_ */
