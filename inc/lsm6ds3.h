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

#ifndef LSM6DS3_H_
#define LSM6DS3_H_

/**
 * must be run in main loop more than 10 times per second
 */
void lsm6ds3SlowCode(float seconds);
/**
 * @return acceleration component along X axis in metres per second squared
 */
float getLsm6ds3AccX(void);
/**
 * @return acceleration component along Y axis in metres per second squared
 */
float getLsm6ds3AccY(void);
/**
 * @return acceleration component along Z axis in metres per second squared
 */
float getLsm6ds3AccZ(void);
/**
 * @return angular velocity around X axis in radians per second
 */
float getLsm6ds3GyroX(void);
/**
 * @return angular velocity around Y axis in radians per second
 */
float getLsm6ds3GyroY(void);
/**
 * @return angular velocity around Z axis in radians per second
 */
float getLsm6ds3GyroZ(void);


#endif /* LSM6DS3_H_ */
