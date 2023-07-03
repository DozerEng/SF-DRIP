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

//*************************************************
//includes
//*************************************************
#include <stdint.h>
#include "_processorGlobal.h"


//*************************************************
//defines
//*************************************************


//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************
static uint32_t m_appEdmsId = 39788;//this is the EDMS ID of the SFDQ Firmware
static uint32_t m_appVersion = 0;



//*************************************************
//function prototypes
//*************************************************


//*************************************************
//code
//*************************************************




/**
 * sets the version of this app. This is zero for a SFDQ.
 * An app based on the SFDQ code will set this so it is identified properly
 * @param edmsId this is the id for this firmware from the StarFish electronic document management system.
 * @version this is a 32 bit unsigned that conveys the version. The format of this number is up to the specific app.
 *
 */
void setAppVersion(uint32_t edmsId, uint32_t version){
	m_appEdmsId = edmsId;
	m_appVersion = version;
}
/**
 * @return the app version.
 */
uint32_t getAppVersion(void){
	return m_appVersion;
}
/**
 * @return the EDMS number of the app. This will be 39788 if it's just an SFDQ
 */
uint32_t getAppEdmsId(void){
	return m_appEdmsId;
}
/**
 * @return the version of the SFDQ source code that was used to make this app
 */
uint32_t getSfdqVersion(void){
	return SFDQ_FW_VERSION;
}
