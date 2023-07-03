/*
Copyright (c) 2020 STARFISH PRODUCT ENGINEERING INC.

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
#include <powerDown.h>
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rtc.h"
#include <stm32f4xx_iwdg.h>



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






//*************************************************
//code
//*************************************************



void gotoLowPowerMode(double wakeupDelaySeconds, bool stopNotStandby, bool pin1Enable, bool pin2Enable){
	// must enable the power peripheral
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//this should already be enabled


	//disable systick interrupt interrupts
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

	//reset watchdog prior to messing with it. This will likely cause the prescaler and value to be loaded so will likely cause following while loop to block for a bit.
	IWDG_ReloadCounter();

	//set watchdog time
	// blocking code - possibly not optimal, however it is only used at program startup, plus the status bits are almost certainly clear at program startup
	//must be sure the prescaler and value are not being used prior to changing them, otherwise they cannot be changed
	//should take up to 294us at most (5 cycles at min clock of 17kHz).
	while (IWDG->SR & (IWDG_FLAG_PVU | IWDG_FLAG_RVU)){
		// must ensure that (wait until) these flag bits are cleared before we can write to the following registers
	}
	//enable writing to watchdog config registers
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	//watchdog input is between 17 and 47 kHz. Typical is 32
	IWDG_SetPrescaler(IWDG_Prescaler_256);//this divides the clock down to 125Hz typical. Timeout is therefore multiple of 8ms
	double d = wakeupDelaySeconds * 125;
	if(d > 4095){
		d = 4095;
	} else if(d < 1){
		d = 1;
	}


	IWDG_SetReload((uint16_t)d);//cannot be bigger than 0x0fff. 125 gives delay of 1s.
	IWDG_Enable();

	//debugging
	PWR_WakeUpPinCmd(PWR_WakeUp_Pin1, DISABLE);
	PWR_WakeUpPinCmd(PWR_WakeUp_Pin2, DISABLE);

	uint32_t temp = PWR_CR_PDDS;

	// configure the Wakeup pin input
	PWR->CR |= (PWR_CR_CSBF | PWR_CR_CWUF);                      // Clear the standby flag & wakeup flag
	PWR_WakeUpPinCmd(PWR_WakeUp_Pin1, pin1Enable ? ENABLE : DISABLE);                                    // enable the WKUP pin
	PWR_WakeUpPinCmd(PWR_WakeUp_Pin2, pin2Enable ? ENABLE : DISABLE);

	//in order to allow pin2 to work (c13) the following should have happened
//	PWR_BackupAccessCmd(ENABLE);
//	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
//	RCC_RTCCLKCmd(ENABLE);
//	RTC_TamperCmd(RTC_Tamper_1, ENABLE);


	PWR->CR |= (PWR_CR_CSBF | PWR_CR_CWUF);                      // Clear the standby flag & wakeup flag
	if(stopNotStandby){
		PWR_EnterSTOPMode(PWR_MainRegulator_ON, PWR_STOPEntry_WFI);
	} else {
		PWR_EnterSTANDBYMode();
	}
}
bool wasWakeupFromEvent(void){
	return PWR_GetFlagStatus(PWR_FLAG_WU);
}
