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
#include <stdbool.h>

#include "bootloader.h"
#include "fastcodeUtil.h"
#include "stm32f4xx.h"


//*************************************************
//defines
//*************************************************

//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************

static uint32_t bootloaderTimerReg = 0;
static bool triggered = false;
//*************************************************
//function prototypes
//*************************************************
void jumpToBootloader(void);

//*************************************************
//code
//*************************************************

bool isBootloaderTriggered(void){
	return triggered;
}

/**
 * trigger a transfer of command to the STM32F446 bootloader
 * The actual transfer will occur after a second
 */
bool triggerBootloader(void){
	static uint32_t lastTimeReg = 0;
	uint32_t t = getTimeInMicroSeconds();

	//reset the trigger if this hasn't been called within a second or never at all
	if(compareTimeMicroSec(t, lastTimeReg) > 1.0f || lastTimeReg == 0){
		bootloaderTimerReg = t;
		triggered = false;
	}
	lastTimeReg = t;

	//if our timer is more than a second old then trigger the bootloader
	if(compareTimeMicroSec(t, bootloaderTimerReg) > 1.0f){
		bootloaderTimerReg = t;
		triggered = true;
	}
	return triggered;

}

void bootloaderSlowCode(void){
	if(triggered){
		if(compareTimeToNow(bootloaderTimerReg) > 2.0f){
			jumpToBootloader();
			triggered = false;
		}
	}
}


/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 * This was copied from code found here:
 * https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
 * It's made for an STM32F429 but I believe it's the same for an STM32F446
 */
void jumpToBootloader(void) {
    void (*SysMemBootJump)(void);

    /**
     * Step: Set system memory address.
     *
     *       For STM32F429, system memory is on 0x1FFF 0000
     *       For other families, check AN2606 document table 110 with descriptions of memory addresses
     */
    volatile uint32_t addr = 0x1FFF0000;

    /**
     * Step: Disable RCC, set it to default (after reset) settings
     *       Internal clock, no PLL, etc.
     */

    RCC_DeInit();

    //disable peripheral clocks
    RCC->AHB1ENR = 0;
	RCC->AHB2ENR = 0;
	RCC->AHB3ENR = 0;
	RCC->APB1ENR = 0;
	RCC->APB2ENR = 0;



    /**
     * Step: Disable systick timer and reset it to default values
     */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;


    //make sure WDT can't fire
    IWDG_ReloadCounter();

    //set watchdog time
	// blocking code - possibly not optimal, however it is only used at program startup, plus the status bits are almost certainly clear at program startup
	//must be sure the prescaler and value are not being used prior to changing them, otherwise they cannot be changed
	//should take up to 294us at most (5 cycles at min clock of 17kHz).
	while (IWDG->SR & (IWDG_FLAG_PVU | IWDG_FLAG_RVU)){
		// must ensure that (wait until) these flag bits are cleared before we can write to the following registers
	}
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    //watchdog input is between 17 and 47 kHz. Typical is 32
	IWDG_SetPrescaler(IWDG_Prescaler_256);//this divides the clock down to 125Hz typical. Timeout is therefore multiple of 8ms


	IWDG_SetReload((uint16_t)4095);//cannot be bigger than 0x0fff. 125 gives delay of 1s.
	IWDG_Enable();



    /**
     * Step: Disable all interrupts
     */
    __disable_irq();

    /**
     * Step: Remap system memory to address 0x0000 0000 in address space
     *       For each family registers may be different.
     *       Check reference manual for each family.
     *
     *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
     *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
     *       For others, check family reference manual
     */
    //Remap by hand... {
#if defined(STM32F4)
    SYSCFG->MEMRMP = 0x01;
#endif
#if defined(STM32F0)
    SYSCFG->CFGR1 = 0x01;
#endif
    //} ...or if you use HAL drivers
    //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();    //Call HAL macro to do this for you

    /**
     * Step: Set jump memory location for system memory
     *       Use address with 4 bytes offset which specifies jump location where program starts
     */
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

    /**
     * Step: Set main stack pointer.
     *       This step must be done last otherwise local variables in this function
     *       don't have proper value since stack pointer is located on different position
     *
     *       Set direct address location which specifies stack pointer in SRAM location
     */
    __set_MSP(*(uint32_t *)addr);

    /**
     * Step: Actually call our function to jump to set location
     *       This will start system memory execution
     */
    SysMemBootJump();

    /**
     * Step: Connect USB<->UART converter to dedicated USART pins and test
     *       and test with bootloader works with STM32 Flash Loader Demonstrator software
     */
}




