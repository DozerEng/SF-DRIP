/*
Copyright (c) 2022 STARFISH PRODUCT ENGINEERING INC.

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

#include <clockSetup.h>
#include <stm32f4xx_iwdg.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx.h>
#include <math.h>
#include <stm32f4xx_rcc.h>





//*************************************************
//Defines
//*************************************************
//Clock config stuff
#define HSE_VALUE_HZ (20.0e6)
#define CLOCK_FREQ_HZ 180.0e6
//#define VCO_FREQ_MHZ (CLOCK_FREQ_MHZ*2)
#define PLL_INPUT_FREQ_HZ 1.0e6
//PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
//M valid range 2 to 63 inclusive
//N valid range 50 to 432 inclusive
//#define PLL_M      (HSE_VALUE_MHZ/PLL_INPUT_FREQ_MHZ) //ideally this is the value of the HSE in MHz. That gives a 1 MHz PLL input
//#define PLL_N      VCO_FREQ_MHZ //This sets the VCO clock to 360 MHz. Valid values can yield 100 MHz to 432 MHz

//USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ. this can be 2 to 15 inclusive
//#define PLL_Q      8 //this gives 48 MHz, which is what it should be
//PLL division factor for I2S, SAI, SYSTEM and SPDIF: Clock =  PLL_VCO / PLLR. This can be 2 to 7 inclusive
//#define PLL_R      7 //this is also 51.4 MHz and it really would rather be 48 MHz
//SYSCLK = PLL_VCO / PLL_P. This can be 2, 4, 6 or 8
//#define PLL_P      (VCO_FREQ_MHZ/CLOCK_FREQ_MHZ) //this sets the main PLL output to /2 yielding 180 MHz

#define VECT_TAB_OFFSET  0x00 //!< Vector Table base offset field.

//*************************************************
//Types
//*************************************************


//*************************************************
//Variables
//*************************************************
uint32_t m_systemCoreClock = 180000000;
static uint32_t m_mainClock = 180000000;
static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
static float m_hseClockCheck;
static float m_roundedHse;

//*************************************************
//function prototypes
//*************************************************
static void internalSetupSystemClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q, uint32_t r);
/**
 * while HSI is still running, do a quick check on the HSE frequency
 * @return frequency in Hz
 */
static float checkHseFrequency(void);
static float startupHse();

//*************************************************
//Code
//*************************************************


void initSystem(void){
	/* FPU settings ------------------------------------------------------------*/
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */

	/* Reset the RCC clock configuration to the default reset state ------------*/
	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;



	/* Configure the Vector Table location add offset address ------------------*/
//	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
}


/**
 * configure the RCC reg according to the specified, desired HSE and main clock frequencies
 * @return hse frequency in MHz.
 */
float setupSystemClock(uint32_t mainMHz){




//	m = 20;
//	n = 360;
//	q = 8;
//	r = 7;
//	p = 2;
	float result = NAN;
	float hse = startupHse();

	if(finitef(hse)){

		uint32_t p = 2;//can be 2,4,6,8
		float vcoOut = mainMHz * p;//this will likely be 360 MHz

		uint32_t m;
		uint32_t n;
		if(fabsf(hse - 14.7456e6f) < 100e3){
			//this is a special case
			n = 293;
			m = 12;
			result = 14.7456e6f;
			//pll frequency of 1.23 MHz
		} else {
			m = (uint32_t)roundf(hse/PLL_INPUT_FREQ_HZ);//2 to 63
			n = mainMHz*p;//can be 50 to 432
			result = m * 1e6f;
		}





		uint32_t q = vcoOut / 48;//2 to 15
		uint32_t r = vcoOut / 48;//2 to 7

		if(r > 7){
			r = 7;
		}
		if(q > 15){
			q = 15;
		}




		m_mainClock = mainMHz*1000000;
		internalSetupSystemClock(m, n, p, q, r);
	} else {
		//TODO: the hse has not started
	}
	m_roundedHse = result;
	return result;

}


static float startupHse(){
	float result = NAN;

	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do {
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET){
		HSEStatus = (uint32_t)0x01;
	} else {
		HSEStatus = (uint32_t)0x00;
	}

	if (HSEStatus == (uint32_t)0x01) {

		result = checkHseFrequency();
	}
	return result;
}
static void internalSetupSystemClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q, uint32_t r){

	/* Select regulator voltage output Scale 1 mode */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	/* HCLK = SYSCLK / 1*/
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/* PCLK2 = HCLK / 2*/
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

	/* PCLK1 = HCLK / 4*/
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	// configure RTCPRE for max devision (this is to aide HSE measurement)
	RCC->CFGR |=  RCC_CFGR_RTCPRE | RCC_CFGR_RTCPRE_0 | RCC_CFGR_RTCPRE_1 | RCC_CFGR_RTCPRE_2 | RCC_CFGR_RTCPRE_3 | RCC_CFGR_RTCPRE_4;


	/* Configure the main PLL */
	RCC->PLLCFGR = m | (n << 6) | (((p >> 1) -1) << 16) |
				   (RCC_PLLCFGR_PLLSRC_HSE) | (q << 24) | (r << 28);

	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0){
	}

	/* Enable the Over-drive to extend the clock frequency to 180 Mhz */
	PWR->CR |= PWR_CR_ODEN;
	while((PWR->CSR & PWR_CSR_ODRDY) == 0){
	}
	PWR->CR |= PWR_CR_ODSWEN;
	while((PWR->CSR & PWR_CSR_ODSWRDY) == 0){
	}
	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

	/* Select the main PLL as system clock source */
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL){
	}

}
/**
 * computes the system core clock according to the actual setup of the RCC reg
 * Not sure if this is really adding value
 */
void systemCoreClockUpdate(void){
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  uint32_t pllr = 2;
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp){
    case 0x00:  /* HSI used as system clock source */
      m_systemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      m_systemCoreClock = (uint32_t)HSE_VALUE_HZ;
      break;
    case 0x08:  /* PLL P used as system clock source */
       /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

      if (pllsource != 0){
        /* HSE used as PLL clock source */
        pllvco = (((uint32_t)HSE_VALUE_HZ) / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      } else {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      if (pllsource == 0){
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      m_systemCoreClock = pllvco/pllp;
      break;
      case 0x0C:  /* PLL R used as system clock source */
       /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_R
         */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      if (pllsource != 0){
        /* HSE used as PLL clock source */
        pllvco = (((uint32_t)HSE_VALUE_HZ) / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      } else {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >>28) + 1 ) *2;
      m_systemCoreClock = pllvco/pllr;
      break;
    default:
      m_systemCoreClock = HSI_VALUE;
      break;
	}
	/* Compute HCLK frequency --------------------------------------------------*/
	/* Get HCLK prescaler */
	//  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
	int32_t ahbPrescale = ((RCC->CFGR & RCC_CFGR_HPRE) >> 4) - 7;

	ahbPrescale = ahbPrescale < 0 ? 0 : ahbPrescale;




  /* HCLK frequency */
  m_systemCoreClock >>= ahbPrescale;
}
float getRoundedHse(void){
	return m_roundedHse;
}
float getSystemClockHz(void){
	return m_mainClock;
}

float getApb1ClockHz(void){
	float result = NAN;
	uint32_t tmp;
	uint32_t presc;
	/* Get HCLK prescaler */
	tmp = RCC->CFGR & RCC_CFGR_HPRE;
	tmp = tmp >> 4;
	presc = APBAHBPrescTable[tmp];
	/* HCLK clock frequency */
	uint32_t hclk = ((uint32_t)m_mainClock) >> presc;

	/* Get PCLK1 prescaler */
	tmp = RCC->CFGR & RCC_CFGR_PPRE1;
	tmp = tmp >> 10;
	presc = APBAHBPrescTable[tmp];
	/* PCLK1 clock frequency */
	result = (float)(hclk >> presc);
	return result;
}
float getApb2ClockHz(void){
	float result = NAN;
	uint32_t tmp;
	uint32_t presc;
	/* Get HCLK prescaler */
	tmp = RCC->CFGR & RCC_CFGR_HPRE;
	tmp = tmp >> 4;
	presc = APBAHBPrescTable[tmp];
	/* HCLK clock frequency */
	uint32_t hclk = ((uint32_t)m_mainClock) >> presc;

	/* Get PCLK2 prescaler */
	tmp = RCC->CFGR & RCC_CFGR_PPRE2;
	tmp = tmp >> 13;
	presc = APBAHBPrescTable[tmp];
	/* PCLK1 clock frequency */
	result = (float)(hclk >> presc);
	return result;
}
/**
 * turn on a bunch of peripheral clocks.
 * Also setup the watchdog timer
 */
/**
 * turn on a bunch of peripheral clocks.
 * Also setup the watchdog timer
 */
void enablePeripheralClocks(void) {
	//turn on clocks for GPIO and peripherals
	uint32_t mask = 0;
	mask |= RCC_AHB1Periph_GPIOA;
	mask |= RCC_AHB1Periph_GPIOB;
	mask |= RCC_AHB1Periph_GPIOC;
	mask |= RCC_AHB1Periph_GPIOD;
	mask |= RCC_AHB1Periph_DMA1;
	mask |= RCC_AHB1Periph_DMA2;
	RCC_AHB1PeriphClockCmd(mask, ENABLE);
	mask = 0;
	mask |= RCC_APB1Periph_PWR;
	mask |= RCC_APB1Periph_WWDG;
	mask |= RCC_APB1Periph_TIM3;
	mask |= RCC_APB1Periph_TIM3;
	mask |= RCC_APB1Periph_TIM2;
	mask |= RCC_APB1Periph_TIM7;
	mask |= RCC_APB1Periph_SPI2;
	mask |= RCC_APB1Periph_SPI3;
	mask |= RCC_APB1Periph_DAC;
	mask |= RCC_APB1Periph_I2C1;
	mask |= RCC_APB1Periph_I2C2;
	mask |= RCC_APB1Periph_I2C3;
	mask |= RCC_APB1Periph_USART3;
	RCC_APB1PeriphClockCmd(mask, ENABLE);
	mask = 0;
	mask |= RCC_APB2Periph_ADC1;
	mask |= RCC_APB2Periph_SYSCFG;
	mask |= RCC_APB2Periph_USART1;
	mask |= RCC_APB2Periph_TIM8;
	mask |= RCC_APB2Periph_USART6;
	mask |= RCC_APB2Periph_TIM11;
	RCC_APB2PeriphClockCmd(mask, ENABLE);
	// blocking code - possibly not optimal, however it is only used at program startup, plus the status bits are almost certainly clear at program startup
	while (IWDG->SR & (IWDG_FLAG_PVU | IWDG_FLAG_RVU)) {
		// must ensure that (wait until) these flag bits are cleared before we can write to the following registers
	}
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	//IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetReload(32); //cannot be bigger than 0x0fff. 16 sets timeout to 128ms. (32 sets to 256ms)
	//IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
	IWDG_Enable();
}

/**
 * while HSI is still running, do a quick check on the HSE frequency
 * @return frequency in Hz
 */
static float checkHseFrequency(void){
	//TODO: add this
	//use TMR11 with HSE_RTC as input (while CPU running on HSI)

	//RCC_CFGR should already be configured for maximum RTC prescale from HSE (RTCPRE = 31)
	//enable timer
	RCC->APB2ENR |= RCC_APB2Periph_TIM11;
	RCC->CFGR |= RCC_RTCCLKSource_HSE_Div31;
	//first setup timer
	TIM11->CR1 = 0x0000;//this disables the timer. Can't be setup until it's off
	TIM11->CCMR1 = (TIM_CCMR1_IC1PSC_0 | TIM_CCMR1_IC1PSC_1 | TIM_CCMR1_CC1S_0);
	TIM11->OR = TIM11_HSE;//this sets CH1 as HSE_RTC
	TIM11->CCER = TIM_CCER_CC1E;
	TIM11->CCR1 = 0x0000;//zero this for now
	TIM11->SR = 0;//clear status flags
	//now start it!
	TIM11->CR1 = 0x0001;//this disables the timer. Can't be setup until it's off
	//use it to capture based on RTC_PRE input, use as much prescale as possible on this input
	//this should give us another 8x I think
	while((TIM11->SR & TIM_FLAG_CC1) == 0){
		//kill time until a double overflow
	}

	uint32_t t0 = TIM11->CCR1;

	while((TIM11->SR & TIM_FLAG_CC1) == 0){
		//kill time until a double overflow
	}
	uint32_t t1 = TIM11->CCR1;


	m_hseClockCheck = (float)((t1 - t0) & 0xffff);

	m_hseClockCheck *= 1.0f/31.0f/8.0f;//this should undo the prescalers
	m_hseClockCheck = 16e6/m_hseClockCheck;

	return (float)m_hseClockCheck;
}

