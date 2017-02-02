/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
//#include "gpio_defs.h"
//#include "LEDs.h"
#include "timers.h"
//#include "delay.h"
//#include "ADC.h"
#include "config.h"

#include "my_math.h"
#include "approx.h"
#include "mma8451.h"
#include "i2c.h"

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

		i2c_init();
		init_mma();		
		// Allow low leakage stop mode
		SMC->PMPROT = SMC_PMPROT_ALLS_MASK; // 
		//SMC->PMPROT = SMC_PMPROT_AVLLS_MASK;
		// Enable low-leakage stop mode and regular run mode
		SMC->PMCTRL = SMC_PMCTRL_STOPM(3) | SMC_PMCTRL_RUNM(0);
		//SMC->PMCTRL = SMC_PMCTRL_STOPM(4) | SMC_PMCTRL_RUNM(0);
		SMC->STOPCTRL = SMC_STOPCTRL_PSTOPO(0) | SMC_STOPCTRL_VLLSM(3);
		//SMC->STOPCTRL = SMC_STOPCTRL_PSTOPO(0) | SMC_STOPCTRL_VLLSM(3);
	
		// Enable LLWU
		// allow LPTMR0 to wake LLWU
		LLWU->ME |= LLWU_ME_WUME0_MASK;
	
	// Enable stop mode (deep sleep)
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	
		Init_LPTMR(50);
		Start_LPTMR();
		Init_PWM();	
		SIM->SOPT1 &= ~SIM_SOPT1_USBREGEN_MASK;
		SIM->SCGC4 &= ~(SIM_SCGC4_I2C1_MASK|
										SIM_SCGC4_UART0_MASK|
										SIM_SCGC4_UART1_MASK|
										SIM_SCGC4_UART2_MASK|
										SIM_SCGC4_SPI0_MASK|
										SIM_SCGC4_USBOTG_MASK|
										SIM_SCGC4_CMP_MASK|
										SIM_SCGC4_SPI1_MASK
										);
		SIM->SCGC5 &= ~(SIM_SCGC5_TSI_MASK|
										SIM_SCGC5_PORTA_MASK|
								//		SIM_SCGC5_PORTB_MASK|
										SIM_SCGC5_PORTC_MASK|
										SIM_SCGC5_PORTD_MASK//|
								//		SIM_SCGC5_PORTE_MASK
										);
		SIM->SCGC6 &= ~(
										SIM_SCGC6_DMAMUX_MASK|
										SIM_SCGC6_PIT_MASK|
										SIM_SCGC6_TPM0_MASK|
										SIM_SCGC6_TPM1_MASK|
										SIM_SCGC6_ADC0_MASK|
										SIM_SCGC6_RTC_MASK|
										SIM_SCGC6_DAC0_MASK
										);
										
		SIM->SCGC7 &= ~SIM_SCGC7_DMA_MASK;
		
		__enable_irq();
		//put unwanted peripherals to sleep
		// work is in interrupt
		while (1) {
	//		PTE->PCOR |= MASK(DEBUG_RUNNING_POS);
//#if USE_SLEEP_MODES
			__wfi() ; // then go to sleep	
//#endif
	//		PTE->PSOR |= MASK(DEBUG_RUNNING_POS);
		}
}

// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
