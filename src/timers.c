#include "timers.h"
#include "MKL25Z4.h"
//#include "ADC.h"
#include "LEDs.h"
#include "GPIO_defs.h"
#include "math.h"
#include "mma8451.h"
#include "approx.h"


void Init_LPTMR(unsigned int ticks) {
	SIM->SCGC5 |=  SIM_SCGC5_LPTMR_MASK;

	// Configure LPTMR
	// select 1 kHz LPO clock with prescale factor 0, dividing clock by 2
	// resulting in 500 Hz clock
	LPTMR0->PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PRESCALE(0); 
	LPTMR0->CSR = LPTMR_CSR_TIE_MASK;
	LPTMR0->CMR = ticks; // Generate interrupt every 50 clock ticks or 100 ms

	// Configure NVIC 
	NVIC_SetPriority(LPTimer_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(LPTimer_IRQn); 
	NVIC_EnableIRQ(LPTimer_IRQn);	

}

void Start_LPTMR(void) {
	LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;
}

void Stop_LPTMR(void) {
	LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK;
}

void LPTimer_IRQHandler(void) {
	static uint8_t n=LED_PERIOD;
	float roll, pitch;
	
	PTE->PSOR |= MASK(DEBUG_RUNNING_POS);
	NVIC_ClearPendingIRQ(LPTimer_IRQn);
	LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;

	//read accelerometer
	read_full_xyz();
	
	if (n==1) {
		//set lptmr to 10ms
		//Control_RGB_LEDs(1, 1, 1);	
		//LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK;	
		
		roll = (float)acc_Y/(float)acc_Z;
		pitch= (float)acc_X/(float)approx_sqrtf(acc_Y*acc_Y + acc_Z*acc_Z);
		
		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
		SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
		SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK);
		if(roll >= RAD_30 || pitch >= TAN_30 || roll <= RAD_N30 || pitch <= TAN_N30 )
		{ 
			SET_LED_RED(50);//Control_RGB_LEDs(1, 0, 0);
		}
		else if(roll >= RAD_15 || pitch >= TAN_15 || roll <= RAD_N15 || pitch <= TAN_N15)
		{ 
			SET_LED_GREEN(50);//Control_RGB_LEDs(1, 1, 0);
			SET_LED_RED(50);
		}
		else //if(roll_rad < RAD_15 || pitch_rad < RAD_15)
		{ 
			SET_LED_GREEN(50);//Control_RGB_LEDs(0, 1, 0);
		}
		SIM->SCGC5 &= ~SIM_SCGC5_PORTB_MASK;
		SIM->SCGC6 &= ~SIM_SCGC6_TPM2_MASK;
		SIM->SCGC5 &= ~SIM_SCGC5_PORTE_MASK;
		LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;
		LPTMR0->CMR = 5;			
	} 
	
		//Control_RGB_LEDs(0,0,0);
		//reset LPTMR to 100ms
	if(n==0)
	{
		//Control_RGB_LEDs(1, 1, 0);
		//LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK;
		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
		SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
		SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK);
		SET_LED_GREEN(0);
		SET_LED_RED(0);
		LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;
		LPTMR0->CMR = 50;
		n = LED_PERIOD;
		SIM->SCGC5 &= ~SIM_SCGC5_PORTB_MASK;
		SIM->SCGC6 &= ~SIM_SCGC6_TPM2_MASK;
		SIM->SCGC5 &= ~SIM_SCGC5_PORTE_MASK;
		//Control_RGB_LEDs(0, 0, 0);	
	}
	n--;
}

void Init_PWM(void)
{
  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;// | SIM_SCGC5_PORTD_MASK;
  SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
  SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1)| SIM_SOPT2_PLLFLLSEL_MASK);

  PORTB->PCR[18] = (0 | PORT_PCR_MUX(3)); // TPM2_CH0 enable on PTB18 
  PORTB->PCR[19] = (0 | PORT_PCR_MUX(3)); // TPM2_CH1 enable on PTB19 
 

  TPM2->MOD  = TPM_MODULE;   // 0x0063 / 25MHz = 4uS PWM period
	TPM2->CONTROLS[0].CnSC = TPM_Cn_MODE;  // No Interrupts; Low True pulses on Edge Aligned PWM
	TPM2->CONTROLS[1].CnSC = TPM_Cn_MODE;  // No Interrupts; Low True pulses on Edge Aligned PWM
	
	TPM2->CONTROLS[0].CnV = TPM_INIT_VAL;
	TPM2->CONTROLS[1].CnV = TPM_INIT_VAL; // 90% pulse width
  TPM2->SC   = TPM_SC_CMOD(1) | TPM_SC_PS(0);  // Edge Aligned PWM running from BUSCLK / 1 
}


// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
