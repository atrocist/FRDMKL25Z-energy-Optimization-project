#ifndef TIMERS_H
#define TIMERS_H
#include "MKL25Z4.h"

#define TPM_Cn_MODE        (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK)
#define TPM_MODULE         1000
#define TPM_INIT_VAL       0

#define SET_LED_GREEN(x)   TPM2->CONTROLS[1].CnV = (x)
#define SET_LED_RED(x)     TPM2->CONTROLS[0].CnV = (x)

#define RAD_15 0.261799f
#define RAD_30 0.523598f
#define RAD_N15 -0.261799f
#define RAD_N30 -0.523598f

#define TAN_15 0.26794919f
#define TAN_N15 -0.26794919f 
#define TAN_30 0.57735026f
#define TAN_N30 -0.57735026f

void Init_PWM(void);
void Set_PWM_Values(uint16_t perc1, uint16_t perc2);

void Init_LPTMR(unsigned int ticks);
void Start_LPTMR(void);
void Stop_LPTMR(void);


extern volatile unsigned PIT_interrupt_counter;
extern volatile unsigned LCD_update_requested;

#endif
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
