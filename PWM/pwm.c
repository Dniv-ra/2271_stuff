#include "MKL25Z4.h"

#define MASK(x) (1 << (x))


int calc_mod(int clkspd, int prescaler, int req_freq) {
	int cycles_per_sec = clkspd / prescaler;
	return (cycles_per_sec / req_freq) - 1;
}

int calc_cnv(int mod, float req_duty) {
	return (mod * req_duty);
}


void initPWM() {
	// Enable clock for Port B and set pin as GPIO
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	//Change from 5 to something meaningful
	PORTB->PCR[5] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[5] = PORT_PCR_MUX(3); //Also the same as MASK(8) (should test, if using just mask need clear bits)
	
	//CLOCK SETUP
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; //Clearing the bits in TPMSRC
	//Use MCGFLLCLK CLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //Same as MASK(24)
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // Prescaler = 2^7 = 128
	TPM1->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM1_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
	TPM1_C0SC |= (TPM_CnSC_ELSB_MASK | TPM_CnSC_MSB_MASK);
	
	TPM1_MOD = calc_mod(48000000, 128, 50);
	TPM1_C0V = calc_cnv(TPM1_MOD, 0.5);
}


static void delay(volatile uint32_t duration)
{
  while (duration != 0) 
  {
    __asm("NOP");
     duration--;
  }
}

int main(void) {

	while(1) {
	}
}

