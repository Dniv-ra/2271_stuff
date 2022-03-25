#include "MKL25Z4.h"

#define MASK(x) (1 << x)
#define UART2_TX_PIN 22
#define UART2_RX_PIN 23

/* UART */
void initUART(uint32_t baud) 
{
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART2_TX_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART2_TX_PIN] |= ~PORT_PCR_MUX(4);
	
	PORTE->PCR[UART2_RX_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART2_RX_PIN] |= ~PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
	divisor = bus_clock / (baud * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->C3 = 0;
	UART2->S2 = 0;
	//Add the two interrupt stuff
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) |(UART_C2_RIE_MASK));
	
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_SetPriority(UART2_IRQn, 2);
	NVIC_EnableIRQ(UART2_IRQn);
}

/* PWM */
int calc_mod(int clkspd, int prescaler, int req_freq) {
	int cycles_per_sec = clkspd / prescaler;
	return (cycles_per_sec / req_freq) - 1;
}

int calc_cnv(int mod, float req_duty) {
	return (mod * req_duty);
}

void initPWM() {
	// Enable clock for Port B and set pin as GPIO
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	//Change from 5 to something meaningful
	PORTA->PCR[12] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[12] = PORT_PCR_MUX(3); 
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	//CLOCK SETUP
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; //Clearing the bits in TPMSRC
	//Use MCGFLLCLK CLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // Prescaler = 2^7 = 128
	TPM1->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM1_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM1_MOD = calc_mod(48000000, 128, 262);
	TPM1_C0V = calc_cnv(TPM1_MOD, 0.5);
}

/* GPIO */
void initGPIOA(int PIN_NO)
{
 // Enable Clock to PORTB and PORTD
 SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK);
 // Configure MUX settings to make all 3 pins GPIO
 PORTA->PCR[PIN_NO] &= ~PORT_PCR_MUX_MASK;
 PORTA->PCR[PIN_NO] |= PORT_PCR_MUX(1);
  
 // Set Data Direction Registers for PortB and PortD
 PTA->PDDR |= MASK(PIN_NO);
}

void initGPIOB(int PIN_NO)
{
 // Enable Clock to PORTB and PORTD
 SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK);
 // Configure MUX settings to make all 3 pins GPIO
 PORTB->PCR[PIN_NO] &= ~PORT_PCR_MUX_MASK;
 PORTB->PCR[PIN_NO] |= PORT_PCR_MUX(1);
  
 // Set Data Direction Registers for PortB and PortD
 PTB->PDDR |= MASK(PIN_NO);
}

void initGPIOC(int PIN_NO)
{
 // Enable Clock to PORTB and PORTD
 SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK);
 // Configure MUX settings to make all 3 pins GPIO
 PORTC->PCR[PIN_NO] &= ~PORT_PCR_MUX_MASK;
 PORTC->PCR[PIN_NO] |= PORT_PCR_MUX(1);
  
 // Set Data Direction Registers for PortB and PortD
 PTC->PDDR |= MASK(PIN_NO);
}

void initGPIOD(int PIN_NO)
{
 // Enable Clock to PORTB and PORTD
 SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK);
 // Configure MUX settings to make all 3 pins GPIO
 PORTD->PCR[PIN_NO] &= ~PORT_PCR_MUX_MASK;
 PORTD->PCR[PIN_NO] |= PORT_PCR_MUX(1);
  
 // Set Data Direction Registers for PortB and PortD
 PTD->PDDR |= MASK(PIN_NO);
}