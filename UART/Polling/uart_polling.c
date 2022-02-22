#include "MKL25Z4.h"

#define UART2_TX_PIN 22
#define UART2_RX_PIN 23
#define BAUD_RATE 9600


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
	//Polling no need set interrupts so just the two
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
}

//RX
uint8_t receive()
{
	while(!(UART2_S1 & UART_S1_RDRF_MASK));
	return UART2_D;
}

//TX
void transmit(uint8_t data)
{
	while(!(UART2_S1 & UART_S1_TDRE_MASK));
	UART2_D = data;
}


int main() 
{
	uint8_t rx_data = 0x69;
	SystemCoreClockUpdate();
	initUART(BAUD_RATE);
	
	while(1)
	{
		transmit(0x69);
	}
}

