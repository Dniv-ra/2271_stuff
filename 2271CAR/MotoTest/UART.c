#include "UART.h"
#include "Constants.h"

volatile uint8_t data_to_send;
volatile uint8_t received_data;

void UART2_IRQHandler(void) 
{
	//If empty means triggered by UDRE else is RXC
	if(UART2->S1 & UART_S1_TDRE_MASK) 
	{
		UART2_D = data_to_send;
		UART2->C2 &= ~UART_C2_TIE_MASK;
	}
	else
	{
		 received_data = UART2_D;
	}
}

void initUART(uint32_t baud) 
{
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	PORTD->PCR[UART2_TX_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[UART2_TX_PIN] |= PORT_PCR_MUX(3);
	
	PORTD->PCR[UART2_RX_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[UART2_RX_PIN] |= PORT_PCR_MUX(3);
	
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

uint8_t returnData(void) {
	return received_data;
}

