#include "MKL25Z4.h"

#define UART2_TX_PIN 22
#define UART2_RX_PIN 23
#define BAUD_RATE 9600

volatile uint8_t data_to_send;
volatile uint8_t received_data;
//Unsure should double check
volatile int new_data = 0;

void UART2_IRQHandler(void) 
{
	//If empty means triggered by UDRE else is RXC
	if(UART2->D) 
	{
		UART2_D = data_to_send;
		UART2->C2 &= ~UART_C2_TIE_MASK;
	}
	else
	{
		 received_data = UART2_D;
		 //Has received new data 
		 new_data = 1;
	}
}

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

//RX
uint8_t receive()
{
	return received_data;
}

//TX
void transmit(uint8_t data)
{
	data_to_send = data;
	UART2->C2 |= UART_C2_TIE_MASK;
}

static void delay(volatile uint32_t duration)
{
  while (duration != 0) 
  {
    __asm("NOP");
     duration--;
  }
}

int main() 
{
	uint8_t rx_data = 0;
	SystemCoreClockUpdate();
	initUART(BAUD_RATE);
	
	while(1)
	{
		
		if(new_data == 1)
		{
			rx_data = receive();
			new_data = 0;
		}
		transmit(0x69);
	}
}

