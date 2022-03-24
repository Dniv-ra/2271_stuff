/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"

#define MASK(x) (1 << x)
#define UART2_TX_PIN 3
#define UART2_RX_PIN 2
#define BAUD_RATE 9600
 
void InitGPIO(void)
{
	// Enable Clock to PORTA
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK);
	// Configure MUX settings to make all 3 pins GPIO
	PORTA->PCR[1] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[1] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[2] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[2] |= PORT_PCR_MUX(1);
		
	// Set Data Direction Registers for PortB and PortD
	PTA->PDDR |= (MASK(1) | MASK(2));
}

volatile uint8_t data_to_send;
volatile uint8_t received_data;
//Unsure should double check
//volatile int new_data = 0;

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
		 //Has received new data 
		 //new_data = 1;
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
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

void app_main (void *argument) {
	
  for (;;) {
		switch(received_data) {
			case 0x31:
				PTA->PSOR |= MASK(1);
				PTA->PCOR |= MASK(2);
				break;
			case 0x32:
				PTA->PSOR |= MASK(2);
				PTA->PCOR |= MASK(1);
				break;
			case 0x35:
				PTA->PCOR |= MASK(1);
				PTA->PCOR |= MASK(2);
				break;
		}
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  // ...
	InitGPIO();
	initUART(BAUD_RATE);
	osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);   
  osKernelStart();       
  for (;;) {}
}
