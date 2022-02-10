#include "MKL25Z4.h"

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define SWITCH 6 //PortD switch 6
#define MASK(x) (1 << (x))

volatile int count = 0;

void PORTD_IRQHandler()
{
	// Clear Pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	// Updating some variable / flag
	count++;
	//Clear INT Flag
	PORTD->ISFR = 0xffffffff;
}

static void delay(volatile uint32_t duration)
{
  while (duration != 0) 
  {
    __asm("NOP");
     duration--;
  }
}

void initSwitch(void)
{
	// enable clock for PortD
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	/* Select GPIO and enable pull-up resistors and interrupts on
	falling edges of pin connected to switch*/
	PORTD->PCR[SWITCH] |= (PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0b1010));
	
	// Set PORT D Switch bit to input
	PTD->PDDR &= ~MASK(SWITCH);
	
	//Enable Interrupts
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_SetPriority(PORTD_IRQn, 2);
	NVIC_EnableIRQ(PORTD_IRQn);
}

void initLED(void)
{
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
		
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
		
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
}

int main(void)
{
	initSwitch();
	initLED();
	while(1)
	{
		//Turn on corresponding led
		switch(count % 3){
			case 0:
				PTB->PSOR |= MASK(GREEN_LED);
				PTD->PSOR |= MASK(BLUE_LED);
				PTB->PCOR |= MASK(RED_LED);
				break;
			case 1:
				PTB->PSOR |= MASK(RED_LED);
				PTD->PSOR |= MASK(BLUE_LED);
				PTB->PCOR |= MASK(GREEN_LED);
				break;
			case 2:
				PTB->PSOR |= (MASK(RED_LED) | MASK(GREEN_LED));
				PTD->PCOR |= MASK(BLUE_LED);
				break;
		}
	}
}