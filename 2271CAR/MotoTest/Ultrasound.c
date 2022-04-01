#include "MKL25Z4.h"
#include "cmsis_os2.h"  
#include "Constants.h"

volatile int counter = 0;
//PIT IRQ here

void PIT_IRQHandler(void) {
	counter++;
	PIT_TFLG1 = 0xFFFFFFFF;
}

void initPIT(void) {
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	PIT_MCR = 0x00;
	PIT_LDVAL1 = 0x0000002F; // setup timer 1 for 47 cycles (cycles/sec * time = value) (clkspd is 48000000 ->  48 cycles is 10^-6 s, 1 microsecond)
  PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK; // enable Timer 1 interrupts 
	PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK;
	NVIC_ClearPendingIRQ(PIT_IRQn);
	NVIC_SetPriority(PIT_IRQn, 2);
	NVIC_EnableIRQ(PIT_IRQn);
}

volatile int ultraFlag = 0;

osSemaphoreId_t ultraSem;

volatile int check = 0;

void PORTD_IRQHandler(void) //change X to whatever port the echo pin is supposed to be, enable interrupts
{
	if(ultraFlag == 0){
		//First call will be for rising (hopefully)
		//Start PIT timer and reset counter to 0
		PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK; // start Timer 1
		//Set flag to 1
		ultraFlag = 1;
		counter = 0;
	} else {
		//2nd call will be for falling 
		//Stop PIT timer
		PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK; // stop Timer 1
		//Set flag to 0
		ultraFlag = 0;
		check++;
		//release the semaphore
		osSemaphoreRelease(ultraSem);
	}
	PORTD_ISFR = 0xFFFFFFFF; //Clear flag (change to whatever port)
}



volatile int consecutive = 0;
volatile int value = 0;

void TPM0_IRQHandler(void) {
	if(consecutive == 0) {
		value = TPM0_C4V;
		consecutive = 1;
	}
	else {
		int temp = TPM0_C4V;
		counter = temp - value;
		if(counter < 0)
			counter = counter + TPM0_MOD;
		consecutive = 0;
		osSemaphoreRelease(ultraSem);
	}
	TPM0_STATUS = 0xFFFFFFFF;
}


void initUltrasound(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;

	PORTA->PCR[TRIGGER] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[TRIGGER] |= PORT_PCR_MUX(1);
	PTA->PDDR |= MASK(TRIGGER); //Trigger is output
	
	//Interrupts for echo pin
	PORTD->PCR[ECHO] |= (PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0b1011));
	PORTD->PCR[ECHO] &= ~PORT_PCR_PS_MASK;
	PTA->PDDR &= ~MASK(ECHO); //Echo is input
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_SetPriority(PORTD_IRQn, 2);
	NVIC_EnableIRQ(PORTD_IRQn);
	PORTD_ISFR = 0xFFFFFFFF; //Clear flag 
	
}

float distance = 0;
int count = 0;
int tooClose = 0;

float getDistance(void) {
		//Send from trigger
		PTA->PSOR |= MASK(TRIGGER);
		osDelay(10); //delay for 10 microseconds
		PTA->PCOR |= MASK(TRIGGER);
		//Read from echo
		//Rising edge start counter (prolly use pit module with a counter)
		//Falling edge end counter
		//Pause execution till the count is done (semaphore here)
		osSemaphoreAcquire(ultraSem,osWaitForever);
		//distances: 5cm <-> 300 cm : 0.000147s <-> 0.00882s 
		distance = counter * 0.034 / 2;
		return distance;
}
