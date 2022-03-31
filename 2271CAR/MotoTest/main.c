/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"
#include "Constants.h"
#include "UART.h"
#include "Motors.h"
#include "PWM.h"
#include "Buzzer.h"

volatile int counter = 0;
//PIT IRQ here
void PIT_IRQHandler() {
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
	NVIC_SetPriority(PIT_IRQn, 0);
	NVIC_EnableIRQ(PIT_IRQn);
}

volatile int ultraFlag = 0;
osSemaphoreId_t ultraSem;

volatile int check = 0;

void PORTA_IRQHandler(void) //change X to whatever port the echo pin is supposed to be, enable interrupts
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
	PORTA_ISFR = 0xFFFFFFFF; //Clear flag (change to whatever port)
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
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[ECHO] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[ECHO] |= PORT_PCR_MUX(3) | PORT_PCR_PE_MASK; 
	PORTE->PCR[ECHO] &= ~PORT_PCR_PS_MASK;
	
	PORTA->PCR[TRIGGER] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[TRIGGER] |= PORT_PCR_MUX(1);
	PTA->PDDR |= MASK(TRIGGER); //Trigger is output
	
	
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	//CLOCK SETUP
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; //Clearing the bits in TPMSRC
	//Use MCGFLLCLK CLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // Prescaler = 2^7 = 128
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM0_C4SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_ELSA(1));
	TPM0_C4SC |= TPM_CnSC_CHIE_MASK; //Enable interrupts

	TPM0_MOD = 0xFFFFFFFF; //Causing issues
	
	NVIC_ClearPendingIRQ(TPM0_IRQn);
	NVIC_SetPriority(TPM0_IRQn, 2);
	NVIC_EnableIRQ(TPM0_IRQn);
	TPM0_STATUS = 0xFFFFFFFF;
	
	
	
	/*
	//Interrupts for echo pin
	PORTA->PCR[ECHO] |= (PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0b1011));
	PORTA->PCR[ECHO] &= ~PORT_PCR_PS_MASK;
	PTA->PDDR &= ~MASK(ECHO); //Echo is input
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_SetPriority(PORTA_IRQn, 2);
	NVIC_EnableIRQ(PORTA_IRQn);
	PORTA_ISFR = 0xFFFFFFFF; //Clear flag 
	*/
}

/*----------------------------------------------------------------------------
 * Threads
 *---------------------------------------------------------------------------*/
float distance = 0;
int count = 0;
void ultrasonic (void *argument) {
	for(;;) {
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
	}
}

uint8_t data = 0;

void app_main (void *argument) {
	
  for (;;) {
		data = returnData();
		setDirection(data); //Might need make the motors have some delays for other threads to be able to take control
	}
}

int numNotes = 0;
int i = 0;

void buzzer_thread (void *argument) {
	numNotes = getNumNotes();
	for(;;) {
		for(i = 0; i < numNotes; i+=2)
		{
			playNote(i);
			osDelay(1000/2);
		}
	}
}

osThreadAttr_t buzzer_attr = {
  .priority = osPriorityHigh              
};
 
int main (void) {
 
  // System Initialization
	SystemCoreClockUpdate();
	initUART(BAUD_RATE);
	initMotors();
	initBuzzer();
  // ...
	
	//Control mechanisms
	//Semaphore to signal the echo pulse is finished
	
	//initPIT();
	//initUltrasound();
	
	osKernelInitialize();                 // Initialize CMSIS-RTOS
	ultraSem = osSemaphoreNew(1, 0, NULL);
  osThreadNew(buzzer_thread, NULL, &buzzer_attr);  
	osThreadNew(app_main, NULL, NULL);
  osKernelStart();       
  for (;;) {}
}
