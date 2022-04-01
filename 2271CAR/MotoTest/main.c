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
#include "LED.h"


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


void initUltrasound(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK;

	PORTE->PCR[TRIGGER] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[TRIGGER] |= PORT_PCR_MUX(1);
	PTE->PDDR |= MASK(TRIGGER); //Trigger is output
	
	//Interrupts for echo pin
	PORTD->PCR[ECHO] |= (PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0b1011));
	PORTD->PCR[ECHO] &= ~PORT_PCR_PS_MASK;
	PTD->PDDR &= ~MASK(ECHO); //Echo is input
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_SetPriority(PORTD_IRQn, 2);
	NVIC_EnableIRQ(PORTD_IRQn);	
	PORTD_ISFR = 0xFFFFFFFF; //Clear flag 
	
}

/*----------------------------------------------------------------------------
 * Threads
 *---------------------------------------------------------------------------*/
float distance = 0;
int count = 0;
int tooClose = 0;
int firsttime = 0;

void ultrasonic (void *argument) {
	for(;;) {
		//Send from trigger
		PTE->PSOR |= MASK(TRIGGER);
		osDelay(10); //delay for 10 microseconds
		PTE->PCOR |= MASK(TRIGGER);
		//Read from echo
		//Rising edge start counter (prolly use pit module with a counter)
		//Falling edge end counter
		//Pause execution till the count is done (semaphore here)
		osSemaphoreAcquire(ultraSem,osWaitForever);
		//distances: 5cm <-> 300 cm : 0.000147s <-> 0.00882s 
		distance = counter * 0.034 / 2;
		if(distance < 10)
			tooClose = 1;
		else {
			tooClose = 0;
			firsttime = 0;
		}
	}
}

uint8_t data = 0;


void app_main (void *argument) {
  for (;;) {
		data = returnData();
		if (tooClose == 1 && data != 0x32) {
		  if(firsttime == 0) {
				setDirection(0x32);
				///osDelay(100);
				//setDirection(0x35);
				firsttime = 1;
			} else {
				setDirection(0x35);
			}
		}
		else {
			setDirection(data);
		}
	}
}

int numNotes = 0;
int i = 0;

void buzzer_thread (void *argument) {
	while(returnData() == 0);
	numNotes = getNumNotes();
	for(;;) {
		for(i = 0; i < numNotes; i+=2)
		{
			playNote(i);
			osDelay(200);
		}
	}
}

int j = 0;

void led_thread (void *argument) {
	 for (;;) {
			data = returnData();
			playRedLedSeq(returnDelay(data));
			playGreenLedSeq(&j, data); 
	 }
}

osThreadAttr_t buzzer_attr = {
  .priority = osPriorityHigh              
};

osThreadAttr_t ultra_attr = {
  .priority = osPriorityHigh              
};
 
int main (void) {
 
  // System Initialization
	SystemCoreClockUpdate();
	initUART(BAUD_RATE);
	initMotors();
	initBuzzer();
	InitGPIOGreenLED();
	InitGPIORedLED();	
  // ...
	
	//Control mechanisms
	//Semaphore to signal the echo pulse is finished
	
	initPIT();
	initUltrasound();
	
	osKernelInitialize();                 // Initialize CMSIS-RTOS
	ultraSem = osSemaphoreNew(1, 0, NULL);
  osThreadNew(buzzer_thread, NULL, &buzzer_attr);  
	osThreadNew(led_thread, NULL, &buzzer_attr); 
	osThreadNew(app_main, NULL, NULL);
	osThreadNew(ultrasonic, NULL, &ultra_attr);
  osKernelStart();       
  for (;;) {}
}
