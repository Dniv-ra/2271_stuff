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
osSemaphoreId_t ultraSem, autoSem;

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
		if(distance < 15)
			tooClose = 1;
		else {
			tooClose = 0;
			firsttime = 0;
		}
	}
}

uint8_t data = 0;

int flag = 0;
void app_main (void *argument) {
  while(data != 0x41) {
		data = returnData();
		setDirection(data, 1);
	}
	osSemaphoreRelease(autoSem);
}
int time = 0;
void runCommand(int command, int duration) {
	time = 0;
	data = command; //0x39 is sharp right
	while(time < duration) {
		setDirection(command, 0.5);
		osDelay(1);
		time++;
	}
	//Stop
	time = 0;
	data = 0x35;
	while(time < 100) { //0x35 is stop
		setDirection(0x35, 0.5);
		osDelay(1);
		time++;
	}
}


void autonomous_thread(void *attr) {
	osSemaphoreAcquire(autoSem, osWaitForever);
	int steps[] = {
		0x39, (RIGHT_KEBELAKANG_PUSING / 4), //turn right 1
		0x31, FORWARD_AUTO, //move forward 1
		0x38, (LEFT_KEBELAKANG_PUSING / 2), //turn left 1
		0x31, FORWARD_AUTO, //move forward 2
		0x38, (LEFT_KEBELAKANG_PUSING / 2), //turn left 2
		0x31, FORWARD_AUTO, //move forward 3
		0x38, (LEFT_KEBELAKANG_PUSING / 2), //turn left 3
		0x31, FORWARD_AUTO, //move forward 4
		0x39, (RIGHT_KEBELAKANG_PUSING / 4) + 100, //turn right 2
	};
	//Autonomous mode
	//We will use all sharp turns here
	//Move forward while no object is close
	//Moving towards object
	
	/* 1. Move forward 0 */
	while(!tooClose) { 
		setDirection(0x31, 0.5); //0x31 is forward
	}	
	//Stop at object
	time = 0;
	data = 0x35; //0x35 is stop
	while(time < 500) {
		setDirection(0x35, 0.5);
		osDelay(1);
		time++;
	}
	
	for(int k = 0; k < (sizeof(steps) / sizeof(steps[0])); k+=2){
		runCommand(steps[k], steps[k+1]);
	}
	
	/* 11. Move Forward 5*/
	tooClose = 0;
	while(!tooClose) { 
		setDirection(0x31, 0.5); //0x31 is forward
	}	
	//Stop at object
	time = 0;
	data = 0x40; //Buzzer change tone
	while(time < 500) {
		setDirection(0x35, 0.5);
		osDelay(1);
		time++;
	}
	
	/* 12. Detect finishing object and STOP*/
}



int numNotes = 0;
int i = 0;
int dur = 0;

void buzzer_thread (void *argument) { 
	//Blocking till start
	while(returnData() == 0);
	//First song
	numNotes = getNumNotes(0);
	while(data!= 0x40) { //if data changes between the two breaks is issue (shouldnt)
		for(i = 0; i < numNotes * 2 && data!= 0x40; i+=2)
		{
			dur = playNote(i); 
			osDelay(dur);
		}
	}
	//Second song
	numNotes = getNumNotes(1);
	for(i = 0; i < numNotes * 2; i+=2)
	{
		dur = playEndNote(i); 
		osDelay(dur);
	}
	TPM2_C0V = calc_cnv(TPM0_MOD, 0);
}

int j = 0;

void led_red_thread (void *argument) {
	 for (;;) {
			playRedLedSeq(returnDelay(data));
	 }
}

void led_green_thread (void *argument) {
	 for (;;) {
			playGreenLedSeq(&j, data); 
			osDelay(100);
	 }
}

osThreadAttr_t main_attr = {
  .priority = osPriorityLow              
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
	autoSem = osSemaphoreNew(1, 0, NULL);
  osThreadNew(buzzer_thread, NULL, NULL);  
	osThreadNew(led_red_thread, NULL, NULL); 
	osThreadNew(led_green_thread, NULL, NULL); 
	osThreadNew(app_main, NULL, &main_attr);
	osThreadNew(ultrasonic, NULL, NULL);
	osThreadNew(autonomous_thread, NULL, NULL);
  osKernelStart();       
  for (;;) {}
}

/*

0x31, 800, //move forward 1
		0x38, 540, //turn left 1
		0x31, 800, //move forward 2
		0x38, 580, //turn left 2
		0x31, 800, //move forward 3
		0x38, 580, //turn left 3
		0x31, 800, //move forward 4
		0x39, 340, //turn right 2

*/
