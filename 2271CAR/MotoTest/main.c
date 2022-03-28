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
#define MOTOR_BACK_LEFT_FWD  2 //
#define MOTOR_BACK_LEFT_BK  1 //
#define MOTOR_BACK_RIGHT_FWD 5 //
#define MOTOR_BACK_RIGHT_BK  4//
#define MOTOR_FRONT_LEFT_FWD 29//
#define MOTOR_FRONT_LEFT_BK  30//
#define MOTOR_FRONT_RIGHT_FWD  4//
#define MOTOR_FRONT_RIGHT_BK   5//
#define BAUD_RATE 9600
 
void InitGPIO(void)
{
	// Enable Clock to PORTA
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTE_MASK));
	// Configure MUX settings to make all 3 pins GPIO
	//BACK
	PORTA->PCR[MOTOR_BACK_LEFT_BK] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_BACK_LEFT_BK] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[MOTOR_BACK_LEFT_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_BACK_LEFT_FWD] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[MOTOR_BACK_RIGHT_BK] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_BACK_RIGHT_BK] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[MOTOR_BACK_RIGHT_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_BACK_RIGHT_FWD] |= PORT_PCR_MUX(1);
	
	//FRONT
	PORTE->PCR[MOTOR_FRONT_LEFT_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[MOTOR_FRONT_LEFT_FWD] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[MOTOR_FRONT_LEFT_BK] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[MOTOR_FRONT_LEFT_BK] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[MOTOR_FRONT_RIGHT_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[MOTOR_FRONT_RIGHT_FWD] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[MOTOR_FRONT_RIGHT_BK] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[MOTOR_FRONT_RIGHT_BK] |= PORT_PCR_MUX(1);
		
	// Set Data Direction Registers for PortB and PortD
	PTA->PDDR |= (MASK(MOTOR_BACK_LEFT_BK) | MASK(MOTOR_BACK_LEFT_FWD) | MASK(MOTOR_BACK_RIGHT_BK) | MASK(MOTOR_BACK_RIGHT_FWD));
	PTE->PDDR |= (MASK(MOTOR_FRONT_LEFT_FWD) | MASK(MOTOR_FRONT_LEFT_BK) | MASK(MOTOR_FRONT_RIGHT_FWD) | MASK(MOTOR_FRONT_RIGHT_BK));
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


volatile int counter = 0;
//PIT IRQ here
void PIT_IRQHandler() {
	counter++;
}


void initPIT(void) {
	PIT_LDVAL1 = 47; // setup timer 1 for 47 cycles (cycles/sec * time = value) (clkspd is 48000000 ->  48 cycles is 10^-6 s, 1 microsecond)
  PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK; // enable Timer 1 interrupts 
	NVIC_ClearPendingIRQ(PIT_IRQn);
	NVIC_SetPriority(PIT_IRQn, 2);
	NVIC_EnableIRQ(PIT_IRQn);
}

volatile int ultraFlag = 0;

void PORTA_IRQHandler(void) //change A to whatever port the echo pin is supposed to be, enable interrupts
{
	
	if(!ultraFlag){
		//First call will be for rising (hopefully)
		//Start PIT timer and reset counter to 0
		PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK; // start Timer 1
		//Set flag to 1
		ultraFlag = 1;
		counter = 0;
	} else {
		//2nd call will be for falling 
		//Stop PIT timer
		PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK; // start Timer 1
		//Set flag to 0
		ultraFlag = 0;
		//release the semaphore
		//osSemaphoreRelease(ultraSem);
	}
}

/*----------------------------------------------------------------------------
 * Threads
 *---------------------------------------------------------------------------*/

int distance = 0;
void ultrasonic (void *argument) {
	for(;;) {
		//Send from trigger
		//PTX->PSOR |= MASK(TRIGGER);
		//delay(10); //delay for 10 microseconds
		//PTX->PCOR |= MASK(TRIGGER);
		//Read from echo
		//Rising edge start counter (prolly use pit module with a counter)
		//Falling edge end counter
		//Pause execution till the count is done (semaphore here)
		//osSemaphoreAcquire(ultraSem,osWaitForever);
		//distances: 5cm <-> 300 cm : 0.000147s <-> 0.00882s 
		//distance = counter * 0.034 / 2;
	}
}

void app_main (void *argument) {
	
  for (;;) {
		switch(received_data) {
			case 0x31: //forward
				PTA->PSOR |= MASK(MOTOR_BACK_LEFT_FWD);
				PTA->PCOR |= MASK(MOTOR_BACK_LEFT_BK);
				PTA->PSOR |= MASK(MOTOR_BACK_RIGHT_FWD);
				PTA->PCOR |= MASK(MOTOR_BACK_RIGHT_BK);
			
				PTE->PSOR |= MASK(MOTOR_FRONT_LEFT_FWD);
				PTE->PCOR |= MASK(MOTOR_FRONT_LEFT_BK);
				PTE->PSOR |= MASK(MOTOR_FRONT_RIGHT_FWD);
				PTE->PCOR |= MASK(MOTOR_FRONT_RIGHT_BK);
				break;
			case 0x32: //back
				PTA->PSOR |= MASK(MOTOR_BACK_LEFT_BK);
				PTA->PCOR |= MASK(MOTOR_BACK_LEFT_FWD);
				PTA->PSOR |= MASK(MOTOR_BACK_RIGHT_BK);
				PTA->PCOR |= MASK(MOTOR_BACK_RIGHT_FWD);
			
				PTE->PSOR |= MASK(MOTOR_FRONT_LEFT_BK);
				PTE->PCOR |= MASK(MOTOR_FRONT_LEFT_FWD);
				PTE->PSOR |= MASK(MOTOR_FRONT_RIGHT_BK);
				PTE->PCOR |= MASK(MOTOR_FRONT_RIGHT_FWD);
				break;
			case 0x33: //left
				PTA->PSOR |= MASK(MOTOR_BACK_LEFT_BK);
				PTA->PCOR |= MASK(MOTOR_BACK_LEFT_FWD);
				PTA->PSOR |= MASK(MOTOR_BACK_RIGHT_FWD);
				PTA->PCOR |= MASK(MOTOR_BACK_RIGHT_BK);
			
				PTE->PSOR |= MASK(MOTOR_FRONT_LEFT_BK);
				PTE->PCOR |= MASK(MOTOR_FRONT_LEFT_FWD);
				PTE->PSOR |= MASK(MOTOR_FRONT_RIGHT_FWD);
				PTE->PCOR |= MASK(MOTOR_FRONT_RIGHT_BK);
				break;
			case 0x34: //right
				PTA->PSOR |= MASK(MOTOR_BACK_LEFT_FWD);
				PTA->PCOR |= MASK(MOTOR_BACK_LEFT_BK);
				PTA->PSOR |= MASK(MOTOR_BACK_RIGHT_BK);
				PTA->PCOR |= MASK(MOTOR_BACK_RIGHT_FWD);
			
				PTE->PSOR |= MASK(MOTOR_FRONT_LEFT_FWD);
				PTE->PCOR |= MASK(MOTOR_FRONT_LEFT_BK);
				PTE->PSOR |= MASK(MOTOR_FRONT_RIGHT_BK);
				PTE->PCOR |= MASK(MOTOR_FRONT_RIGHT_FWD);
				break;
			case 0x35:
				PTA->PCOR |= MASK(MOTOR_BACK_LEFT_BK);
				PTA->PCOR |= MASK(MOTOR_BACK_LEFT_FWD);
				PTA->PCOR |= MASK(MOTOR_BACK_RIGHT_BK);
				PTA->PCOR |= MASK(MOTOR_BACK_RIGHT_FWD);
			
				PTE->PCOR |= MASK(MOTOR_FRONT_LEFT_FWD);
				PTE->PCOR |= MASK(MOTOR_FRONT_LEFT_BK);
				PTE->PCOR |= MASK(MOTOR_FRONT_RIGHT_FWD);
				PTE->PCOR |= MASK(MOTOR_FRONT_RIGHT_BK);
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
