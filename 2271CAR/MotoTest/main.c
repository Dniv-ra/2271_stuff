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
#define MOTOR_BACK_LEFT_FWD  2 
#define MOTOR_BACK_LEFT_BK  1 
#define MOTOR_BACK_RIGHT_FWD 5 
#define MOTOR_BACK_RIGHT_BK  4
#define MOTOR_FRONT_LEFT_FWD 29
#define MOTOR_FRONT_LEFT_BK  30
#define MOTOR_FRONT_RIGHT_FWD  4
#define MOTOR_FRONT_RIGHT_BK   5 
#define BAUD_RATE 9600

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


void InitGPIOMotor(void)
{
	// Enable Clock to PORTA
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTE_MASK));
	// Configure MUX settings to make all 3 pins GPIO
	//BACK
	PORTA->PCR[MOTOR_BACK_LEFT_BK] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_BACK_LEFT_BK] |= PORT_PCR_MUX(1); //Change to the TPM setting
	
	PORTA->PCR[MOTOR_BACK_LEFT_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_BACK_LEFT_FWD] |= PORT_PCR_MUX(1); //Change to the TPM setting
	
	PORTA->PCR[MOTOR_BACK_RIGHT_BK] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_BACK_RIGHT_BK] |= PORT_PCR_MUX(1); //Change to the TPM setting
	
	PORTA->PCR[MOTOR_BACK_RIGHT_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_BACK_RIGHT_FWD] |= PORT_PCR_MUX(1); //Change to the TPM setting
	
	//FRONT
	PORTE->PCR[MOTOR_FRONT_LEFT_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[MOTOR_FRONT_LEFT_FWD] |= PORT_PCR_MUX(1); //Change to the TPM setting
	
	PORTE->PCR[MOTOR_FRONT_LEFT_BK] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[MOTOR_FRONT_LEFT_BK] |= PORT_PCR_MUX(1); //Change to the TPM setting
	
	PORTE->PCR[MOTOR_FRONT_RIGHT_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[MOTOR_FRONT_RIGHT_FWD] |= PORT_PCR_MUX(1); //Change to the TPM setting
	
	PORTE->PCR[MOTOR_FRONT_RIGHT_BK] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[MOTOR_FRONT_RIGHT_BK] |= PORT_PCR_MUX(1); //Change to the TPM setting
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
osSemaphoreId_t ultraSem;

void PORTX_IRQHandler(void) //change X to whatever port the echo pin is supposed to be, enable interrupts
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
		PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK; // stop Timer 1
		//Set flag to 0
		ultraFlag = 0;
		//release the semaphore
		osSemaphoreRelease(ultraSem);
	}
	//PORTX_ISFR = 0xFFFFFFFF; //Clear flag (change to whatever port)
}

int calc_mod(int clkspd, int prescaler, int req_freq) {
	int cycles_per_sec = clkspd / prescaler;
	return (cycles_per_sec / req_freq) - 1;
}

int calc_cnv(int mod, float req_duty) {
	return (mod * req_duty);
}

void initPWM(void) {
	//The mode setting is done in initGPIOMotor
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	//CLOCK SETUP
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; //Clearing the bits in TPMSRC
	//Use MCGFLLCLK CLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	//For now setting as TPM1 CH0,l and TPM0 CH0,1 can change later based on pins
	//TPM0
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // Prescaler = 2^7 = 128
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM0_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C1SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	//TPM1
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // Prescaler = 2^7 = 128
	TPM1->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM1_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM1_C1SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	//Set the pwm to 0 for init
	TPM1_MOD = calc_mod(48000000, 128, 262);
	TPM1_C0V = calc_cnv(TPM1_MOD, 0);
	TPM1_C1V = calc_cnv(TPM1_MOD, 0);
	
	TPM0_MOD = calc_mod(48000000, 128, 262);
	TPM0_C0V = calc_cnv(TPM0_MOD, 0);
	TPM0_C1V = calc_cnv(TPM0_MOD, 0);
}

void initUltrasound(void) {
	//Dunno the current pins
	//SIM->SCGC5 |= SIM_SCGC5_PORTX_MASK (change the X to required port)
	//PORTX->PCR[ECHO] &= ~PORT_PCR_MUX_MASK;
	//PORTX->PCR[ECHO] |= PORT_PCR_MUX(1); 
	//PORTX->PCR[TRIGGER] &= ~PORT_PCR_MUX_MASK;
	//PORTX->PCR[TRIGGER] |= PORT_PCR_MUX(1);
	//PTA->PDDR |= MASK(TRIGGER); //Trigger is output
	//PTA->PDDR &= ~MASK(ECHO); //Echo is input
	
	//Interrupts for echo pin
	//PORTX->PCR[ECHO] |= PORT_PCR_IRQC(0b1011); //1011 is either edge
	//NVIC_ClearPendingIRQ(PORTX_IRQn);
	//NVIC_SetPriority(PORTX_IRQn, 2);
	//NVIC_EnableIRQ(PORTX_IRQn);
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
		osSemaphoreAcquire(ultraSem,osWaitForever);
		//distances: 5cm <-> 300 cm : 0.000147s <-> 0.00882s 
		distance = counter * 0.034 / 2;
	}
}

void app_main (void *argument) {
	
  for (;;) {
		switch(received_data) {
			case 0x31: //forward
				//we can have two motors attached to one signal so makes life easier (frt left and back left to same TPM1 ch0, ch1) 
				TPM1_C0V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
			
			case 0x32: //back
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
			
			case 0x33: //left forward curved
				TPM1_C0V = calc_cnv(TPM1_MOD, 0.5); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
			
			case 0x34: //right forward curved
				TPM1_C0V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0.5); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
			
			case 0x36: //left forward curved
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0.5); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
			
			case 0x37: //right backward curved
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0.5); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
			
			case 0x38: //left strict
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
			
			case 0x39: //right strict
				TPM1_C0V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
			
			case 0x35: //stop
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				break;
		}
	}
}
 
int main (void) {
 
  // System Initialization
	SystemCoreClockUpdate();
  // ...
	
	//Control mechanisms
	//Semaphores
	ultraSem = osSemaphoreNew(1, 0, NULL);
	
	InitGPIOMotor();
	initPIT();
	initUART(BAUD_RATE);
	osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);   
  osKernelStart();       
  for (;;) {}
}
