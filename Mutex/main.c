/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))

void InitGPIO(void)
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

void offLED() {
	PTB->PSOR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PSOR |= MASK(BLUE_LED);
}

void led_lightUp(int led) {
	//Turn on corresponding led
	switch(led){
		case 0:
			PTB->PCOR |= MASK(RED_LED);
			break;
		case 1:
			PTB->PCOR |= MASK(GREEN_LED);
			break;
		case 2:
			PTD->PCOR |= MASK(BLUE_LED);
			break;
	}
}

static void delay(volatile uint32_t duration)
{
  while (duration != 0) 
  {
    __asm("NOP");
     duration--;
  }
}
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

const osThreadAttr_t attributes = {
	.priority = osPriorityNormal1
};

osMutexId_t mutex;

void led_red (void *argument) {
	
  for (;;) {
		osMutexAcquire(mutex, osWaitForever); 
		led_lightUp(0);
		osDelay(1000);
		offLED();
		osDelay(1000);
		osMutexRelease(mutex);
	}
}

void led_green (void *argument) {
 
  // ...
  for (;;) {
		osMutexAcquire(mutex, osWaitForever); 
		led_lightUp(1);
		osDelay(1000);
		offLED();
		osDelay(1000);
		//osMutexRelease(mutex);
	}
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	offLED();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	mutex = osMutexNew(NULL);
  osThreadNew(led_red, NULL, NULL);  
	osThreadNew(led_green, NULL, &attributes);  
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
