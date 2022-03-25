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


typedef struct{
	uint8_t cmd;
	uint8_t data;
}Message;

Message message;
osThreadId_t redLED_Id, greenLED_Id, blueLED_Id, control_Id;
osEventFlagsId_t myEvent;
osMessageQueueId_t redMQ, greenMQ, blueMQ;
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void control (void *argument) {
 
  for (;;) {
	message.cmd = 0x01;
	message.data = 0x03;
	osMessageQueuePut(redMQ, &message, NULL, 0);
	osMessageQueuePut(greenMQ, &message, NULL, 0);
	osMessageQueuePut(blueMQ, &message, NULL, 0);
	osDelay(1000);
	message.cmd = 0x02;
	message.data = 0x03;
	osMessageQueuePut(redMQ, &message, NULL, 0);
	osMessageQueuePut(greenMQ, &message, NULL, 0);
	osMessageQueuePut(blueMQ, &message, NULL, 0);
	osDelay(1000);
	message.cmd = 0x03;
	message.data = 0x03;
	osMessageQueuePut(redMQ, &message, NULL, 0);
	osMessageQueuePut(greenMQ, &message, NULL, 0);
	osMessageQueuePut(blueMQ, &message, NULL, 0);
	osDelay(1000);
 }
}


void led_red (void *argument) {
	
	Message rxmessage;
 
  for (;;) {
	osMessageQueueGet(redMQ, &rxmessage, NULL, 0);
	if(rxmessage.cmd == 0x01) {
		offLED();
		switch(rxmessage.data) {
			case 0x01:
				led_lightUp(0);
				osDelay(1000);
				offLED();
				osDelay(1000);
				break;
			case 0x02:
				led_lightUp(0);
				osDelay(500);
				offLED();
				osDelay(500);
				break;
			case 0x03:
				led_lightUp(0);
				osDelay(125);
				offLED();
				osDelay(125);
				break;
		}
	}
 }
}

void led_green (void *argument) {
 
	Message rxmessage;
  // ...
  for (;;) {
	osMessageQueueGet(greenMQ, &rxmessage, NULL, 0);
	if(rxmessage.cmd == 0x02) {
		offLED();
		switch(rxmessage.data) {
			case 0x01:
				led_lightUp(1);
				osDelay(1000);
				offLED();
				osDelay(1000);
				break;
			case 0x02:
				led_lightUp(1);
				osDelay(500);
				offLED();
				osDelay(500);
				break;
			case 0x03:
				led_lightUp(1);
				osDelay(125);
				offLED();
				osDelay(125);
				break;
		}
	}
 }
}

void led_blue (void *argument) {
 
	Message rxmessage;
  // ...
  for (;;) {
	osMessageQueueGet(blueMQ, &rxmessage, NULL, 0);
	if(rxmessage.cmd == 0x03) {
		offLED();
		switch(rxmessage.data) {
			case 0x01:
				led_lightUp(2);
				osDelay(1000);
				offLED();
				osDelay(1000);
				break;
			case 0x02:
				led_lightUp(2);
				osDelay(500);
				offLED();
				osDelay(500);
				break;
			case 0x03:
				led_lightUp(2);
				osDelay(125);
				offLED();
				osDelay(125);
				break;
			}
		}
	}
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
  offLED();
 
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	myEvent = osEventFlagsNew(NULL);
	redMQ = osMessageQueueNew(2,sizeof(message), NULL);
	greenMQ = osMessageQueueNew(2,sizeof(message), NULL);
	blueMQ = osMessageQueueNew(2,sizeof(message), NULL);
  osThreadNew(led_red, NULL, NULL);  
	osThreadNew(led_green, NULL, NULL);  
	osThreadNew(led_blue, NULL, NULL);  
	osThreadNew(control, NULL, NULL);  
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
