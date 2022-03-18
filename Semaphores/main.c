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

#define SW_POS 6 // PortD Pin 6

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

volatile uint8_t data_to_send;
volatile uint8_t received_data;
volatile int int_count;
volatile int test = 0;

//global semaphore
osSemaphoreId_t mySem;

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
   osSemaphoreRelease(mySem);
 }
}

/* Init UART2 */
void initUART2(uint32_t baud_rate)
{
  uint32_t divisor, bus_clock;
  SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
  PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
  // Ensure Tx and Rx are disabled before configuration
  UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
  // Set Baud Rate to desired value
  bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
  divisor = bus_clock / (baud_rate * 16);
  UART2->BDH = UART_BDH_SBR(divisor >> 8);
  UART2->BDL = UART_BDL_SBR(divisor);
  // No Parity, 8-bits
  UART2->C1 = 0;
  UART2->S2 = 0;
  UART2->C3 = 0;
  NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
  NVIC_ClearPendingIRQ(UART2_IRQn);
  NVIC_EnableIRQ(UART2_IRQn);
  // Enable Tx and Rx
  UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | (UART_C2_RIE_MASK));
}



static void delay(volatile uint32_t duration)
{
  while (duration != 0) 
  {
    __asm("NOP");
     duration--;
  }
}

void PORTD_IRQHandler(void) 
{ 
  // clear pending interrupts
  NVIC_ClearPendingIRQ(PORTD_IRQn);
  int_count++;
  delay(0x80000);
  
  // clear status flags 
  PORTD->ISFR = 0xffffffff;
}

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


void initSwitch(void)
{
  // enable clock for PortD
  SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK);
  /* Select GPIO and enable pull-up resistors and interrupts on 
  falling edges of pin connected to switch*/
  PORTD->PCR[SW_POS] |= (PORT_PCR_MUX(1) | 
  PORT_PCR_PS_MASK |
  PORT_PCR_PE_MASK |
  PORT_PCR_IRQC(0x0a));
  // Set PORT D Switch bit to input
  PTD->PDDR &= ~MASK(SW_POS);
  //Enable Interrupts
  NVIC_SetPriority(PORTD_IRQn, 2);
  NVIC_ClearPendingIRQ(PORTD_IRQn);
  NVIC_EnableIRQ(PORTD_IRQn);
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


 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

osMutexId_t mutex;

void led_red (void *argument) {
 
  for (;;) {
  osSemaphoreAcquire(mySem, osWaitForever);
    
  led_lightUp(0);
  osDelay(1000);
  offLED();
  osDelay(1000);
  // osSemaphoreRelease(mySem);
 }
}

void led_green (void *argument) {
 
  // ...
  for (;;) {
  osSemaphoreAcquire(mySem, osWaitForever);
  led_lightUp(1);
  osDelay(1000);
  offLED();
  osDelay(1000);
  // osSemaphoreRelease(mySem);
 }
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  initSwitch();
  initUART2(BAUD_RATE);
  InitGPIO();
  
  offLED();
 
  osKernelInitialize();     // Initialize CMSIS-RTOS
  mySem = osSemaphoreNew(1,0,NULL);
  
  osThreadNew(led_red, NULL, NULL);  
  osThreadNew(led_green, NULL, NULL);  
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
