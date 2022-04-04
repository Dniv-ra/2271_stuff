#include "MKL25Z4.h"
#include "Constants.h"
#include "UART.h"
#include "cmsis_os2.h"

void InitGPIOGreenLED(void)
{
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTC_MASK));
	//GREEN LED
	PORTC->PCR[5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[5] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[2] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[2] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[4] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[4] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[5] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[5] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[12] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[12] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[13] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[13] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[16] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[16] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[17] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[17] |= PORT_PCR_MUX(1);

	PTA->PDDR |= (MASK(2) | MASK(4) | MASK(5) | MASK(12) | MASK(13) | MASK(16) | MASK(17));
	PTC->PDDR |= MASK(5);
	

}

void InitGPIORedLED(void)
{
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK);
	
	//RED LED
	PORTC->PCR[3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[3] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[4] |= PORT_PCR_MUX(1);
		
	PTC->PDDR |= (MASK(3) | MASK(4));
}

void Green_LED_Movement (void) {
		int LED_Array[8] = {1, 2, 4, 5, 12, 13, 16, 17};
		int i = 0;
		
		uint8_t data = returnData();
		while (data != 0x35 && data != 0x00) {
			if (i == 8) {
				i = 0;
			}
			PTA->PCOR |= MASK(1);
			PTA->PCOR |= MASK(2);
			PTA->PCOR |= MASK(4);
			PTA->PCOR |= MASK(5);
			PTA->PCOR |= MASK(12);
			PTA->PCOR |= MASK(13);
			PTA->PCOR |= MASK(16);
			PTA->PCOR |= MASK(17);
			PTA->PSOR |= MASK(LED_Array[i]);
			osDelay(500);
			i++;	
		}

			PTA->PSOR |= MASK(1);
			PTA->PSOR |= MASK(2);
			PTA->PSOR |= MASK(4);
			PTA->PSOR |= MASK(5);
			PTA->PSOR |= MASK(12);
			PTA->PSOR |= MASK(13);
			PTA->PSOR |= MASK(16);
			PTA->PSOR |= MASK(17);

}

void Red_LED_Movement (void) {
	int delay = 250;
//	if (data != 0x35 && data != 0x00) {
	//	delay = 500;
//	}
	//else {
		//delay = 250;
	//}
	while(1) {		
			PTC->PSOR |= MASK(3);
			PTC->PSOR |= MASK(4);
					
			osDelay(delay);
		
			PTC->PCOR |= MASK(3);
			PTC->PCOR |= MASK(4);
			
			osDelay(delay);
	}
	
}

int delay = 0;
int returnDelay(uint8_t data) {
	if (data != 0x35 && data != 0x00 && data != 0x40) {
		delay = 500;
	}
	else {
		delay = 250;
	}
	return delay;
}

int ledcheck = 0;
void playRedLedSeq(int delay) {
		PTC->PSOR |= MASK(3);
		PTC->PSOR |= MASK(4);
				
		ledcheck++;
		osDelay(delay);
	
		PTC->PCOR |= MASK(3);
		PTC->PCOR |= MASK(4);
		
		osDelay(delay);
}

void playGreenLedSeq(int *i, uint8_t data) {
		int LED_Array[8] = {5, 2, 4, 5, 12, 13, 16, 17};
		
		if(data != 0x35 && data != 0x00 && data != 0x40) {
			if (*i == 8) {
				*i = 0;
			}
			PTA->PCOR |= (MASK(2) | MASK(4) | MASK(5) | MASK(12) | MASK(13) | MASK(16) | MASK(17));
			PTC->PCOR |= MASK(5);
			if(*i ==0) {
				PTC->PSOR |= MASK(5);
			} else {
				PTA->PSOR |= MASK(LED_Array[*i]);
			}
			*i = *i + 1;	
		}
		else {
			PTA->PSOR |= (MASK(2) | MASK(4) | MASK(5) | MASK(12) | MASK(13) | MASK(16) | MASK(17));
			PTC->PSOR |= MASK(5);
			*i = 0;
		}
}


