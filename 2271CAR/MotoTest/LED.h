//LED fucntions
#include "MKL25Z4.h"                    // Device header

#ifndef MYLED //BUZZER already taken dunno where
#define MYLED
	void InitGPIOGreenLED(void);
	void InitGPIORedLED(void);
	void Green_LED_Movement (void);
	void Red_LED_Movement (void);
	int returnDelay(uint8_t data);
	void playRedLedSeq(int delay);
	void playGreenLedSeq(int *i, uint8_t data);
#endif

