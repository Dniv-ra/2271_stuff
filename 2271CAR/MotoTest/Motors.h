#include "MKL25Z4.h"

//Motor functions
#ifndef MOTORS
#define MOTORS
	 void initMotors(void);
   void InitGPIOMotor(void);
	 void initPWM(void);
	 void setDirection(uint8_t data);
#endif
