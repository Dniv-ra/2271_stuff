//Ultrasound functions

#ifndef ULTRASOUND
#define ULTRASOUND
  void PIT_IRQHandler(void);
	void initPIT(void);
	void PORTD_IRQHandler(void);
	void initUltrasound(void);
	float getDistance(void);
#endif

