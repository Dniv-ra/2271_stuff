//PWM helper functions

#ifndef PWM
#define PWM
   int calc_mod(int clkspd, int prescaler, int req_freq);
	 int calc_cnv(int mod, float req_duty);
#endif
