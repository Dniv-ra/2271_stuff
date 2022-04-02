#include "UART.h"
#include "Constants.h"
#include "PWM.h"

void initMotors(void) {
	// Enable Clock to PORTB and C
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK));
	
	// Configure MUX settings to make all 3 pins PWM
	PORTB->PCR[PWM_TP1CH0] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PWM_TP1CH0] |= PORT_PCR_MUX(3); //TPM setting
	
	PORTB->PCR[PWM_TP1CH1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PWM_TP1CH1] |= PORT_PCR_MUX(3); //TPM setting
	
	PORTC->PCR[PWM_TP0CH0] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PWM_TP0CH0] |= PORT_PCR_MUX(4); //TPM setting
	
	PORTC->PCR[PWM_TP0CH1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PWM_TP0CH1] |= PORT_PCR_MUX(4); //TPM setting
	
	//The mode setting is done in initGPIOMotor
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM0_MASK;
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


void InitGPIOMotor(void)
{
	// Enable Clock to PORTB and C
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK));
	
	// Configure MUX settings to make all 3 pins PWM
	PORTB->PCR[PWM_TP1CH0] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PWM_TP1CH0] |= PORT_PCR_MUX(3); //TPM setting
	
	PORTB->PCR[PWM_TP1CH1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PWM_TP1CH1] |= PORT_PCR_MUX(3); //TPM setting
	
	PORTC->PCR[PWM_TP0CH0] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PWM_TP0CH0] |= PORT_PCR_MUX(4); //TPM setting
	
	PORTC->PCR[PWM_TP0CH1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PWM_TP0CH1] |= PORT_PCR_MUX(4); //TPM setting

}

void initPWM(void) {
	//The mode setting is done in initGPIOMotor
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM0_MASK;
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

void setDirection(uint8_t data) {
	switch(data) {
			case 0x31: //forward
				//we can have two motors attached to one signal so makes life easier (frt left and back left to same TPM1 ch0, ch1) 
				TPM1_C0V = calc_cnv(TPM1_MOD, 1);  //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD //TPM1_CH0
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK //TPM1_CH1
				TPM0_C0V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD //TPM0_CH0
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK //TP0_CH1
				break;
			
			case 0x32: //back
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				break;
			
			case 0x33: //left forward curved
				TPM1_C0V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0.1); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				break;
			
			case 0x34: //right forward curved
				TPM1_C0V = calc_cnv(TPM1_MOD, 0.1); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				break;
			
			case 0x36: //left backward curved
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0.1); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				break;
			
			case 0x37: //right backward curved
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0.1); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				break;
			
			case 0x38: //left strict
				TPM1_C0V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				break;
			
			case 0x39: //right strict
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 1); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 1); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				break;
			
			case 0x35: //stop
				TPM1_C0V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_FWD and MOTOR_BACK_RIGHT_FWD
				TPM1_C1V = calc_cnv(TPM1_MOD, 0); //MOTOR_FRONT_RIGHT_BK and MOTOR_BACK_RIGHT_BK
				TPM0_C0V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_FWD and MOTOR_BACK_LEFT_FWD
				TPM0_C1V = calc_cnv(TPM0_MOD, 0); //MOTOR_FRONT_LEFT_BK and MOTOR_BACK_LEFT_BK
				break;
		}
}

