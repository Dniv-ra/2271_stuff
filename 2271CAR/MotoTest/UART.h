#include "MKL25Z4.h"             // Apparently need for uint32

//UART functions

void UART2_IRQHandler(void);
void initUART(uint32_t baud);
uint8_t returnData(void);

