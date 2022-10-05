// USART 1

#ifndef USART_H_
#define USART_H_

#include "stm32f4xx.h"


// прототипы функций ==========================================================
void USART1_init(void);
void float_to_usart(float data);
void int32_to_usart_1(uint32_t data);



#endif /* USART_H_ */