// Задержки на SysTick

#ifndef DELAY_H_
#define DELAY_H_


//=============================================================================
// Библиотеки 
#include "stm32f4xx.h"

#define F_AHB 		72000000UL              				// частота AHB тактирования SysTick 
#define TOP_us		F_AHB / 1000000UL - 1UL					// вершина счёта для мкс
#define TOP_ms		F_AHB / 1000UL - 1UL					// вершина счёта для мс

// прототипы функций ==========================================================
void _delay_ms(uint32_t set_value);							// задержка в мс без прерываний
void _delay_us(uint32_t set_value);							// задержка в мкс без прерываний


#endif /* DELAY_H_ */