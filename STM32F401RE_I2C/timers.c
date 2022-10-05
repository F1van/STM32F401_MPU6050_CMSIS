// Таймеры общего назначения
#include "stm32f401xe.h"
#include <stdio.h>
#include "delay.h"

void TIM5_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->PSC = 71UL;
    TIM5->ARR = 0xffff;
    //TIM5->CR1 |= TIM_CR1_OPM;
    TIM5->CR1 |= TIM_CR1_CEN;
    //_delay_ms(10);
    return;
}
