// Таймеры общего назначения

#ifndef TIMERS_H_
#define TIMERS_H_

#include "stm32f4xx.h"
#include "delay.h"
#include "gpio.h"
#define TIM_EnableIT_UPDATE(TIMx) TIMx->DIER |= TIM_DIER_UIE // Макрос разрешения прерывания
#define Timer_Enable(TIMx)   TIMx->CR1 |= TIM_CR1_CEN // Макрос включения таймера
#define TIM_DisableCounter(TIMx)  TIMx->CR1 &= ~TIM_CR1_CEN // Макрос отключения таймера

// прототипы функций ==========================================================
void timer_1_init(void);

void timer_2_init(void);

void timer_5_init(void);

void enc_tim_init(void);

void TIM2_4CH_PWM_init(uint16_t prescaler, uint32_t period, uint32_t compare);

void PWM_OCx_Init(TIM_TypeDef* TIMx, uint8_t channel, uint16_t prescaler, uint32_t period, uint32_t compare);

void TIM_PWM_CAPTURE(TIM_TypeDef* TIMx, uint8_t channel, uint16_t prescaler, uint32_t period);

volatile static uint32_t signal_repository;


#endif /* TIMERS_H_ */