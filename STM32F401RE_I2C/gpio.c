// Настройка линий портов

#include "gpio.h"

//=============================================================================
// Разрешение тактирования порта
void gpio_enable(GPIO_TypeDef* GPIOx)
{
    if(GPIOx == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if(GPIOx == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if(GPIOx == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if(GPIOx == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if(GPIOx == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
//	else if(GPIOx == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
//	else if(GPIOx == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    else if(GPIOx == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
//	else if(GPIOx == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
    return;
}

//=============================================================================
// Настройка линии (pin) порта (GPIOx) на альтернативную функцию (alt_func) c типом выхода (out_type)
// out_type = 0 - push-pull
// out_type = 1 - open-drain
void gpio_alt_func(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t alt_func, _Bool out_type)
{
    // разрешение тактирования порта
    gpio_enable(GPIOx);

    /*
    	AFR[0] – записывает значение в регистр GPIOx_AFRL
    	AFR[1] – записывает значение в регистр GPIOx_AFRH
    	альтернативная ф-ция 0001 - AF1(TIM1/2)
    */
    if(pin < 8) {
        GPIOx->AFR[0] &= ~(0xFUL << 4 * pin);
        GPIOx->AFR[0] |= alt_func << 4 * pin;
    } else {
        GPIOx->AFR[1] &= ~(0xFUL << 4 * (pin - 8));
        GPIOx->AFR[1] |= alt_func << 4 * (pin - 8);
    }

    /*
    	GPIOx_MODER - регистр настройки режимов порта X
    		00: Input (reset state)
    		01: General purpose output mode
    		10: Alternate function mode
    		11: Analog mode
    	MODERy[1:0] = 10 - линия Y порта X в альтернативном режиме
    */
    GPIOx->MODER &= ~(0x3UL << 2 * pin);
    GPIOx->MODER |= 0x2UL << 2 * pin;

    /*
    	GPIOx_OTYPER - настройка схемы выхода порта в режимах ВЫХОД или АЛЬТ.ФУНКЦИЯ
    		0: Output push-pull (reset state)
    		1: Output open-drain
    	в данном случае линии работают в режиме push-pull
    */
    if(out_type == PUSH_PULL) GPIOx->OTYPER &= ~(0x1UL << pin);
    else if(out_type == OPEN_DRAIN) GPIOx->OTYPER |= 0x1UL << pin;

    /*
    	GPIOx_OSPEEDR - скорость портов в режимах ВЫХОД или АЛЬТ.ФУНКЦИЯ
    		00: Low speed (400 кГц)
    	 	01: Medium speed (2 МГц)
    		10: High speed (10 МГц)
    		11: Very high speed (50 МГц)
    	в данном случае выбранные линии работают на частоте 50 МГц
    */
    GPIOx->OSPEEDR |= 0x3UL << 2 * pin;

    /*
    	GPIOx_PUPDR - регистр настройки подтягивания входов
    		00: No pull-up, pull-down
    		01: Pull-up
    		10: Pull-down
    		11: Reserved
    	в данном случае выбранные линии не подтягивают
    */
    GPIOx->PUPDR &= ~(0x3UL << 2 * pin);
    return;
}