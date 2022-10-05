#include "EXTI.h"

void EXTI_init(void)
{
    /*
        RCC_APB2ENR - регистр разрешения тактирования переферийных устройств шины APB2
            SYSCFGEN = 1 - разрешаем тактирование контроллера системной конфигурации SYSCFG
    */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /*
      GPIOx_MODER - регистр настройки режимов порта X
      00: Input (reset state)
      01: General purpose output mode
      10: Alternate function mode
      11: Analog mode

      MODERy[1:0] = 10 - линия Y порта X в альтернативном режиме
    */

    GPIOA->MODER &= ~GPIO_MODER_MODER0;

    /*
      GPIOx_PUPDR - регистр настройки подтягивания входов
      00: No pull-up, pull-down
      01: Pull-up
      10: Pull-down
      11: Reserved
      в данном случае выбранные линии не подтягивают
    */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;
    //GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_0;

    /*
        SYSCFG_EXTICR1 - регистр настройки внешних прерываний EXTI11
            EXTIx[3:0]: EXTIx configuration (x = 0 to 15)
            0000: PA[x] pin
            0001: PB[x] pin
            0010: PC[x] pin
            0011: PD[x] pin
            0100: PE[x] pin
            0101: PH[x] (only PH[2:0], PH3 is not available)
            0110: PF[x] pin (Cat.3, Cat.4, Cat.5 and Cat.6 devices only)
            0111: PG[x] pin (Cat.3, Cat.4, Cat.5 and Cat.6 devices only)

        EXTI1[3:0] = 0001 - разрешаем внешние прерывания по линии PA2
    */
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

    /*
        EXTI_RTSR - регистр выбора переднего фронта прерывания
            0: Rising edge trigger disabled
            1: Rising edge trigger enabled

        TR3 = 1 - разрешаем прерывания по переднему фронту EXTI3
    */
    EXTI->RTSR |= EXTI_RTSR_TR0;

    /*
        EXTI_FTSR - регистр выбора заднего фронта прерывания
            0: Falling edge trigger disabled
            1: Falling edge trigger enabled

        TR3 = 0 - запрещаем прерывания по заднему фронту EXTI3
    */
//  EXTI->FTSR &= ~EXTI_FTSR_TR3;

    //	EXTI_PR — регистр ожидания
    EXTI->PR |= EXTI_PR_PR0;								// сброс флага прерывания путём записи "1"

    /*
        EXTI_IMR - регистр масок внешних прерываний
            0: Interrupt request from Line x is masked
            1: Interrupt request from Line x is not masked

        разрешаем прерывания EXTI3
    */
    EXTI->IMR |= EXTI_IMR_MR0;          					// разрешаем прерывания

    NVIC_SetPriority(EXTI0_IRQn, 3);    					// указываем приоритет прерываний
    NVIC_EnableIRQ(EXTI0_IRQn);         					// разрешаем прерывания
}