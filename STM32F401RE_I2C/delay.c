// Задержки на SysTick

#include "delay.h"

//=============================================================================
// задержка на SysTick в [мс] без прерываний
void _delay_ms(uint32_t set_value)
{
    SysTick->LOAD = TOP_ms;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |	                // SysTick тактируется частотой процессора (AHB)
                    SysTick_CTRL_ENABLE_Msk;	                  	// разрешена работа
    uint32_t current_value = 0;
    while (current_value < set_value) {
        //while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));		// ожидаем установки флага прерывания от SysTick
        current_value++;											// инкремент текущего значения счётчика
        SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;				// сброс флага SysTick
    }
    SysTick->CTRL = 0;
    return;
}

//=============================================================================
// задержка на SysTick в [мкс] без прерываний
void _delay_us(uint32_t set_value)
{
    uint32_t current_value = 0;
    SysTick->LOAD = TOP_us;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |					// тактируется частотой процессора (AHB)
                    SysTick_CTRL_ENABLE_Msk;						// разрешена работа
    while (current_value < set_value) {
        //while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));	    // ожидаем установки флага прерывания от SysTick
        current_value++;				                			// инкремент текущего значения счётчика
        SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;		        // сброс флага SysTick
    }
    SysTick->CTRL = 0;
    return;
}


