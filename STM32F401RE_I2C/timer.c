/* 	Настройки таймеров

	tim_3, tim_4 - 16-bit
	tim_2, tim_5 - 32-bit

	tim_1, 8...11 - APB2 (168 МГц)
	tim_2...7, 12...14 - APB1 (84 МГц)
*/

#include "timers.h"

//=============================================================================
// инициализация таймера 1 (16 бит)
// (производит отсчёт времени)
void timer_1_init(void)
{
//	NVIC_SetPriority(TIM1_IRQn, 1);							// установка приоритета (0...15)
//	NVIC_EnableIRQ(TIM1_IRQn);								// TIM1 global Interrupt
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;						// разрешаем тактирование таймера 1
    /*
    	TIMx_PSC - регистр предделителя тактовой частоты
    		Bits 15:0 PSC[15:0]: предделитель
    			fCK_PSC / (PSC[15:0] + 1).
    	При максимальной частоте APB2 = 168 МГц, и предделителе 168 имеем
    	частоту тактирования таймера 1 МГц (период 1 мкс).
    */
    TIM1->PSC = 168UL - 1UL;
    /*
    	TIMx_ARR - модуль счёта
    		Bits 15:0 ARR[15:0]: модуль счёта
    	При частоте тактирования 1 МГц и модуле счёта 0xFFFF имеем период счёта 1 мкс...65536 мкс
    	Таймер не работает, если ARR = 0.
    */
    TIM1->ARR = 65535;

    TIM1->CR1 |= TIM_CR1_OPM;								// режим одного импульса
//	TIM1->DIER |= TIM_DIER_UIE;								// разрешаем прерывания по переполнению

    TIM1->CNT = 0;											// сброс счётного регистра
//	TIM1->CR1 |= TIM_CR1_CEN;								// разрешаем работу таймера

    return;
}

//=============================================================================
// инициализация таймера 2 (32 бита)
// (отсчитывает интервал времени и выдает прерывание по переполнению)
void timer_2_init(void)
{
    NVIC_EnableIRQ(TIM2_IRQn);								// TIM2 global Interrupt
    NVIC_SetPriority(TIM2_IRQn, 1);							// установка приоритета (0...15)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;						// разрешаем тактирование таймера 2
//	TIM2->SMCR &= ~TIM_SMCR_SMS;							// включение внутреннего тактирования
    /*
    	TIMx_PSC - регистр предделителя тактовой частоты
    		Bits 15:0 PSC[15:0]: предделитель
    			fCK_PSC / (PSC[15:0] + 1).
    	При максимальной частоте APB1 = 84 МГц, и предделителе 83 имеем
    	частоту тактирования таймера 1 МГц (период 1 мкс).
    */
    TIM2->PSC = 83;
    /*
    	TIMx_ARR - модуль счёта
    		Bits 15:0 ARR[15:0]: модуль счёта
    	При частоте тактирования 1 МГц и модуле счёта 10000 имеем период счёта = 10 мс
    	Таймер не работает, если ARR = 0.
    */
    TIM2->ARR = 10000;

    TIM2->DIER |= TIM_DIER_UIE;				// разрешаем прерывания по переполнению
    Timer_Enable(TIM2);					// разрешаем работу таймера	2

    return;
}

//=============================================================================
// инициализация таймера 5 (32 бита)
// (производит отсчёт времени)
void timer_5_init(void)
{
//	NVIC_SetPriority(TIM5_IRQn, 1);					// установка приоритета (0...15)
//	NVIC_EnableIRQ(TIM5_IRQn);					// TIM2 global Interrupt
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;				// разрешаем тактирование таймера
    /*
    	TIMx_PSC - регистр предделителя тактовой частоты
    		Bits 15:0 PSC[15:0]: предделитель
    			fCK_PSC / (PSC[15:0] + 1).
    	При максимальной частоте APB2 = 84 МГц, и предделителе 84 имеем
    	частоту тактирования таймера 1 МГц (период 1 мкс).
    */
    TIM5->PSC = 84UL - 1UL;
    /*
    	TIMx_ARR - модуль счёта
    		Bits 15:0 ARR[15:0]: модуль счёта
    	При частоте тактирования 1 МГц и модуле счёта 0xFFFF имеем период счёта 1 мкс...65536 мкс
    	Таймер не работает, если ARR = 0.
    */
    TIM5->ARR = 0xFFFFFFFF;
    /*
    	TIMx_CR1 - TIMx control register 1
    		ARPE: 	Auto-reload preload enable
    		OPM: 	One-pulse mode
    		UDIS: 	Update disable
    		URS: 	Update request source
    */
//	TIM5->CR1 |= TIM_CR1_OPM;					// режим одного импульса (исключает дребезг 1 мл. разряда)
//	TIM5->CR1 |= TIM_CR1_ARPE;					// обновление ARR на вершине счёта
//	TIM5->CR1 |= TIM_CR1_UDIS;
//	TIM5->CR1 |= TIM_CR1_URS;
    /*
    	TIMx_EGR - TIMx event generation register
    		UG: Update generation
    			This bit can be set by software, it is automatically cleared by hardware
    */
//	TIM5->EGR |= TIM_EGR_UG;					// ручное обновление регистров

//	TIM5->DIER |= TIM_DIER_UIE;					// разрешаем прерывания по переполнению
//	Timer_Clear(TIM5);						// сброс счётного регистра
    Timer_Enable(TIM5);						// разрешаем работу таймера
    return;
}

//=============================================================================
// Инициализация энкодера на таймере 3 (16 бит)
//
// TIM3_CH1 -> PC6 (PB4/PA6)
// TIM3_CH2 -> PC7 (PB5/PA7)
// TIM3_CH3 -> PC8 (PB0)
// TIM3_CH4 -> PC9 (PB1)

void enc_tim_init(void)
{
    gpio_alt_func(GPIOC, 6, TIM_3, OPEN_DRAIN);			// инициализация порта энкодера (TIM3_CH1/TIM3_CH2)
    gpio_alt_func(GPIOC, 7, TIM_3, OPEN_DRAIN);
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;				// разрешаем тактирование таймера 3

    /*
    	TIMx_PSC - регистр предделителя тактовой частоты
    		Bits 15:0 PSC[15:0]: предделитель
    			fCK_PSC / (PSC[15:0] + 1).
    	При максимальной частоте APB1 = 84 МГц, и предделителе 0 имеем
    	частоту тактирования таймера 84 МГц.
    */
    TIM3->PSC = 0;

    /*
    	TIMx_ARR - модуль счёта
    		Bits 15:0 ARR[15:0]: модуль счёта
    	При частоте тактирования 1 МГц и модуле счёта 65535 имеем период счёта 1 мкс...65536 мкс
    	Таймер не работает, если ARR = 0.
    */
    TIM3->ARR = 65535;

    TIM3->CCMR1 |= TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;			// фильтр на входе захвата таймера F_sampling = F_dts/32, N = 8
    TIM3->CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P;			// полярность неинвертирована

    /*
    	CC2S[1:0]:
    		00: CC2 channel is configured as output
    		01: CC2 channel is configured as input, IC2 is mapped on TI2
    		10: CC2 channel is configured as input, IC2 is mapped on TI1
    		11: CC2 channel is configured as input, IC2 is mapped on TRC. This mode is working only if
    			an internal trigger input is selected through the TS bit (TIMx_SMCR register)
    	CC1S[1:0]:
    		00: CC1 channel is configured as output.
    		01: CC1 channel is configured as input, IC1 is mapped on TI1.
    		10: CC1 channel is configured as input, IC1 is mapped on TI2.
    		11: CC1 channel is configured as input, IC1 is mapped on TRC.
    	Note: CCxS bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER).
    */
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);		// настраиваем мультиплексор (для первого и второго входа)
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

    /*
    	TIMx_SMCR:
    		SMS = 001 counting on TI2 edges only;
    		SMS = 010 counting on TI1 edges only;
    		SMS = 011 counting on both TI1 and TI2 edges.
    */
    TIM3->SMCR &= ~TIM_SMCR_SMS;					// сброс тактирования
    TIM3->SMCR |= TIM_SMCR_SMS_0;					// Encoder mode 1

//	NVIC_EnableIRQ(TIM3_IRQn);					// TIM3 global Interrupt
//	NVIC_SetPriority(TIM3_IRQn, 15);				// установка самого низкого приоритета (0...15)
//	TIM3->DIER |= TIM_DIER_UIE;					// разрешаем прерывания по переполнению

    TIM3->CR1 |= TIM_CR1_CEN;					// разрешаем работу таймера	3
    return;
}


//=============================================================================
// Инициализация 4-канального ШИМ на таймере 2 (APB1 - 84 МГц)
//
// TIM2_CH1 -> PA5
// TIM2_CH2 -> PA1
// TIM2_CH3 -> PA2
// TIM2_CH4 -> PA3
//
void TIM2_4CH_PWM_init(uint16_t prescaler, uint32_t period, uint32_t compare)
{
    gpio_alt_func(GPIOA, 1, TIM_2, PUSH_PULL);			// настройка линий порта A на альтернативную функцию
    gpio_alt_func(GPIOA, 2, TIM_2, PUSH_PULL);
    gpio_alt_func(GPIOA, 3, TIM_2, PUSH_PULL);
    gpio_alt_func(GPIOA, 5, TIM_2, PUSH_PULL);

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;				// разрешение тактирования таймера 2
    TIM2->CR1 = 0x0000;						// остановка таймера 2
    TIM2->PSC = (uint16_t)(prescaler - 1UL);			// fCK_PSC / (PSC[15:0] + 1) - предделитель тактовой частоты
    TIM2->ARR = period;						// TIMx_ARR - модуль счёта
    TIM2->CR1 |= TIM_CR1_ARPE;					// обновление модуля на вершине счёта
    TIM2->CCER = 0x0000;					        // запрет работы каналов на выход

    // TIMx_CCRy - регистры сравнения
    TIM2->CCR1 = compare;
    TIM2->CCR2 = compare;
    TIM2->CCR3 = compare;
    TIM2->CCR4 = compare;

    // TIMx_CCMRn - режим выхода компаратор таймера PWM Mode 1
    TIM2->CCMR1 = 0x0000;
    TIM2->CCMR2 = 0x0000;
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |
                   TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 |
                   TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;

    // обновление регистра сравнения на вершине счёта
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    // TIM_CCER_CCyE - разрешение работы каналов на выход
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E |
                  TIM_CCER_CC3E | TIM_CCER_CC4E;
    /*
    	TIMx_CR1 - регистр 1 управления таймером
    		CMS: выбор режима выравнивания
    			00: выравнивание по фронту
    		DIR: направление счёта
    			0: счёт вверх
    			1: счёт вниз
    		CEN: включение таймера
    			0: выключен
    			1: включен
    */
//	TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);  			// считаем вверх, выравнивание по фронту (Fast PWM)
    /*
    	TIMx_EGR - TIMx event generation register
    		UG: Update generation
    */
    TIM2->EGR |= TIM_EGR_UG;					// запись значений предзагрузки и сравнения
    TIM2->CR1 |= TIM_CR1_CEN;					// запуск таймера
    return;
}

//=============================================================================
// Инициализация выхода ШИМ таймера X (APB1 - 84 МГц)
//
// channel - номер канала таймера
// prescaler - значение предделителя частоты
// period - модуль счёта таймера (период)
// compare - константа сравнения компаратора таймера
//
void PWM_OCx_Init(TIM_TypeDef* TIMx, uint8_t channel, uint16_t prescaler, uint32_t period, uint32_t compare)
{
    if(TIMx == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;		// разрешение тактирования таймеров
    else if(TIMx == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if(TIMx == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if(TIMx == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if(TIMx == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    TIMx->CR1 &= ~TIM_CR1_CEN;						// остановка таймера
    TIMx->PSC = (uint16_t)(prescaler - 1UL);		// fCK_PSC / (PSC[15:0] + 1) - предделитель тактовой частоты
    TIMx->ARR = period;								// TIMx_ARR - модуль счёта
    TIMx->CR1 |= TIM_CR1_ARPE;						// обновление модуля на вершине счёта

    if(channel == 1) {
        TIMx->CCER &= ~TIM_CCER_CC1E;				// запрет работы канала 1 на выход
        TIMx->CCR1 = compare;						// TIMx_CCRy - регистр сравнения канала Y
        TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;				// TIMx_CCMRn - режим выхода компаратор таймера PWM Mode 1
        TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
        TIMx->CCER |= TIM_CCER_CC1E;				// TIM_CCER_CCyE - разрешение работы канала 1 на выход
    } else if(channel == 2) {
        TIMx->CCER &= ~TIM_CCER_CC2E;
        TIMx->CCR2 = compare;
        TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;
        TIMx->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
        TIMx->CCER |= TIM_CCER_CC2E;
    } else if(channel == 3) {
        TIMx->CCER &= ~TIM_CCER_CC3E;
        TIMx->CCR3 = compare;
        TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;
        TIMx->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
        TIMx->CCER |= TIM_CCER_CC3E;
    } else if(channel == 4) {
        TIMx->CCER &= ~TIM_CCER_CC4E;
        TIMx->CCR4 = compare;
        TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;
        TIMx->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
        TIMx->CCER |= TIM_CCER_CC4E;
    }
    /*
    	TIMx_CR1 - регистр 1 управления таймером
    		CMS: выбор режима выравнивания
    			00: выравнивание по фронту
    		DIR: направление счёта
    			0: счёт вверх
    			1: счёт вниз
    		CEN: включение таймера
    			0: выключен
    			1: включен
    */
    TIMx->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);  	// считаем вверх, выравнивание по фронту (Fast PWM)
    /*
    	TIMx_EGR - TIMx event generation register
    		UG: Update generation
    */
    TIMx->EGR |= TIM_EGR_UG;						// запись значений предзагрузки и сравнения
    TIMx->CR1 |= TIM_CR1_CEN;						// запуск таймера
    return;
}

void TIM_PWM_CAPTURE(TIM_TypeDef* TIMx, uint8_t channel, uint16_t prescaler, uint32_t period)
{
    if(TIMx == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;		// разрешение тактирования таймеров
    else if(TIMx == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if(TIMx == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if(TIMx == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if(TIMx == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;


    TIMx->CR1 &= ~TIM_CR1_CEN;				           // остановка таймера
    TIMx->PSC = (uint16_t)(prescaler - 1UL);		          // fCK_PSC / (PSC[15:0] + 1) - предделитель тактовой частоты
    TIMx->ARR = period;					 	 // TIMx_ARR - модуль счёта
    TIMx->CR1 |= TIM_CR1_ARPE;				        // обновление модуля на вершине счёта

    if(channel == 1) {
        TIMx->CCMR1 |= TIM_CCMR1_CC1S_0;		      // TIMx_CCMRn - режим выхода компаратор таймера PWM Mode 1
        TIMx->CCMR1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC);
        TIMx->CCER &= ~TIM_CCER_CC1P;
        TIMx->CCER |= TIM_CCER_CC1E;		             // TIM_CCER_CCyE - разрешение работы канала 1 на в[jl
        TIMx->DIER |= TIM_DIER_CC1IE;
    } else if(channel == 2) {
        TIMx->CCMR1 |= TIM_CCMR1_CC2S_0;		      // TIMx_CCMRn - режим выхода компаратор таймера PWM Mode 1
        TIMx->CCMR1 &= ~(TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC);
        TIMx->CCER &= ~TIM_CCER_CC2P;
        TIMx->CCER |= TIM_CCER_CC2E;		             // TIM_CCER_CCyE - разрешение работы канала 1 на в[jl
        TIMx->DIER |= TIM_DIER_CC2IE;
    } else if(channel == 3) {
        TIMx->CCMR2 |= TIM_CCMR2_CC3S_0;		      // TIMx_CCMRn - режим выхода компаратор таймера PWM Mode 1
        TIMx->CCMR2 &= ~(TIM_CCMR2_IC3F | TIM_CCMR2_IC3PSC);
        TIMx->CCER &= ~TIM_CCER_CC3P;
        TIMx->CCER |= TIM_CCER_CC3E;		             // TIM_CCER_CCyE - разрешение работы канала 1 на в[jl
        TIMx->DIER |= TIM_DIER_CC3IE;
    } else if(channel == 4) {
        TIMx->CCMR2 |= TIM_CCMR2_CC4S_0;		      // TIMx_CCMRn - режим выхода компаратор таймера PWM Mode 1
        TIMx->CCMR2 &= ~(TIM_CCMR2_IC4F | TIM_CCMR2_IC4PSC);
        TIMx->CCER &= ~TIM_CCER_CC4P;
        TIMx->CCER |= TIM_CCER_CC4E;		             // TIM_CCER_CCyE - разрешение работы канала 1 на в[jl
        TIMx->DIER |= TIM_DIER_CC4IE;
    }
    /*
    	TIMx_CR1 - регистр 1 управления таймером
    		CMS: выбор режима выравнивания
    			00: выравнивание по фронту
    		DIR: направление счёта
    			0: счёт вверх
    			1: счёт вниз
    		CEN: включение таймера
    			0: выключен
    			1: включен
    */
    TIMx->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);  	// считаем вверх, выравнивание по фронту (Fast PWM)
    /*
    	TIMx_EGR - TIMx event generation register
    		UG: Update generation
    */
    TIMx->EGR |= TIM_EGR_UG;						// сброс регистров таймера
    TIMx->CR1 |= TIM_CR1_CEN;						// запуск таймера
}
