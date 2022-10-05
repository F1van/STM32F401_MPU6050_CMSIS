// Таймеры общего назначения

#ifndef SET_CLOCK_H_
#define SET_CLOCK_H_

#include "stm32f4xx.h"
/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M      8
#define PLL_N      216

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      6

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      9

// прототипы функций ==========================================================
void SetSysClock(void);


#endif /* SET_CLOCK_H_ */