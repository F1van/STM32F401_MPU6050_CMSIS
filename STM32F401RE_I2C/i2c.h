// Настройки I2C

#ifndef I2C_H_
#define I2C_H_


//=============================================================================
// Библиотеки 
#include "stm32f4xx.h"
#include "gpio.h"
#include "delay.h"

// Чтобы использовать I2C на частоте 400 кГц в режиме Fm
// частота PCLK1 должна быть кратна 10 МГц!

// Cтруктура параметров шины I2C
// *typedef создает новое имя (i2c_options_t) для типа struct
/*
typedef struct
{
	uint32_t I2C_F_PCLK;								// тактовая частота I2C
	uint32_t I2C_F_SCL;									// скорость шины I2C
	uint8_t I2C_Mode;									// 
	uint8_t I2C_DutyCycle;								// 
} I2C_Opt_TypeDef;
*/

#define F_PCLK1	72000000UL												// для HSE = 8 МГц и PLL = 168 МГц
#define F_SCL	400000UL												// частота SCL [Гц]
#define DUTY 	1														// 0 - 2:1; 1 - 16:9

#if F_SCL > 100000UL													// < 400кГц
	#if DUTY == 0														// 2:1
		#define ccr		(uint16_t)(F_PCLK1/(3UL*F_SCL))					// CCR = Fpclk1/(3*Fscl), при Tlow:Thigh = 2:1
		#define trise	(uint16_t)(F_PCLK1/1000000UL*300UL/1000UL+1UL)	// максимальное время нарастания импульса
																		// TRISE[5:0] =  Tr max[нс] / Tpclk1[нс] + 1
																		// Tr max = 300 нс
	#elif DUTY == 1														// 16:9	
		#define ccr		(uint16_t)(F_PCLK1/(25UL*F_SCL))				// CCR = Fpclk1/(25*Fscl), при Tlow:Thigh = 16:9
		#define trise	(uint16_t)(F_PCLK1/1000000UL*300UL/1000UL+1UL)	// Tr max = 300 нс
	#endif																
#else																	// < 100кГц
	#define ccr		(uint16_t)(F_PCLK1/(2UL*F_SCL))						// CCR = Fpclk1/(2*Fscl)	
	#define trise	(uint16_t)(F_PCLK1/1000000UL+1UL)					// Tr max = 1000 нс
#endif

// прототипы функций ==========================================================
void I2C_port_init(I2C_TypeDef* I2Cx);				// инициализация gорта I2Cx
void I2C_init(I2C_TypeDef* I2Cx);    				// инициализация I2Cx     								
void I2C_start(I2C_TypeDef* I2Cx, uint8_t dev_addr);
													// запуск I2C c адресом ведомого устройства (dev_addr)
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);	// отправка байта по I2C
void I2C_write_last(I2C_TypeDef* I2Cx, uint8_t data);
													// отправка gоследнего байта по I2C
void I2C_repeated_start(I2C_TypeDef* I2Cx, uint8_t dev_addr);
													// повторный запуск I2C (ПОВСТАРТ) и передача адреса ведомого для чтения
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);			// прием байта по I2C с подтверждением 
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);			// прием байта по I2C без подтверждения

void I2C_write_bits(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t start_bit, uint8_t data);
													// запись нескольких бит (data) в регистр (reg_addr) начиная с бита (start_bit) без изменения остальных бит
void I2C_write_slave(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t data);
													// запись байта (data) в ведомое устройство (dev_addr) без внутреннего адреса
void I2C_write_reg(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
													// запись байта (data) во внутренний регистр (reg_addr) ведомого устройства (dev_addr)
void I2C_write_regs(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t length);
													// запись нескольких байт (data[length]) начиная с внутреннего регистра (reg_addr) ведомого устройства (dev_addr)
uint8_t I2C_read_reg(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr);
													// чтение байта (data) из внутреннего регистра (reg_addr) ведомого устройства (dev_addr)
void I2C_read_regs(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t length);
													// чтение нескольких байт (data[length]) начиная с внутреннего регистра (reg_addr) ведомого устройства (dev_addr)

#endif /* I2C_H_ */



