// Подключение ЖК-дисплея 1602 по интерфейсу I2C

#ifndef LCD1602_I2C_H_
#define LCD1602_I2C_H_

//=============================================================================
// Библиотеки 
#include "stm32f4xx.h"
#include "delay.h"
#include "i2c.h"

//=============================================================================
// Распиновка PCF8574 модуля LCD_I2C
#define RS  				0								// команда/данные
#define RW  				1								// запись/чтение
#define E   				2								// импульс записи
#define P3  				3								// управление подсветкой
#define D4  				4								// данные
#define D5  				5								// данные
#define D6  				6								// данные
#define D7  				7								// данные
#define LCD1602_addr		0x27							// адрес ЖК-дисплея
#define LCD1602_write_addr	LCD1602_addr<<1					// адрес ЖК-дисплея для записи
#define LCD1602_read_addr	LCD1602_addr<<1|1<<0			// адрес ЖК-дисплея для чтения
extern char LCD_buff[16];						// буфер для ЖК-дисплея

//=============================================================================
// Прототипы функций LCD1602
void LCD_init(void);										// инициализация дисплея 
void send_half_byte(uint8_t byte);							// отправка полубайта
void send_com(uint8_t com);									// отправка команды
void send_data(uint8_t data);								// отправка данных
void LCD_pos(uint8_t y, uint8_t x);							// позиция на дисплее
void LCD_send_string(char* str);							// отправка строки 


#endif /* LCD1602_I2C_H_ */