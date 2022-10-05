// Настройки I2C

#include "i2c.h"

//=============================================================================
// Настройка линий I2C на альтернативную функцию AF4 с открытым стоком
// шина APB1 - 42 МГц
//---------------------------
// I2C1_SCL -> PB8 (PB6)
// I2C1_SDA -> PB9 (PB7)
//---------------------------
// I2C2_SCL -> PB10 (PF1/PH4)
// I2C2_SDA -> PB11 (PF0/PH5)
//---------------------------
// I2C3_SCL -> PA8 (PH7)
// I2C3_SDA -> PC9 (PH8)
//---------------------------

void I2C_port_init(I2C_TypeDef* I2Cx)
{
    if (I2Cx == I2C1) {
        /*
        	RCC_AHB1ENR - регистр включения устройств, подключеных к шине AHB1
        		I2CxEN = 1 - разрешаем тактирование I2Cx
        	включаем тактирование I2C1
        */
        RCC->APB1ENR |=	RCC_APB1ENR_I2C1EN;
        gpio_alt_func(GPIOB, 8, I2C_1, OPEN_DRAIN);		// настройка линий порта B на альтернативную функцию
        gpio_alt_func(GPIOB, 9, I2C_1, OPEN_DRAIN);
    } else if (I2Cx == I2C2) {
        RCC->APB1ENR |=	RCC_APB1ENR_I2C2EN;				// включаем тактирование I2C2
        gpio_alt_func(GPIOB, 10, I2C_2, OPEN_DRAIN);	// настройка линий порта B на альтернативную функцию
        gpio_alt_func(GPIOB, 3, I2C_2, OPEN_DRAIN);
    } else if (I2Cx == I2C3) {
        RCC->APB1ENR |=	RCC_APB1ENR_I2C3EN;				// включаем тактирование I2C3
        gpio_alt_func(GPIOA, 8, I2C_3, OPEN_DRAIN);		// настройка линий порта A и С на альтернативную функцию
        gpio_alt_func(GPIOB, 4, I2C_3, OPEN_DRAIN);
    }
    return;
}

//=============================================================================
// Инициализация I2C
void I2C_init(I2C_TypeDef* I2Cx)
{
    I2Cx->CR1 |= I2C_CR1_SWRST;								// программный сброс I2C
    I2Cx->CR1 &= ~I2C_CR1_SWRST;
    I2Cx->CR1 &= ~I2C_CR1_PE;								// выключаем модуль I2C
    I2Cx->CR1 &= ~I2C_CR1_SMBUS;							// настраиваем модуль в режим I2C
    // указываем частоту тактирования модуля в МГц
    I2Cx->CR2 &= ~I2C_CR2_FREQ;                 			// очистка значения частоты
    I2Cx->CR2 |= (uint16_t)(F_PCLK1 / 1000000UL);  			// F_PCLK1 [МГц]

    // записываем CCR
    // F/S  = 1 - I2C в режиме Fm (400 кГц)
    // DUTY = 1 - Tlow:Thigh = 16:9
    I2Cx->CCR &= ~I2C_CCR_CCR;                  			// очистка значения ССR
    if(F_SCL > 100000UL) {
        if(DUTY == 0)										// 400кГц; 1:2
            I2Cx->CCR = I2C_CCR_FS | ccr;
        else if(DUTY == 1)									// 400кГц; 9:16
            I2Cx->CCR = I2C_CCR_FS | I2C_CCR_DUTY | ccr;
    } else I2Cx->CCR = ccr;									// 100кГц

    I2Cx->TRISE = trise;                           			// максимальное время нарастания импульса TRISE[5:0]
    I2Cx->CR1 |= I2C_CR1_PE;								// включаем модуль
}

//=============================================================================
// Запуск I2C c адресом ведомого устройства (dev_addr)
void I2C_start(I2C_TypeDef* I2Cx, uint8_t dev_addr)
{
    // формируем режим СТАРТ
    I2Cx->CR1 |= I2C_CR1_START;				// генерируем состояние СТАРТ
    while(!(I2Cx->SR1 & I2C_SR1_SB))			// ждем установки флага SB = 1
        (void)I2Cx->SR1;					// чтение SR1 с последующей записью DR сбрасывают SB
    // передаем адрес устройства + режим записи
    I2Cx->DR = dev_addr << 1;					// передаем адрес ведомого для записи
    while(!(I2Cx->SR1 & I2C_SR1_ADDR))			// ждем установки флага ADDR = 1
        (void)I2Cx->SR1;					// читаем SR1 и SR2 для сброса ADDR
    (void)I2Cx->SR2;
}

//=============================================================================
// Отправка байта по I2C
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
    I2Cx->DR = data;
    while(!(I2Cx->SR1 & I2C_SR1_TXE))
        (void)I2Cx->SR1;
    (void)I2Cx->SR2;
}

//=============================================================================
// Отправка последнего байта по I2C
void I2C_write_last(I2C_TypeDef* I2Cx, uint8_t data)
{
    I2Cx->DR = data;
    while(!(I2Cx->SR1 & I2C_SR1_BTF))
        (void)I2Cx->SR1;
    (void)I2Cx->SR2;
}

//=============================================================================
// Повторный запуск I2C (ПОВСТАРТ) и передача адреса ведомого для чтения
void I2C_repeated_start(I2C_TypeDef* I2Cx, uint8_t dev_addr)
{
    I2Cx->CR1 |= I2C_CR1_START;								// генерируем состояние ПОВСТАРТ
    while(!(I2Cx->SR1 & I2C_SR1_SB))						// ждем установки флага SB = 1
        (void)I2Cx->SR1;										// чтение SR1 с последующей записью DR сбрасывают SB
    (void)I2Cx->SR2;
    I2Cx->DR = dev_addr << 1 | 1 << 0;							// передаем адрес ведомого для чтения
    while(!(I2Cx->SR1 & I2C_SR1_ADDR)) 					// ждем установки флага ADDR = 1
        (void)I2Cx->SR1;                   // читаем SR1 и SR2 для сброса ADDR
    (void)I2Cx->SR2;
}

//=============================================================================
// Прием байта по I2C с подтверждением
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
    I2Cx->CR1 |= I2C_CR1_ACK;
    while(!(I2Cx->SR1 & I2C_SR1_RXNE))
        (void)I2Cx->SR1;
    (void)I2Cx->SR2;
    uint8_t data = I2Cx->DR;
    return data;
}

//=============================================================================
// Прием байта по I2C без подтверждения
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
    I2Cx->CR1 &= ~I2C_CR1_ACK;
    while(!(I2Cx->SR1 & I2C_SR1_RXNE))
        (void)I2Cx->SR1;
    (void)I2Cx->SR2;
    uint8_t data = I2Cx->DR;
    return data;
}

//=============================================================================
// Запись байта (data) в ведомое устройство (dev_addr) без внутреннего адреса
void I2C_write_slave(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t data)
{
    // запуск I2C с адресом ведомого
    I2C_start(I2Cx, dev_addr);
    // пишем байт данных
    I2C_write_last(I2Cx, data);
    // останов I2C
    I2Cx->CR1 |= I2C_CR1_STOP;
    return;
}

//=============================================================================
// Запись байта (data) во внутренний регистр (reg_addr) ведомого устройства (dev_addr)

void I2C_write_reg(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    // запуск I2C с адресом ведомого
    I2C_start(I2Cx, dev_addr);
    // передаем адрес регистра
    I2C_write(I2Cx, reg_addr);
    // передаем байт данных
    I2C_write_last(I2Cx, data);
    // останов I2C
    I2Cx->CR1 |= I2C_CR1_STOP;
}
/*
void I2C_write_reg(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	// формируем режим СТАРТ
	I2Cx->CR1 |= I2C_CR1_START;								// генерируем состояние СТАРТ
	while(!(I2Cx->SR1 & I2C_SR1_SB));						// ждем установки флага SB = 1
//	(void)I2Cx->SR1;										// чтение SR1 с последующей записью DR сбрасывают SB
	// передаем адрес устройства + режим записи
	I2Cx->DR = dev_addr<<1;									// передаем адрес ведомого для записи
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)); 					// ждем установки флага ADDR = 1
//	(void)I2Cx->SR1;										// читаем SR1 и SR2 для сброса ADDR
	(void)I2Cx->SR2;
	while(!(I2Cx->SR1 & I2C_SR1_TXE));
	I2Cx->DR = reg_addr;
	while(!(I2Cx->SR1 & I2C_SR1_TXE));
	I2Cx->DR = data;
	while(!(I2Cx->SR1 & I2C_SR1_TXE));
	while(!(I2Cx->SR1 & I2C_SR1_BTF));
	// останов I2C
	I2Cx->CR1 |= I2C_CR1_STOP;
	return;
}
*/

//=============================================================================
// Запись нескольких байт (data[length]) начиная с внутреннего регистра (reg_addr) ведомого устройства (dev_addr)
void I2C_write_regs(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t length)
{
    // запуск I2C с адресом ведомого
    I2C_start(I2Cx, dev_addr);
    // передаем адрес регистра
    I2C_write(I2Cx, reg_addr);
    // отправка [length-1] байт данных
    for(uint8_t j = 0; j < length - 1; j++) I2C_write(I2Cx, data[j]);
    // отправка последнего байта данных
    I2C_write_last(I2Cx, data[length - 1]);
    // останов I2C
    I2Cx->CR1 |= I2C_CR1_STOP;
    return;
}

//=============================================================================
// Чтение байта (data) из внутреннего регистра (reg_addr) ведомого устройства (dev_addr)
uint8_t I2C_read_reg(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr)
{
    // запуск I2C с адресом ведомого для записи
    I2C_start(I2Cx, dev_addr);
    // отправка адреса регистра
    I2C_write(I2Cx, reg_addr);
    // повторный старт и передача адреса ведомого для чтения
    I2C_repeated_start(I2Cx, dev_addr);
    // прием байта данных
    uint8_t data = I2C_read_nack(I2Cx);
    // останов I2C
    I2Cx->CR1 |= I2C_CR1_STOP;
    return data;
}

//=============================================================================
// чтение нескольких байт (data[length]) начиная с внутреннего регистра (reg_addr) ведомого устройства (dev_addr)
void I2C_read_regs(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t length)
{
    // запуск I2C с адресом ведомого для записи
    I2C_start(I2Cx, dev_addr);
    // отправка адреса регистра
    I2C_write(I2Cx, reg_addr);
    // повторный старт и передача адреса ведомого для чтения
    I2C_repeated_start(I2Cx, dev_addr);
    // прием [length-1] байт данных
    for(uint8_t j = 0; j < length - 1; j++) data[j] = I2C_read_ack(I2Cx);
    // прием последнего байта данных
    data[length - 1] = I2C_read_nack(I2Cx);
    // останов I2C
    I2Cx->CR1 |= I2C_CR1_STOP;
    return;
}

void I2C_write_bits(I2C_TypeDef* I2Cx, uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t start_bit, uint8_t data)
{
    I2C_start(I2Cx, dev_addr);
    I2C_write(I2Cx, reg_addr);
    I2C_repeated_start(I2Cx, dev_addr);
    uint8_t temp = I2C_read_nack(I2Cx) & ~(((1 << length) - 1) << start_bit);
    I2C_start(I2Cx, dev_addr);
    I2C_write(I2Cx, reg_addr);
    I2C_write_last(I2Cx, temp | data << start_bit);
    I2Cx->CR1 |= I2C_CR1_STOP;
}












