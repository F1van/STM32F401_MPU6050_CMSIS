#include "USART.h"

// Инициализация USART_1
void USART1_init(void)
{
    /*
      TX1 -> PA9
      RX1 -> PA10 - при передаче не используется

      USART_1, USART_6 подключены к APB2
      USART_2...USART_5 подключены к APB1

      RCC_APB2ENR - регистр включения устройств, подключеных к шине AHB2 (USART_1)
      RCC_AHB1ENR - регистр включения устройств, подключеных к шине AHB1
    */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // включаем тактирование UART1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // включаем тактирование порта А

    /*
      AFR[0] – записывает значение в регистр GPIOx_AFRL
      AFR[1] – записывает значение в регистр GPIOx_AFRH
      альтернативная ф-ция 0111 - AF7(USART1)
    */
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10); // сброс в 0x0000
    GPIOA->AFR[1] |= 0x7UL << GPIO_AFRH_AFSEL9_Pos | 0x7UL << GPIO_AFRH_AFSEL10_Pos;

    /*
      GPIOx_MODER - регистр настройки режимов порта X
      00: Input (reset state)
      01: General purpose output mode
      10: Alternate function mode
      11: Analog mode

      MODERy[1:0] = 10 - линия Y порта X в альтернативном режиме
    */
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
    GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;

    /*
      GPIOx_OTYPER - настройка схемы выхода порта в режимах ВЫХОД или АЛЬТ.ФУНКЦИЯ
      0: Output push-pull (reset state)
      1: Output open-drain
      в данном случае линии работают в режиме push-pull
    */
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10);

    /*
      GPIOx_OSPEEDR - скорость портов в режимах ВЫХОД или АЛЬТ.ФУНКЦИЯ
      00: Low speed (400 кГц)
      01: Medium speed (2 МГц)
      10: High speed (10 МГц)
      11: Very high speed (50 МГц)
      в данном случае выбранные линии работают на частоте 50 МГц
    */
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;

    /*
      GPIOx_PUPDR - регистр настройки подтягивания входов
      00: No pull-up, pull-down
      01: Pull-up
      10: Pull-down
      11: Reserved
      в данном случае выбранные линии не подтягивают
    */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10);

    /*
      USART_BRR - регистр скорости UART
      USARTDIV = F_CPU / (8 * (2 - OVER8) * BAUD) = 72000000 / (8 * (2 - 0) * 115200) = 39,0625
      целая часть делителя = 39 = 0x27
      дробная часть делителя = 16 * 0,0625 = 0x1
    */
    USART1->BRR = 0x00000271;

    /*
      USARTx_CR1 - регистр управления 1
      UE: разрешение USART
      M: длина слова (0: 1 Start bit, 8 Data bits, n Stop bit)
      WAKE: Wakeup method
      PCE: разрешение контроля четности (0: без контроля четности)
      PS: тип контроля чётности
      PEIE: разрешение прерывания при ошибке четности
      TXEIE: разрешение прерывания при опустошении буфера передачи данных
      TCIE: разрешение прерываний по окончании передачи данных
      TE: разрешение передачи
      RE: разрешение приема

    */
    USART1->CR1 = USART_CR1_UE | // разрешение USART_1
                  USART_CR1_TE; // передача USART_1

    /*
      USARTx_CR2 - регистр управления 2
      STOP: STOP bits (00: 1 Stop bit);
    */
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    return;
}

void float_to_usart(float data)
{
    uint32_t* ptr = (uint32_t*) &data;
    for (uint8_t j = 0; j < 4; j++) {
        while((USART1->SR & USART_SR_TXE) == 0);  // ждем установки флага очистки буфера
        USART1->DR = (*ptr) >> (8 * j) & 0x000000ff; // выводим очередное значение фазы А
    }
}

void int32_to_usart_1(uint32_t data)
{
    for (uint8_t j = 0; j < 4; j++) {
        while((USART1->SR & USART_SR_TXE) == 0);			// ждем установки флага очистки буфера
        USART1->DR = data >> (8 * j) & 0x000000ff;			// выводим очередное значение фазы А
    }
}