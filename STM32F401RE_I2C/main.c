/*
    STM32F401RE (NUCLEO-64)
    Интерфейс I2C
*/

#include <stdio.h>											// стандартная библиотека ввода/вывода Си
#include "stm32f4xx.h"
#include "gpio.h"
#include "delay.h"
#include "i2c.h"
#include "LCD1602_I2C.h"
#include "MPU6050.h"
#include "EXTI.h"
#include "USART.h"
#include "timers.h"
#include "SET_CLOCK.h"
#include "PID.h"
// Глобальные переменные
#define  M_PI  3.1415926535897932384626433f
#define  Kf    0.05f    									// коэффициент комплементарного фильтра
#define  DEG   180.f/M_PI									// коэффициент перевода в градусы
#define  RAD   M_PI/180.f									// коэффициент перевода в радианы
char LCD_buff[16];
volatile  float PID_Kp = 1;
volatile  float PID_ti = 0.01;
volatile  float PID_td = 0.01;
volatile  float PID_t0 = 0.01;

// Ошибки
float PID_error_k1 = 0;
float PID_error_k2 = 0;

// Начальное состояние
float PID_state = 0;

// Вывод регулятора
float PID_out = 0;

// Основная программа =========================================================
int main()
{
    __disable_irq();
    SetSysClock();
    I2C_port_init(I2C1);
    I2C_init(I2C1);
    //I2C_port_init(I2C3);
    //I2C_init(I2C3);
    LCD_init();
    LCD_pos(1, 1);
    LCD_send_string("                ");
    LCD_pos(2, 1);
    LCD_send_string("                ");
    LCD_pos(1, 1);
    LCD_send_string("LCD init");
    //EXTI_init();
    timer_5_init();
    gpio_alt_func(GPIOA, 0, 0001, PUSH_PULL);
    TIM_PWM_CAPTURE(TIM2, 1, 720, 0xffff);
    TIM2->SR = 0x00;
    USART1_init();

    read_MPU_ID();
    MPU6050_reset_wakeup();
    MPU6050_self_test();
    MPU_calibrate();
    MPU6050_init();
    NVIC_EnableIRQ(TIM2_IRQn);
    //DMP2_init();				// инициализация DMP MotionApp_v2
    //DMP6_init();				// инициализация DMP MotionApp_v6


    // разрешаем DMP
    //I2C_write_bits(I2C1, MPU6050_addr, USER_CTRL, 1, DMP_EN_BIT, 1);
    // сброс FIFO
    //I2C_write_bits(I2C1, MPU6050_addr, USER_CTRL, 1, FIFO_RESET, 1);
    __enable_irq();



    /*
        uint8_t data = I2C_read_reg(I2C1, 0x68, 0x75);
        LCD_pos(1, 1);
        sprintf(LCD_buff, "0x%02X      ", data);
        LCD_send_string(LCD_buff);
    */
    while (1) {

    }
}


// Функции ====================================================================
// обработчики прерываний
/*
void EXTI0_IRQHandler(void)
{

       	uint32_t time = TIM5->CNT;		// сохранение счёта таймера 5
	TIM5->EGR |= TIM_EGR_UG;			// запуск таймера 5
	sprintf(LCD_buff, "%05i  ", time);
	LCD_pos(2, 1);
	LCD_send_string(LCD_buff);

        // проверяем число байт в FIFO (42/28)
	uint8_t data_arr[2];
	I2C_read_regs(I2C2, MPU6050_addr, FIFO_COUNT_H, data_arr, 2);
	uint16_t FIFO_cnt = data_arr[0]<<8 | data_arr[1];

	sprintf(LCD_buff, "%i", FIFO_cnt);
	LCD_pos(1, 1);
	LCD_send_string(LCD_buff);


	// извлекаем кватернионы из FIFO_buff
	float q[4];
	DMP2_get_quat_32(q);

	// преобразуем кватернион в проекции вектора силы тяжести
	float Ax = 2 * (q[1]*q[3] - q[0]*q[2]);
//	float Ay = 2 * (q[0]*q[1] + q[2]*q[3]);
	float Az = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

         uint8_t data[40];
	// читаем 42/28 (DMP_packet_size) байта из FIFO в массив data[]
	// ... для ускоррения процесса читаем только первые 16 байт...
	I2C_read_regs(I2C2, MPU6050_addr, FIFO_R_W, data, 40);

	// считаем тангаж (поворот вокруг оси Y)
	float pitch = DEG * atan2f(Ax, Az);
	// выводим график тангажа на плоттер
	float_to_usart(pitch);


	// сброс FIFO
	I2C_write_bits(I2C1, MPU6050_addr, USER_CTRL, 1, FIFO_RESET, 1);
        // сброс флага прерывания MPU6050 путем чтения регистра INT_STATUS
	I2C_read_reg(I2C1, MPU6050_addr, INT_STATUS);

	// сбрасываем флаг прерывания записю 1
        EXTI->PR |= EXTI_PR_PR0;
        asm("nop");				// сброс флага прерывания путём записи "1"
}
*/
void TIM2_IRQHandler()
{
    signal_repository = TIM2->CCR1;

    int32_to_usart_1(signal_repository);

    _delay_ms(1000);
}


void angle_adjustment(float angle, uint32_t time, float setpoint)
{
    /* Initialise PID controller */

    //int32_t throttle = 126000; // текущее значение ШИМ
    PIDControll pid = {PID_Kp, PID_ti, PID_td, PID_t0, PID_error_k1, PID_error_k2, PID_state, PID_out};
    PID_Controller(&pid, angle);
    //int32_t force = 84000 * (int32_t)pid.out;

    // Получаем коэффициент ПИД-регулятора
    if (angle > 0) {
        /*
          Предположительно отколениен от нормального угла 0 будет +45(градусов) по крену.
          В таком случае необходимо отправить это отколенение в PID регулятор и на выходе получить ошибку умноженную на коэффициент
          Этот коэффициент умножить на 84000000 и в нашем случае крена +45, прибавить эту величину к текущему значению
          частоты шим двигателей слева(при регулировании крена), а отнять от частоты шим двигателей справа(при регулировании крена)
          Throttle - текущий шим сигнал (мс)
          force -реакция квадрокоптера на отклонение
          force = 1/((PID_out/90) * 84000000) (мс)
          Signal_M1 = Throttle - force
          Signal_M2 = Throttle + force
        */
        // Подаем шим сигнал на задние двигатели throttle + force(84000000 * (выходную величину с ПИД-регулятор/90))
        // Подаем шим сигнал на передние двигатели throttle - force(Текущий Шим сигнал на двигателе * выходную величину с ПИД-регулятор)
        // Крен
        //uint32_t Signal_M1 =  throttle + force;
        //uint32_t Signal_M2 = throttle - force;
        // Настройка скважности обоих двигателей
    } else if (angle < 0) {
        // Крен = -30
        // Подаем шим сигнал на задние двигатели throttle - force(Текущий Шим сигнал на двигателе * выходную величину с ПИД-регулятор)
        // Подаем шим сигнал на передние двигатели throttle + force(Текущий Шим сигнал на двигателе * выходную величину с ПИД-регулятор)
        /*
          Throttle - текущий шим сигнал (мс)
          force -реакция квадрокоптера на отклонение
          force = 1/((PID_out/-90) * 84000000)
          M1 - левый мотор
          M2 - правый мотор
          Signal_M1 = Throttle + force
          Signal_M2 = Throttle - force
        */
        //uint32_t Signal_M1 =  throttle + force;
        //uint32_t Signal_M2 = throttle - force;
        // Настройка скважности обоих двигателей
    }


}
