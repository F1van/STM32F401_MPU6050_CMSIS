#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


typedef struct
{
  // Постоянные времени и коэффициенты регулятора
  float Kp;
  float ti;
  float td;
  float t0;
  
  // Ошибки
  float error_k1;
  float error_k2;
  
  // Начальное состояние
  float state;
  
  // Вывод регулятора
  float out;
} PIDControll;

float PID_Controller(PIDControll *pid, float angle);
//PID pid_pitch;

#endif