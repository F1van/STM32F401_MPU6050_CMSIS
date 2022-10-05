#include "PID.h"

float PID_Controller(PIDControll *pid, float angle)
{
    float k0 = pid->Kp + pid->t0 / pid->ti + pid->td / pid->t0;
    float k1 = pid->Kp + 2 * pid->td / pid->t0;
    float k2 = pid->td / pid->t0;
    float error_k = pid->state - angle;
    pid->out += k0 * error_k - k1 * pid->error_k1 + k2 * pid->error_k2;
    pid->error_k2 = pid->error_k1;
    pid->error_k1 = error_k;
    return pid->out;
}

