#include "pid.h"

void PIDController_Init(PID_Controller *pid)
{
    /* Initialize controller variables */
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;

    pid->differentiator = 0.0f;
    pid->prev_measurement = 0.0f;

    pid->out = 0.0f;
}

float PIDController_Update(PID_Controller *pid, float setpoint, float measurement)
{
    /* Error signal */
    float error = setpoint - measurement;

    /* Proportional */
    float proportional = pid->Kp * error;

    /* Integral */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->sampling_time * (error + pid->prev_error);

    /* Anti-windup via dynamic integrator clamping */
    float min, max;

    /* Compute integrator limits */
    if (pid->lim_max > proportional)
    {
        max = pid->lim_max - proportional;
    }
    else
    {
        max = 0.0f;
    }
    if (pid->lim_min < proportional)
    {
        min = pid->lim_min - proportional;
    }
    else
    {
        min = 0.0f;
    }

    /* Clamp integrator */
    if (pid->integrator > max)
    {
        pid->integrator = max;
    }
    else if (pid->integrator < min)
    {
        pid->integrator = min;
    }

    /* Derivative (band-limited differentiator/cascaded with a low-pass filter) */
    /* Note: derivative on measurement, therefore minus sign in front of equation! */
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prev_measurement) + (2.0f * pid->tau - pid->sampling_time) * pid->differentiator) / (2.0f * pid->tau + pid->sampling_time);

    /* Compute output and apply limits */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->lim_max)
    {

        pid->out = pid->lim_max;
    }
    else if (pid->out < pid->lim_min)
    {

        pid->out = pid->lim_min;
    }

    /* Store error and measurement for later use */
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    /* Return controller output */
    return pid->out;
}