#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float lim_min;
    float lim_max;

    /* Sample time (in seconds) */
    float sampling_time;

    /* Controller "memory" */
    float integrator;
    float prev_error;
    float differentiator;
    float prev_measurement;

    /* Controller ouput */
    float out;
} PID_Controller;

void PIDController_Init(PID_Controller *pid);
float PIDController_Update(PID_Controller *pid, float setpoint, float measurement);

#ifdef __cplusplus
} // extern "C"
#endif

#endif