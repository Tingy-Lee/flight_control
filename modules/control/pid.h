#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float last_error;
    float integrator_min;
    float integrator_max;
    float output_min;
    float output_max;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float out_min, float out_max);
void pid_reset(pid_t *pid);
float pid_update(pid_t *pid, float error, float dt_s);

#endif
