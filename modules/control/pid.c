#include "modules/control/pid.h"

static float constrainf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

void pid_init(pid_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->integrator_min = out_min;
    pid->integrator_max = out_max;
    pid->output_min = out_min;
    pid->output_max = out_max;
}

void pid_reset(pid_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
}

float pid_update(pid_t *pid, float error, float dt_s)
{
    if ((pid == 0) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    pid->integrator += error * pid->ki * dt_s;
    pid->integrator = constrainf_local(pid->integrator, pid->integrator_min, pid->integrator_max);

    const float derivative = (error - pid->last_error) / dt_s;
    pid->last_error = error;

    return constrainf_local((pid->kp * error) + pid->integrator + (pid->kd * derivative),
                            pid->output_min,
                            pid->output_max);
}
