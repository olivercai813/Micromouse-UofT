/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : Implementation of PID + Feedforward controller with
  *                   anti-windup, derivative-on-measurement, and LPF.
  ******************************************************************************
  */

#include "pid.h"

/* Helper inline function to clamp floats */
static inline float clamp_f(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

/**
  * @brief  Initializes a PID controller instance with safe defaults.
  */
void PID_Init(PID_Controller_t *pid,
              float Kp, float Ki, float Kd, float Kf,
              float out_min, float out_max,
              float integral_max)
{
    if (!pid) return;

    pid->Kp               = Kp;
    pid->Ki               = Ki;
    pid->Kd               = Kd;
    pid->Kf               = Kf;

    pid->out_min          = out_min;
    pid->out_max          = out_max;
    pid->integral_max     = (integral_max >= 0.0f) ? integral_max : -integral_max;

    /* Default derivative settings: moderate LPF (0.3) and D-on-measurement */
    pid->d_filter_alpha   = 0.3f;
    pid->d_on_measurement = true;

    PID_Reset(pid);
}

/**
  * @brief  Resets the internal state of the PID controller.
  */
void PID_Reset(PID_Controller_t *pid)
{
    if (!pid) return;

    pid->integral         = 0.0f;
    pid->prev_error       = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered       = 0.0f;
    pid->last_output      = 0.0f;
    pid->first_run        = true;
}

/**
  * @brief  Updates PID gains dynamically.
  */
void PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd, float Kf)
{
    if (!pid) return;

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;
}

/**
  * @brief  Sets derivative low-pass filter weight and derivative mode.
  * @param  alpha: 0.05f (heavy smoothing) to 1.0f (no smoothing)
  * @param  d_on_measurement: true = -Kd * d(measurement)/dt (avoids setpoint kick)
  */
void PID_ConfigureDerivative(PID_Controller_t *pid, float alpha, bool d_on_measurement)
{
    if (!pid) return;

    pid->d_filter_alpha   = clamp_f(alpha, 0.01f, 1.0f);
    pid->d_on_measurement = d_on_measurement;
}

/**
  * @brief  Computes the PID + Feedforward control output for one time step.
  */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement, float dt_s)
{
    if (!pid || dt_s <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    /* 1. Proportional Term */
    float p_term = pid->Kp * error;

    /* 2. Integral Term with Anti-Windup Clamping */
    pid->integral += (pid->Ki * error * dt_s);
    pid->integral  = clamp_f(pid->integral, -pid->integral_max, pid->integral_max);
    float i_term   = pid->integral;

    /* 3. Derivative Term (with First-Run guard and Low-Pass Filter) */
    float d_raw = 0.0f;
    if (pid->first_run) {
        pid->first_run        = false;
        pid->prev_error       = error;
        pid->prev_measurement = measurement;
        pid->d_filtered       = 0.0f;
    } else {
        if (pid->d_on_measurement) {
            /* Derivative on measurement eliminates spikes when setpoint changes */
            d_raw = -pid->Kd * ((measurement - pid->prev_measurement) / dt_s);
        } else {
            /* Standard derivative on error */
            d_raw = pid->Kd * ((error - pid->prev_error) / dt_s);
        }

        /* Exponential moving average low-pass filter on derivative */
        pid->d_filtered = (pid->d_filter_alpha * d_raw) +
                          ((1.0f - pid->d_filter_alpha) * pid->d_filtered);
    }

    pid->prev_error       = error;
    pid->prev_measurement = measurement;
    float d_term          = pid->d_filtered;

    /* 4. Velocity/Setpoint Feedforward Term */
    float ff_term = pid->Kf * setpoint;

    /* 5. Sum and Clamp Final Output */
    float output = p_term + i_term + d_term + ff_term;
    output       = clamp_f(output, pid->out_min, pid->out_max);

    pid->last_output = output;
    return output;
}

