/**
  ******************************************************************************
  * @file           : pid.h
  * @brief          : Generic, stateful PID + Feedforward controller for
  *                   Micromouse translational and rotational control loops.
  ******************************************************************************
  */

#ifndef INC_PID_H_
#define INC_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                              DATA STRUCTURES                               */
/* ========================================================================== */
typedef struct {
    /* Controller Gains */
    float Kp;                   /* Proportional gain                          */
    float Ki;                   /* Integral gain                              */
    float Kd;                   /* Derivative gain                            */
    float Kf;                   /* Feedforward gain (e.g., % duty per mm/s)   */

    /* Limits & Filtering */
    float out_min;              /* Minimum total output clamp (e.g., -100.0f) */
    float out_max;              /* Maximum total output clamp (e.g., +100.0f) */
    float integral_max;         /* Anti-windup clamp for integral term        */
    float d_filter_alpha;       /* Low-pass filter weight for D term (0..1]   */
                                /* 1.0 = no filtering, 0.2 = heavy filtering  */

    /* Internal Controller State */
    float integral;             /* Accumulated integral term (Ki * sum(e*dt)) */
    float prev_error;           /* Previous error (used if D-on-error)        */
    float prev_measurement;     /* Previous measurement (for D-on-measurement)*/
    float d_filtered;           /* Filtered derivative term                   */
    float last_output;          /* Last computed total output                 */
    bool  d_on_measurement;     /* true = Derivative on measurement (no kick) */
    bool  first_run;            /* Prevents derivative spike on first sample  */
} PID_Controller_t;

/* ========================================================================== */
/*                            FUNCTION PROTOTYPES                             */
/* ========================================================================== */
void  PID_Init(PID_Controller_t *pid,
               float Kp, float Ki, float Kd, float Kf,
               float out_min, float out_max,
               float integral_max);

void  PID_Reset(PID_Controller_t *pid);

void  PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd, float Kf);

void  PID_ConfigureDerivative(PID_Controller_t *pid, float alpha, bool d_on_measurement);

float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_H_ */

