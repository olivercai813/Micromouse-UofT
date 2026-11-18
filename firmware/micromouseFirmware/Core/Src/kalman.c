/**
  ******************************************************************************
  * @file           : kalman.c
  * @brief          : Implementation of 2-State Adaptive Linear Kalman Filter
  *                   for Micromouse Yaw Heading and Gyro Bias Estimation.
  ******************************************************************************
  */

#include "kalman.h"
#include <math.h>

void Kalman_Init(YawKalman_t *kf, float initial_bias_degs)
{
    if (!kf) return;

    kf->theta_deg           = 0.0f;
    kf->gyro_bias_degs      = initial_bias_degs;
    kf->omega_unbiased_degs = 0.0f;

    /* Initialize 2x2 Covariance Matrix P */
    kf->P[0][0] = 0.01f;  kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;   kf->P[1][1] = 0.10f;

    /* Default Tuning Parameters for 1 kHz Micromouse Loop */
    kf->Q_theta        = 0.001f;  /* High trust in gyro integration short-term */
    kf->Q_bias         = 0.0001f; /* Gyro bias drifts slowly with temperature  */
    kf->R_omega_base   = 25.0f;   /* Base variance of encoder turn rate        */
    kf->k_slip_penalty = 15.0f;   /* Inflates R when wheels slip or turn fast  */
}

void Kalman_ResetHeading(YawKalman_t *kf, float new_heading_deg)
{
    if (!kf) return;
    kf->theta_deg = new_heading_deg;
    kf->P[0][0]   = 0.01f;
    kf->P[0][1]   = 0.0f;
    kf->P[1][0]   = 0.0f;
}

void Kalman_Update(YawKalman_t *kf,
                   float gyro_omega_degs,
                   float enc_omega_degs,
                   bool  is_stationary,
                   float dt_s)
{
    if (!kf || dt_s <= 0.0f) return;

    /* ====================================================================== */
    /* 1. PREDICT STEP (Time Update using Gyroscope)                          */
    /*    State transition:                                                   */
    /*      theta_k = theta_{k-1} + (gyro_omega - bias_{k-1}) * dt            */
    /*      bias_k  = bias_{k-1}                                              */
    /* ====================================================================== */
    kf->omega_unbiased_degs = gyro_omega_degs - kf->gyro_bias_degs;
    kf->theta_deg          += kf->omega_unbiased_degs * dt_s;

    /* Update Error Covariance: P = F * P * F^T + Q
     * where F = [ 1, -dt ]
     *           [ 0,   1 ]
     */
    float p00 = kf->P[0][0];
    float p01 = kf->P[0][1];
    float p10 = kf->P[1][0];
    float p11 = kf->P[1][1];

    kf->P[0][0] = p00 - dt_s * (p10 + p01) + (dt_s * dt_s * p11) + (kf->Q_theta * dt_s);
    kf->P[0][1] = p01 - dt_s * p11;
    kf->P[1][0] = p10 - dt_s * p11;
    kf->P[1][1] = p11 + (kf->Q_bias * dt_s);

    /* ====================================================================== */
    /* 2. ADAPTIVE MEASUREMENT UPDATE (Bias Calibration & Slip Rejection)     */
    /*    Observation model:                                                  */
    /*      enc_omega = gyro_omega - bias  =>  bias_meas = gyro_omega - enc   */
    /*      H = [ 0, 1 ]                                                      */
    /* ====================================================================== */
    float measured_bias = gyro_omega_degs - enc_omega_degs;
    float R_effective   = kf->R_omega_base;

    if (is_stationary) {
        /* Zero-Velocity Update (ZUPT): Robot is parked, true omega is 0.0 deg/s.
         * Trust this measurement strongly to lock in the exact gyro bias! */
        measured_bias = gyro_omega_degs;
        R_effective   = 0.5f;
    } else {
        /* Dynamic Slip Rejection: When turning rapidly or when gyro and encoders
         * disagree (wheel slip), inflate R_effective so encoders don't corrupt
         * the gyro bias or heading estimate. */
        float slip_diff = fabsf(kf->omega_unbiased_degs - enc_omega_degs);
        float turn_mag  = fabsf(kf->omega_unbiased_degs);
        R_effective += kf->k_slip_penalty * (slip_diff + 0.5f * turn_mag);

        /* Also gently nudge theta_deg toward encoder delta when moving straight */
        if (turn_mag < 15.0f && slip_diff < 20.0f) {
            float omega_err = enc_omega_degs - kf->omega_unbiased_degs;
            kf->theta_deg  += (0.02f * omega_err * dt_s);
        }
    }

    /* Innovation (residual) on bias: y = z - H * x */
    float y_bias = measured_bias - kf->gyro_bias_degs;

    /* Innovation covariance: S = H * P * H^T + R = P[1][1] + R_effective */
    float S = kf->P[1][1] + R_effective;
    float inv_S = 1.0f / S;

    /* Kalman Gain: K = P * H^T * S^-1 = [ P[0][1] / S, P[1][1] / S ]^T */
    float K0 = kf->P[0][1] * inv_S;
    float K1 = kf->P[1][1] * inv_S;

    /* State Update: x = x + K * y */
    kf->theta_deg      += K0 * y_bias;
    kf->gyro_bias_degs += K1 * y_bias;

    /* Recompute unbiased angular velocity after bias update */
    kf->omega_unbiased_degs = gyro_omega_degs - kf->gyro_bias_degs;

    /* Covariance Update: P = (I - K * H) * P */
    p00 = kf->P[0][0];
    p01 = kf->P[0][1];
    p10 = kf->P[1][0];
    p11 = kf->P[1][1];

    kf->P[0][0] = p00 - K0 * p10;
    kf->P[0][1] = p01 - K0 * p11;
    kf->P[1][0] = p10 * (1.0f - K1);
    kf->P[1][1] = p11 * (1.0f - K1);
}

void Kalman_CorrectWithWallError(YawKalman_t *kf, float wall_angle_error_deg, float R_wall)
{
    if (!kf || R_wall <= 0.0f) return;

    /* Direct observation of heading error: H = [ 1, 0 ] */
    float S  = kf->P[0][0] + R_wall;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    kf->theta_deg      += K0 * wall_angle_error_deg;
    kf->gyro_bias_degs += K1 * wall_angle_error_deg;

    float p00 = kf->P[0][0];
    float p01 = kf->P[0][1];
    kf->P[0][0] -= K0 * p00;
    kf->P[0][1] -= K0 * p01;
    kf->P[1][0] -= K1 * p00;
    kf->P[1][1] -= K1 * p01;
}

float Kalman_GetHeading_deg(const YawKalman_t *kf)
{
    return kf ? kf->theta_deg : 0.0f;
}

float Kalman_GetAngularVelocity_degs(const YawKalman_t *kf)
{
    return kf ? kf->omega_unbiased_degs : 0.0f;
}

float Kalman_GetGyroBias_degs(const YawKalman_t *kf)
{
    return kf ? kf->gyro_bias_degs : 0.0f;
}

