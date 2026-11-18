/**
  ******************************************************************************
  * @file           : kalman.h
  * @brief          : 2-State Adaptive Linear Kalman Filter for Micromouse
  *                   Yaw Heading and Gyro Bias Estimation with Slip Rejection.
  ******************************************************************************
  */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                              DATA STRUCTURES                               */
/* ========================================================================== */
typedef struct {
    /* State Vector x = [ theta_deg, gyro_bias_degs ]^T */
    float theta_deg;            /* Fused yaw heading angle (degrees, CCW+)    */
    float gyro_bias_degs;       /* Estimated gyroscope Z-axis bias (deg/s)    */
    float omega_unbiased_degs;  /* Bias-corrected turn rate (deg/s, CCW+)     */

    /* 2x2 Error Covariance Matrix P */
    float P[2][2];

    /* Process & Measurement Noise Tuning Parameters */
    float Q_theta;              /* Process noise variance for angle           */
    float Q_bias;               /* Process noise variance for gyro bias drift */
    float R_omega_base;         /* Base measurement noise for encoder omega   */
    float k_slip_penalty;       /* Adaptive penalty factor for wheel slip     */
} YawKalman_t;

/* ========================================================================== */
/*                            FUNCTION PROTOTYPES                             */
/* ========================================================================== */
void  Kalman_Init(YawKalman_t *kf, float initial_bias_degs);
void  Kalman_ResetHeading(YawKalman_t *kf, float new_heading_deg);

/**
  * @brief  Executes 1 kHz Predict + Adaptive Update step.
  * @param  kf: Pointer to filter instance
  * @param  gyro_omega_degs: Raw Z-axis angular rate from IMU (deg/s, CCW+)
  * @param  enc_omega_degs: Differential wheel angular rate from encoders (deg/s)
  * @param  is_stationary: true if both wheels are stopped (enables ZUPT)
  * @param  dt_s: Control loop period in seconds (0.001f for 1 kHz)
  */
void  Kalman_Update(YawKalman_t *kf,
                    float gyro_omega_degs,
                    float enc_omega_degs,
                    bool  is_stationary,
                    float dt_s);

/**
  * @brief  Optional absolute heading correction using maze corridor walls.
  */
void  Kalman_CorrectWithWallError(YawKalman_t *kf, float wall_angle_error_deg, float R_wall);

float Kalman_GetHeading_deg(const YawKalman_t *kf);
float Kalman_GetAngularVelocity_degs(const YawKalman_t *kf);
float Kalman_GetGyroBias_degs(const YawKalman_t *kf);

#ifdef __cplusplus
}
#endif

#endif /* INC_KALMAN_H_ */

