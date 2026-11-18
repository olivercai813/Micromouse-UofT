/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : Micromouse DC Motor, Quadrature Encoder, and Current-Sense
  *                   driver implementation for STM32F746VETx.
  *
  * ============================================================================
  *                     COMPLETE GPIO & FUNCTION CALLOUT TABLE
  * ============================================================================
  * Pin   | Schematic Net | Peripheral / Mode     | Hardware Function
  * ------|---------------|-----------------------|-----------------------------
  * PD12  | MO1_IN1       | TIM4_CH1 (AF2 PWM)    | Motor 1 H-Bridge Input 1
  * PD13  | MO1_IN2       | TIM4_CH2 (AF2 PWM)    | Motor 1 H-Bridge Input 2
  * PD14  | MO2_IN1       | TIM4_CH3 (AF2 PWM)    | Motor 2 H-Bridge Input 1
  * PD15  | MO2_IN2       | TIM4_CH4 (AF2 PWM)    | Motor 2 H-Bridge Input 2
  * PE7   | NSLEEP_HB_1   | GPIO_Output (PushPull)| Motor 1 H-Bridge Sleep (High=Wake)
  * PE10  | NSLEEP_HB_2   | GPIO_Output (PushPull)| Motor 2 H-Bridge Sleep (High=Wake)
  * PE9   | Encoder_A_1   | TIM1_CH1 (AF1 Enc)    | Motor 1 Quadrature Encoder Ch A
  * PE11  | Encoder_B_1   | TIM1_CH2 (AF1 Enc)    | Motor 1 Quadrature Encoder Ch B
  * PA6   | Encoder_A_2   | TIM3_CH1 (AF2 Enc)    | Motor 2 Quadrature Encoder Ch A
  * PA7   | Encoder_B_2   | TIM3_CH2 (AF2 Enc)    | Motor 2 Quadrature Encoder Ch B
  * PA2   | VISEN_1       | ADC1_IN2 (Analog)     | Motor 1 Current Draw Sense
  * PA3   | VISEN_2       | ADC2_IN3 (Analog)     | Motor 2 Current Draw Sense
  * PB0   | TEST_LED      | GPIO_Output (PushPull)| Status / Heartbeat LED (D1)
  * ============================================================================
  */

#include "motor.h"
#include <math.h>

/* External peripheral handles defined in main.c */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* Private variables */
static Motor_State_t motor_state[MOTOR_COUNT] = {0};
static uint32_t      pwm_max_counts = 400; /* ARR + 1 (399 + 1 = 400) */

/**
  * @brief  Initializes PWM timers, Quadrature Encoder timers, and wakes H-bridges.
  */
void Motor_Init(void)
{
    pwm_max_counts = __HAL_TIM_GET_AUTORELOAD(MO1_PWM_TIM_HANDLE) + 1U;

    HAL_TIM_PWM_Start(MO1_PWM_TIM_HANDLE, MO1_IN1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(MO1_PWM_TIM_HANDLE, MO1_IN2_TIM_CHANNEL);
    HAL_TIM_PWM_Start(MO2_PWM_TIM_HANDLE, MO2_IN1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(MO2_PWM_TIM_HANDLE, MO2_IN2_TIM_CHANNEL);

    Motor_StopAll();

    HAL_TIM_Encoder_Start(MO1_ENC_TIM_HANDLE, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(MO2_ENC_TIM_HANDLE, TIM_CHANNEL_ALL);

    Motor_ResetOdometry();
    Motor_WakeAll();
}

void Motor_SetSleep(Motor_ID_t motor, bool sleep_enable)
{
    GPIO_PinState pin_state = sleep_enable ? GPIO_PIN_RESET : GPIO_PIN_SET;

    if (motor == MOTOR_ID_1) {
        HAL_GPIO_WritePin(MO1_NSLEEP_GPIO_PORT, MO1_NSLEEP_GPIO_PIN, pin_state);
    } else if (motor == MOTOR_ID_2) {
        HAL_GPIO_WritePin(MO2_NSLEEP_GPIO_PORT, MO2_NSLEEP_GPIO_PIN, pin_state);
    }
}

void Motor_WakeAll(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_state[i].stall_counter_ms = 0U;
        motor_state[i].is_stalled       = false;
    }
    Motor_SetSleep(MOTOR_ID_1, false);
    Motor_SetSleep(MOTOR_ID_2, false);
}

void Motor_SleepAll(void)
{
    Motor_StopAll();
    Motor_SetSleep(MOTOR_ID_1, true);
    Motor_SetSleep(MOTOR_ID_2, true);
}

/**
  * @brief  Sets motor speed and direction using Slow-Decay (Drive/Brake) PWM
  *         with standstill deadband and gearbox stiction compensation.
  * @param  motor: MOTOR_ID_1 or MOTOR_ID_2
  * @param  duty_percent: signed physical PWM percentage. Commands are clamped
  *         to +/-MOTOR_MAX_DUTY_PERCENT for the 6 V motors.
  */
void Motor_SetDuty(Motor_ID_t motor, float duty_percent)
{
    /* 1. Limit the motor terminal duty for the 7.4 V battery / 6 V motors. */
    if (duty_percent > MOTOR_MAX_DUTY_PERCENT) {
        duty_percent = MOTOR_MAX_DUTY_PERCENT;
    }
    if (duty_percent < -MOTOR_MAX_DUTY_PERCENT) {
        duty_percent = -MOTOR_MAX_DUTY_PERCENT;
    }

    /* 2. Apply direction inversion macro */
    if (motor == MOTOR_ID_1) {
        duty_percent *= (float)MOTOR_1_DIR_INVERT;
    } else {
        duty_percent *= (float)MOTOR_2_DIR_INVERT;
    }

    float abs_duty = fabsf(duty_percent);

    /* 3. Standstill Deadband & Stiction Compensation */
    if (abs_duty < MOTOR_DEADBAND_DUTY) {
        abs_duty = 0.0f;
    } else {
        /* Scale [DEADBAND .. MAX] into [MIN_SPIN_DUTY .. MAX]. */
        abs_duty = MOTOR_MIN_SPIN_DUTY +
                   ((abs_duty - MOTOR_DEADBAND_DUTY) /
                    (MOTOR_MAX_DUTY_PERCENT - MOTOR_DEADBAND_DUTY)) *
                   (MOTOR_MAX_DUTY_PERCENT - MOTOR_MIN_SPIN_DUTY);
    }

    uint32_t mag_counts = (uint32_t)((abs_duty / 100.0f) * (float)pwm_max_counts);
    uint32_t in1_ccr = pwm_max_counts;
    uint32_t in2_ccr = pwm_max_counts;

    if (abs_duty == 0.0f) {
        /* Zero command: Hold in Active Brake with zero differential voltage */
        in1_ccr = pwm_max_counts;
        in2_ccr = pwm_max_counts;
    } else if (duty_percent > 0.0f) {
        /* Forward (Slow Decay): IN1 = 100%, IN2 = (100% - duty) */
        in1_ccr = pwm_max_counts;
        in2_ccr = pwm_max_counts - mag_counts;
    } else {
        /* Reverse (Slow Decay): IN1 = (100% - duty), IN2 = 100% */
        in1_ccr = pwm_max_counts - mag_counts;
        in2_ccr = pwm_max_counts;
    }

    if (motor == MOTOR_ID_1) {
        __HAL_TIM_SET_COMPARE(MO1_PWM_TIM_HANDLE, MO1_IN1_TIM_CHANNEL, in1_ccr);
        __HAL_TIM_SET_COMPARE(MO1_PWM_TIM_HANDLE, MO1_IN2_TIM_CHANNEL, in2_ccr);
    } else if (motor == MOTOR_ID_2) {
        __HAL_TIM_SET_COMPARE(MO2_PWM_TIM_HANDLE, MO2_IN1_TIM_CHANNEL, in1_ccr);
        __HAL_TIM_SET_COMPARE(MO2_PWM_TIM_HANDLE, MO2_IN2_TIM_CHANNEL, in2_ccr);
    }
}

void Motor_Brake(Motor_ID_t motor)
{
    if (motor == MOTOR_ID_1) {
        __HAL_TIM_SET_COMPARE(MO1_PWM_TIM_HANDLE, MO1_IN1_TIM_CHANNEL, pwm_max_counts);
        __HAL_TIM_SET_COMPARE(MO1_PWM_TIM_HANDLE, MO1_IN2_TIM_CHANNEL, pwm_max_counts);
    } else if (motor == MOTOR_ID_2) {
        __HAL_TIM_SET_COMPARE(MO2_PWM_TIM_HANDLE, MO2_IN1_TIM_CHANNEL, pwm_max_counts);
        __HAL_TIM_SET_COMPARE(MO2_PWM_TIM_HANDLE, MO2_IN2_TIM_CHANNEL, pwm_max_counts);
    }
}

void Motor_Coast(Motor_ID_t motor)
{
    if (motor == MOTOR_ID_1) {
        __HAL_TIM_SET_COMPARE(MO1_PWM_TIM_HANDLE, MO1_IN1_TIM_CHANNEL, 0U);
        __HAL_TIM_SET_COMPARE(MO1_PWM_TIM_HANDLE, MO1_IN2_TIM_CHANNEL, 0U);
    } else if (motor == MOTOR_ID_2) {
        __HAL_TIM_SET_COMPARE(MO2_PWM_TIM_HANDLE, MO2_IN1_TIM_CHANNEL, 0U);
        __HAL_TIM_SET_COMPARE(MO2_PWM_TIM_HANDLE, MO2_IN2_TIM_CHANNEL, 0U);
    }
}

void Motor_StopAll(void)
{
    Motor_Brake(MOTOR_ID_1);
    Motor_Brake(MOTOR_ID_2);
}

/**
  * @brief  Samples both 16-bit hardware quadrature encoders and updates position
  *         (mm) and low-pass filtered velocity (mm/s). Call at 1 kHz.
  */
void Motor_UpdateOdometry(float dt_s)
{
    if (dt_s <= 0.0f) return;

    uint16_t raw_cnt[MOTOR_COUNT];
    raw_cnt[MOTOR_ID_1] = (uint16_t)__HAL_TIM_GET_COUNTER(MO1_ENC_TIM_HANDLE);
    raw_cnt[MOTOR_ID_2] = (uint16_t)__HAL_TIM_GET_COUNTER(MO2_ENC_TIM_HANDLE);

    const int32_t enc_invert[MOTOR_COUNT] = { ENCODER_1_DIR_INVERT, ENCODER_2_DIR_INVERT };

    for (int i = 0; i < MOTOR_COUNT; i++) {
        int16_t diff = (int16_t)(raw_cnt[i] - motor_state[i].prev_raw_cnt);
        motor_state[i].prev_raw_cnt = raw_cnt[i];

        int32_t signed_ticks = (int32_t)diff * enc_invert[i];
        motor_state[i].delta_ticks  = signed_ticks;
        motor_state[i].total_ticks += signed_ticks;

        float delta_mm = (float)signed_ticks * MOTOR_MM_PER_TICK;
        motor_state[i].position_mm += delta_mm;

        /* Tunable LPF (MOTOR_VEL_LPF_ALPHA = 0.30f) suppresses 12-CPR tick noise */
        float inst_vel_mms = delta_mm / dt_s;
        motor_state[i].velocity_mms = (MOTOR_VEL_LPF_ALPHA * inst_vel_mms) +
                                      ((1.0f - MOTOR_VEL_LPF_ALPHA) * motor_state[i].velocity_mms);
    }
}

void Motor_ResetOdometry(void)
{
    __HAL_TIM_SET_COUNTER(MO1_ENC_TIM_HANDLE, 0U);
    __HAL_TIM_SET_COUNTER(MO2_ENC_TIM_HANDLE, 0U);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_state[i].prev_raw_cnt = 0U;
        motor_state[i].delta_ticks  = 0;
        motor_state[i].total_ticks  = 0;
        motor_state[i].position_mm  = 0.0f;
        motor_state[i].velocity_mms = 0.0f;
    }
}

uint16_t Motor_ReadVisenADC(Motor_ID_t motor)
{
    ADC_HandleTypeDef *hadc = (motor == MOTOR_ID_1) ? MO1_VISEN_ADC_HANDLE : MO2_VISEN_ADC_HANDLE;
    uint16_t raw_val = 0U;

    if (HAL_ADC_Start(hadc) == HAL_OK) {
        if (HAL_ADC_PollForConversion(hadc, 1U) == HAL_OK) {
            raw_val = (uint16_t)HAL_ADC_GetValue(hadc);
        }
        HAL_ADC_Stop(hadc);
    }
    motor_state[motor].visen_adc_raw = raw_val;
    return raw_val;
}

/**
  * @brief  Checks both motors for sustained overcurrent/stall via VISEN_1 & VISEN_2.
  *         If either motor exceeds MOTOR_VISEN_STALL_THRESH while barely moving for
  *         MOTOR_STALL_TIME_LIMIT_MS, both H-bridges are put to sleep to prevent burnout.
  * @retval true if a stall fault occurred, false if normal.
  */
bool Motor_CheckStall(void)
{
    bool any_stalled = false;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint16_t current_adc = Motor_ReadVisenADC((Motor_ID_t)i);
        bool high_current    = (current_adc > MOTOR_VISEN_STALL_THRESH);
        bool low_speed       = (fabsf(motor_state[i].velocity_mms) < 15.0f);

        if (high_current && low_speed) {
            motor_state[i].stall_counter_ms++;
            if (motor_state[i].stall_counter_ms >= MOTOR_STALL_TIME_LIMIT_MS) {
                motor_state[i].is_stalled = true;
                any_stalled = true;
            }
        } else if (motor_state[i].stall_counter_ms > 0U) {
            motor_state[i].stall_counter_ms--;
        }
    }

    if (any_stalled) {
        Motor_SleepAll();
    }

    return any_stalled;
}

const Motor_State_t* Motor_GetState(Motor_ID_t motor)
{
    return &motor_state[motor];
}

float Motor_GetLinearVelocity_mms(void)
{
    return 0.5f * (motor_state[MOTOR_LEFT].velocity_mms + motor_state[MOTOR_RIGHT].velocity_mms);
}

float Motor_GetLinearDistance_mm(void)
{
    return 0.5f * (motor_state[MOTOR_LEFT].position_mm + motor_state[MOTOR_RIGHT].position_mm);
}

float Motor_GetAngularVelocity_degs(void)
{
    float diff_mms = motor_state[MOTOR_RIGHT].velocity_mms - motor_state[MOTOR_LEFT].velocity_mms;
    float omega_rads = diff_mms / MOTOR_TRACK_WIDTH_MM;
    return omega_rads * (180.0f / MOTOR_PI);
}

float Motor_GetHeading_deg(void)
{
    float diff_mm = motor_state[MOTOR_RIGHT].position_mm - motor_state[MOTOR_LEFT].position_mm;
    float theta_rad = diff_mm / MOTOR_TRACK_WIDTH_MM;
    return theta_rad * (180.0f / MOTOR_PI);
}

