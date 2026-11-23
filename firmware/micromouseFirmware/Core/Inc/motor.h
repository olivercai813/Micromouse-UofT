/**
  ******************************************************************************
  * @file           : motor.h
  * @brief          : Header for Micromouse DC Motor, Quadrature Encoder, and
  *                   Current-Sense (VISEN) driver on STM32F746VETx.
  ******************************************************************************
  */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                    CONFIGURABLE MECHANICAL PARAMETERS                      */
/* ========================================================================== */
#define MOTOR_CONTROL_LOOP_HZ     1000.0f  /* 1 kHz (1 ms) control loop rate  */
#define MOTOR_CONTROL_DT_S        (1.0f / MOTOR_CONTROL_LOOP_HZ)

#define MOTOR_ENCODER_CPR         12.0f    /* 12 counts per motor shaft rev   */
#define MOTOR_GEAR_RATIO          30.0f    /* Configurable: Gear ratio        */
#define MOTOR_WHEEL_DIAMETER_MM   32.0f    /* Configurable: Wheel diameter mm */
#define MOTOR_TRACK_WIDTH_MM      75.0f    /* Configurable: Wheel spacing mm  */

#define MOTOR_PI                  3.14159265359f
#define MOTOR_WHEEL_CIRCUM_MM     (MOTOR_PI * MOTOR_WHEEL_DIAMETER_MM)
#define MOTOR_TICKS_PER_WHEEL_REV (MOTOR_ENCODER_CPR * MOTOR_GEAR_RATIO)
#define MOTOR_MM_PER_TICK         (MOTOR_WHEEL_CIRCUM_MM / MOTOR_TICKS_PER_WHEEL_REV)

/* ========================================================================== */
/*                EFFICIENCY, FILTERING & STALL PROTECTION MACROS             */
/* ========================================================================== */
#define MOTOR_VEL_LPF_ALPHA       0.30f    /* LPF weight for 12-CPR encoders  */
#define MOTOR_DEADBAND_DUTY       0.50f    /* Commands < 0.5% cut to 0% PWM   */
#define MOTOR_MIN_SPIN_DUTY       5.00f    /* Minimum % duty to break stiction*/
#define MOTOR_MAX_DUTY_PERCENT   71.00f    /* <= 6 V avg at 8.4 V full charge  */

#define MOTOR_VISEN_STALL_THRESH  3000U    /* 12-bit ADC threshold (0..4095)  */
#define MOTOR_STALL_TIME_LIMIT_MS 50U      /* Trip fault after 50 ms of stall */

/* Invert flags (-1 or 1) to easily match physical Left/Right wiring polarity */
#define MOTOR_1_DIR_INVERT        1
#define MOTOR_2_DIR_INVERT       -1
#define ENCODER_1_DIR_INVERT     -1    /* left counted backwards in the bench test */
#define ENCODER_2_DIR_INVERT      1

/* Map logical Left/Right wheels to Motor 1 and Motor 2 */
#define MOTOR_LEFT                MOTOR_ID_1
#define MOTOR_RIGHT               MOTOR_ID_2

/* ========================================================================== */
/*                 EXPLICIT GPIO & PERIPHERAL PINOUT DEFINITIONS              */
/* ========================================================================== */
/* --- Motor 1 (U3, J8) --- */
#define MO1_PWM_TIM_HANDLE        (&htim4)
#define MO1_IN1_GPIO_PORT         GPIOD
#define MO1_IN1_GPIO_PIN          GPIO_PIN_12       /* PD12 -> TIM4_CH1 */
#define MO1_IN1_TIM_CHANNEL       TIM_CHANNEL_1

#define MO1_IN2_GPIO_PORT         GPIOD
#define MO1_IN2_GPIO_PIN          GPIO_PIN_13       /* PD13 -> TIM4_CH2 */
#define MO1_IN2_TIM_CHANNEL       TIM_CHANNEL_2

#define MO1_NSLEEP_GPIO_PORT      GPIOE
#define MO1_NSLEEP_GPIO_PIN       GPIO_PIN_7        /* PE7  -> NSLEEP_HB_1 */

#define MO1_ENC_TIM_HANDLE        (&htim1)
#define MO1_ENC_A_GPIO_PORT       GPIOE
#define MO1_ENC_A_GPIO_PIN        GPIO_PIN_9        /* PE9  -> TIM1_CH1 (Encoder_A_1) */
#define MO1_ENC_B_GPIO_PORT       GPIOE
#define MO1_ENC_B_GPIO_PIN        GPIO_PIN_11       /* PE11 -> TIM1_CH2 (Encoder_B_1) */

#define MO1_VISEN_ADC_HANDLE      (&hadc1)
#define MO1_VISEN_GPIO_PORT       GPIOA
#define MO1_VISEN_GPIO_PIN        GPIO_PIN_2        /* PA2  -> ADC1_IN2 (VISEN_1) */

/* --- Motor 2 (U2, J7) --- */
#define MO2_PWM_TIM_HANDLE        (&htim4)
#define MO2_IN1_GPIO_PORT         GPIOD
#define MO2_IN1_GPIO_PIN          GPIO_PIN_14       /* PD14 -> TIM4_CH3 */
#define MO2_IN1_TIM_CHANNEL       TIM_CHANNEL_3

#define MO2_IN2_GPIO_PORT         GPIOD
#define MO2_IN2_GPIO_PIN          GPIO_PIN_15       /* PD15 -> TIM4_CH4 */
#define MO2_IN2_TIM_CHANNEL       TIM_CHANNEL_4

#define MO2_NSLEEP_GPIO_PORT      GPIOE
#define MO2_NSLEEP_GPIO_PIN       GPIO_PIN_10       /* PE10 -> NSLEEP_HB_2 */

#define MO2_ENC_TIM_HANDLE        (&htim3)
#define MO2_ENC_A_GPIO_PORT       GPIOA
#define MO2_ENC_A_GPIO_PIN        GPIO_PIN_6        /* PA6  -> TIM3_CH1 (Encoder_A_2) */
#define MO2_ENC_B_GPIO_PORT       GPIOA
#define MO2_ENC_B_GPIO_PIN        GPIO_PIN_7        /* PA7  -> TIM3_CH2 (Encoder_B_2) */

#define MO2_VISEN_ADC_HANDLE      (&hadc2)
#define MO2_VISEN_GPIO_PORT       GPIOA
#define MO2_VISEN_GPIO_PIN        GPIO_PIN_3        /* PA3  -> ADC2_IN3 (VISEN_2) */

/* ========================================================================== */
/*                              DATA STRUCTURES                               */
/* ========================================================================== */
typedef enum {
    MOTOR_ID_1 = 0,
    MOTOR_ID_2 = 1,
    MOTOR_COUNT = 2
} Motor_ID_t;

typedef struct {
    uint16_t prev_raw_cnt;       /* Last 16-bit timer counter reading         */
    int32_t  delta_ticks;        /* Ticks elapsed during last control period  */
    int64_t  total_ticks;        /* Cumulative ticks since boot/reset         */
    float    position_mm;        /* Cumulative linear distance traveled (mm)  */
    float    velocity_mms;       /* Filtered linear wheel speed (mm/s)        */
    uint16_t visen_adc_raw;      /* Raw 12-bit ADC current sense reading      */
    uint16_t stall_counter_ms;   /* Consecutive ms of overcurrent/stall       */
    bool     is_stalled;         /* Latched true if stall protection tripped  */
} Motor_State_t;

/* ========================================================================== */
/*                            FUNCTION PROTOTYPES                             */
/* ========================================================================== */
/* Lifecycle & Power Management */
void     Motor_Init(void);
void     Motor_SetSleep(Motor_ID_t motor, bool sleep_enable);
void     Motor_WakeAll(void);
void     Motor_SleepAll(void);

/* Low-Level Actuation (Slow-Decay with Deadband & Stiction Compensation) */
void     Motor_SetDuty(Motor_ID_t motor, float duty_percent);
void     Motor_Brake(Motor_ID_t motor);
void     Motor_Coast(Motor_ID_t motor);
void     Motor_StopAll(void);

/* Odometry, Current Sensing & Stall Protection */
void     Motor_UpdateOdometry(float dt_s);
void     Motor_ResetOdometry(void);
uint16_t Motor_ReadVisenADC(Motor_ID_t motor);
bool     Motor_CheckStall(void);

/* State Getters for Micromouse Control & Kalman Filter */
const Motor_State_t* Motor_GetState(Motor_ID_t motor);
float    Motor_GetLinearVelocity_mms(void);
float    Motor_GetLinearDistance_mm(void);
float    Motor_GetAngularVelocity_degs(void);
float    Motor_GetHeading_deg(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_H_ */

