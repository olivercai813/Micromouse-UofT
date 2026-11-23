/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ir.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Motor pins, PWM, encoders and their directions now live in motor.h. */
#define MOTOR_MAX_DUTY_INT          71U   /* same limit as MOTOR_MAX_DUTY_PERCENT */
#define MOTOR_MAX_DUTY_TENTHS      710U
#define MOTOR_MAX_RUN_MS         10000U
#define TELEMETRY_DEFAULT_MS       250U
#define SENSOR_MISSING          (-32768)

#define FLAG_IMU           (1U << 0)
#define FLAG_MAG           (1U << 1)
#define FLAG_IR            (1U << 2)
#define FLAG_TOF_1         (1U << 3)
#define FLAG_TOF_2         (1U << 4)
#define FLAG_TOF_3         (1U << 5)
#define FLAG_BT_CONNECTED  (1U << 6)
#define FLAG_MOTORS_ACTIVE (1U << 7)

#define LIS2MDL_ADDR       0x1EU
#define VL53_DEFAULT_ADDR  0x29U

/* ---- Control loop (motor + imu + kalman + pid) ---------------------------- */
#define CONTROL_LOOP_HZ          1000U    /* 1 ms control step                 */
#define CONTROL_DT_MAX_S         0.05f    /* cap dt after a slow sensor read   */

/* Stall protection reads the motor current (VISEN) on ADC1/ADC2. Keep it OFF
 * until the VREF+ bridge (U1 pin 20 -> 21) is soldered: before that the ADC
 * readings are garbage and could stop the motors for no reason. */
#define MOTOR_STALL_PROTECTION   0

/* Starting gains: GUESSES. Tune them over Bluetooth with the GAINS command. */
#define FWD_KP        0.020f    /* % duty per (mm/s) of speed error          */
#define FWD_KI        0.200f    /* % duty per (mm/s * s)                      */
#define FWD_KD        0.000f
#define FWD_KF        0.070f    /* % duty per mm/s asked for (feedforward)   */
#define FWD_I_MAX     30.0f     /* anti-windup limit, % duty                  */
#define ROT_KP        0.800f    /* % duty difference per degree of error     */
#define ROT_KI        0.000f
#define ROT_KD        0.030f    /* % duty per deg/s (damping)                */
#define ROT_FF        0.050f    /* % duty per deg/s asked for (feedforward)  */
#define ROT_OUT_MAX   40.0f     /* max % duty difference between wheels      */
#define ROT_I_MAX     20.0f

#define TURN_RATE_DEGS        180.0f  /* how fast TURN rotates the target     */
#define TURN_DONE_DEG         2.0f    /* "arrived" when within this many deg  */
#define TURN_DONE_RATE_DEGS   10.0f   /* ...and turning slower than this      */
#define TURN_SETTLE_MS        50U     /* ...for this long                     */
#define TURN_DEFAULT_MS       3000U   /* safety timeout for TURN              */
#define SPEED_LIMIT_MMS       1000
#define RATE_LIMIT_DEGS       720
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static I2C_HandleTypeDef hi2c1;
static ADC_HandleTypeDef hadc3;
static int32_t imu_values[6] = {0};
static int32_t mag_values[3] = {0};
static uint16_t ir_values[4] = {0};
static uint16_t tof_values[3] = {0};
static uint8_t sensor_flags = 0U;
static volatile int16_t commanded_left_tenths = 0;
static volatile int16_t commanded_right_tenths = 0;
static volatile uint32_t motor_stop_tick = 0U;
static volatile uint32_t emergency_stop_generation = 0U;
static volatile uint8_t motor_timeout_event_pending = 0U;
static volatile uint8_t emergency_stop_ack_pending = 0U;
static uint32_t telemetry_period_ms = TELEMETRY_DEFAULT_MS;
static uint8_t uart_rx_byte = 0U;
static volatile uint8_t uart_rx_head = 0U;
static volatile uint8_t uart_rx_tail = 0U;
static uint8_t uart_rx_ring[128] = {0};
static uint8_t uart_stop_candidate = 1U;
static uint8_t uart_stop_index = 0U;
static volatile uint32_t uart_stop_flush_generation = 0U;
static uint8_t uart_tx_byte = 0U;
static volatile uint16_t uart_tx_head = 0U;
static volatile uint16_t uart_tx_tail = 0U;
static volatile uint8_t uart_tx_busy = 0U;
static uint8_t uart_tx_ring[512] = {0};

/* Motion control state */
enum { CTRL_OFF = 0U, CTRL_OPEN_LOOP, CTRL_VELOCITY, CTRL_TURN };
static volatile uint8_t control_mode = CTRL_OFF;
static volatile uint8_t motors_enabled = 0U;
static volatile uint8_t turn_done_event_pending = 0U;
#if MOTOR_STALL_PROTECTION
static volatile uint8_t stall_event_pending = 0U;
#endif
static PID_Controller_t pid_forward;
static PID_Controller_t pid_rotation;
static YawKalman_t yaw_kf;
static float rot_feedforward = ROT_FF;
static float target_speed_mms = 0.0f;
static float target_rate_degs = 0.0f;
static float target_heading_deg = 0.0f;
static float turn_final_deg = 0.0f;
static uint32_t turn_settle_ms = 0U;
static uint32_t control_last_cycles = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Motors_Stop(void);
static uint8_t Motors_SetSignedTenths(int32_t left_tenths, int32_t right_tenths,
                                      uint32_t command_generation);
static void Control_Init(void);
static void Control_Tick(void);
static void SensorHardware_Init(void);
static void Sensors_Update(void);
static void Protocol_HandleLine(char *line, uint32_t command_generation);
static void Protocol_SendTelemetry(void);
static uint8_t UART_PopByte(uint8_t *value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------------------------------------------------------------------------
 * Motor power on/off. Motors_Stop() is also called from interrupts (SysTick
 * timeout and the UART "STOP" detector), so it only touches registers/pins.
 * ------------------------------------------------------------------------- */
static void Motors_Stop(void)
{
  Motor_SleepAll();                       /* brake both, then nSLEEP low */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  control_mode = CTRL_OFF;
  motors_enabled = 0U;
  commanded_left_tenths = 0;
  commanded_right_tenths = 0;
  motor_stop_tick = 0U;
}

static int32_t ClampMotorPercent(int32_t percent)
{
  if (percent > (int32_t)MOTOR_MAX_DUTY_INT) return MOTOR_MAX_DUTY_INT;
  if (percent < -(int32_t)MOTOR_MAX_DUTY_INT) return -(int32_t)MOTOR_MAX_DUTY_INT;
  return percent;
}

static int32_t ClampMotorTenths(int32_t percent_tenths)
{
  if (percent_tenths > (int32_t)MOTOR_MAX_DUTY_TENTHS) return MOTOR_MAX_DUTY_TENTHS;
  if (percent_tenths < -(int32_t)MOTOR_MAX_DUTY_TENTHS) return -(int32_t)MOTOR_MAX_DUTY_TENTHS;
  return percent_tenths;
}

static float ClampFloat(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

/* Wake both H-bridges and switch to <mode>, unless a STOP arrived after the
 * command was received (stale generation). Returns 1 if the motors are live.
 * On success interrupts are left DISABLED so the caller can set the first
 * duty atomically; the caller must call Motors_EndStart() afterwards. */
static uint8_t Motors_BeginStart(uint32_t command_generation, uint8_t mode,
                                 uint32_t *interrupt_state)
{
  if (emergency_stop_generation != command_generation) return 0U;

  Motors_Stop();
  Motor_WakeAll();
  HAL_Delay(2);                           /* MP6550 wake-up time */

  *interrupt_state = __get_PRIMASK();
  __disable_irq();
  if (emergency_stop_generation != command_generation)
  {
    if (*interrupt_state == 0U) __enable_irq();
    Motors_Stop();
    return 0U;
  }
  control_mode = mode;
  motors_enabled = 1U;
  return 1U;
}

static void Motors_EndStart(uint32_t interrupt_state)
{
  if (interrupt_state == 0U) __enable_irq();
}

/* Open-loop drive (the old "M" / "M10" commands): fixed duty per wheel. */
static uint8_t Motors_SetSignedTenths(int32_t left_tenths, int32_t right_tenths,
                                      uint32_t command_generation)
{
  uint32_t interrupt_state = 0U;
  left_tenths = ClampMotorTenths(left_tenths);
  right_tenths = ClampMotorTenths(right_tenths);

  if (left_tenths == 0 && right_tenths == 0)
  {
    if (emergency_stop_generation != command_generation) return 0U;
    Motors_Stop();
    return 1U;
  }

  if (Motors_BeginStart(command_generation, CTRL_OPEN_LOOP, &interrupt_state) == 0U) return 0U;
  /* motor.c applies the mirror-image inversion of motor 2 itself. */
  Motor_SetDuty(MOTOR_LEFT, (float)left_tenths / 10.0f);
  Motor_SetDuty(MOTOR_RIGHT, (float)right_tenths / 10.0f);
  commanded_left_tenths = (int16_t)left_tenths;
  commanded_right_tenths = (int16_t)right_tenths;
  Motors_EndStart(interrupt_state);
  return 1U;
}

/* Closed-loop drive: forward speed (mm/s) + turn rate (deg/s, + = left/CCW). */
static uint8_t Motion_StartVelocity(float speed_mms, float rate_degs,
                                    uint32_t command_generation)
{
  uint32_t interrupt_state = 0U;
  PID_Reset(&pid_forward);
  PID_Reset(&pid_rotation);
  target_speed_mms = speed_mms;
  target_rate_degs = rate_degs;
  target_heading_deg = Kalman_GetHeading_deg(&yaw_kf);
  if (Motors_BeginStart(command_generation, CTRL_VELOCITY, &interrupt_state) == 0U) return 0U;
  Motors_EndStart(interrupt_state);
  return 1U;
}

/* Turn on the spot by <angle_deg> (+ = left/CCW) using the fused heading. */
static uint8_t Motion_StartTurn(float angle_deg, uint32_t command_generation)
{
  uint32_t interrupt_state = 0U;
  PID_Reset(&pid_forward);
  PID_Reset(&pid_rotation);
  target_speed_mms = 0.0f;
  target_rate_degs = 0.0f;
  target_heading_deg = Kalman_GetHeading_deg(&yaw_kf);
  turn_final_deg = target_heading_deg + angle_deg;
  turn_settle_ms = 0U;
  if (Motors_BeginStart(command_generation, CTRL_TURN, &interrupt_state) == 0U) return 0U;
  Motors_EndStart(interrupt_state);
  return 1U;
}

/* ---------------------------------------------------------------------------
 * 1 kHz control step: odometry -> IMU -> Kalman heading -> PID -> motors.
 * Called on every pass of the main loop; it runs only when >= 1 ms passed.
 * ------------------------------------------------------------------------- */
static void Control_Init(void)
{
  /* Cycle counter for an exact dt (ir.c also enables it; harmless twice). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  control_last_cycles = DWT->CYCCNT;

  /* Forward speed loop: output = % duty (both wheels). Kf = % duty per mm/s. */
  PID_Init(&pid_forward, FWD_KP, FWD_KI, FWD_KD, FWD_KF,
           -MOTOR_MAX_DUTY_PERCENT, MOTOR_MAX_DUTY_PERCENT, FWD_I_MAX);
  /* Heading loop: output = % duty difference between the wheels. */
  PID_Init(&pid_rotation, ROT_KP, ROT_KI, ROT_KD, 0.0f,
           -ROT_OUT_MAX, ROT_OUT_MAX, ROT_I_MAX);
}

static void Control_Tick(void)
{
  uint32_t now = DWT->CYCCNT;
  uint32_t elapsed = now - control_last_cycles;
  uint32_t period = SystemCoreClock / CONTROL_LOOP_HZ;
  if (elapsed < period) return;
  control_last_cycles = now;

  float dt = (float)elapsed / (float)SystemCoreClock;
  if (dt > CONTROL_DT_MAX_S) dt = CONTROL_DT_MAX_S;  /* after a slow sensor read */

  /* 1. Sensors */
  Motor_UpdateOdometry(dt);
  IMU_Update(dt);
  float enc_rate = Motor_GetAngularVelocity_degs();
  float gyro_rate = IMU_IsPresent() ? IMU_GetData()->gyro_z_raw_degs : enc_rate;
  const Motor_State_t *left = Motor_GetState(MOTOR_LEFT);
  const Motor_State_t *right = Motor_GetState(MOTOR_RIGHT);
  bool stationary = (motors_enabled == 0U) &&
                    fabsf(left->velocity_mms) < 1.0f &&
                    fabsf(right->velocity_mms) < 1.0f;

  /* 2. Heading estimate: gyro + encoders (Kalman also learns the gyro bias). */
  Kalman_Update(&yaw_kf, gyro_rate, enc_rate, stationary, dt);

#if MOTOR_STALL_PROTECTION
  if (motors_enabled != 0U && Motor_CheckStall())
  {
    emergency_stop_generation++;
    Motors_Stop();
    stall_event_pending = 1U;
    return;
  }
#endif

  /* 3. Closed-loop motion */
  uint8_t mode = control_mode;
  if (mode != CTRL_VELOCITY && mode != CTRL_TURN) return;

  float heading = Kalman_GetHeading_deg(&yaw_kf);
  if (mode == CTRL_TURN)
  {
    /* Move the target heading towards the final angle at TURN_RATE_DEGS,
     * so the mouse turns smoothly instead of jumping. */
    float remaining = turn_final_deg - target_heading_deg;
    float step = TURN_RATE_DEGS * dt;
    if (fabsf(remaining) <= step)
    {
      target_heading_deg = turn_final_deg;
      target_rate_degs = 0.0f;
    }
    else
    {
      target_rate_degs = (remaining > 0.0f) ? TURN_RATE_DEGS : -TURN_RATE_DEGS;
      target_heading_deg += target_rate_degs * dt;
    }

    if (target_rate_degs == 0.0f &&
        fabsf(turn_final_deg - heading) < TURN_DONE_DEG &&
        fabsf(Kalman_GetAngularVelocity_degs(&yaw_kf)) < TURN_DONE_RATE_DEGS)
    {
      turn_settle_ms += (uint32_t)(dt * 1000.0f + 0.5f);
      if (turn_settle_ms >= TURN_SETTLE_MS)
      {
        Motors_Stop();
        turn_done_event_pending = 1U;
        return;
      }
    }
    else
    {
      turn_settle_ms = 0U;
    }
  }
  else
  {
    target_heading_deg += target_rate_degs * dt;
  }

  float forward = PID_Compute(&pid_forward, target_speed_mms,
                              Motor_GetLinearVelocity_mms(), dt);
  float turn = PID_Compute(&pid_rotation, target_heading_deg, heading, dt) +
               rot_feedforward * target_rate_degs;

  /* Turning left (CCW, +) means the right wheel goes faster. */
  float left_duty = ClampFloat(forward - turn, MOTOR_MAX_DUTY_PERCENT);
  float right_duty = ClampFloat(forward + turn, MOTOR_MAX_DUTY_PERCENT);

  /* Apply only if no STOP/timeout happened since this step started. */
  uint32_t interrupt_state = __get_PRIMASK();
  __disable_irq();
  if (control_mode == mode && motors_enabled != 0U)
  {
    Motor_SetDuty(MOTOR_LEFT, left_duty);
    Motor_SetDuty(MOTOR_RIGHT, right_duty);
    commanded_left_tenths = (int16_t)(left_duty * 10.0f);
    commanded_right_tenths = (int16_t)(right_duty * 10.0f);
  }
  if (interrupt_state == 0U) __enable_irq();
}

/* Format a float with one decimal without needing printf float support. */
static void FormatTenths(char *out, size_t size, float value)
{
  long tenths = lroundf(value * 10.0f);
  const char *sign = (tenths < 0) ? "-" : "";
  if (tenths < 0) tenths = -tenths;
  snprintf(out, size, "%s%ld.%ld", sign, tenths / 10L, tenths % 10L);
}

static long ToMilli(float value)
{
  return lroundf(value * 1000.0f);
}

static HAL_StatusTypeDef I2C_Write8(uint8_t address, uint16_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(address << 1), reg,
                           I2C_MEMADD_SIZE_16BIT, &value, 1U, 20U);
}

static HAL_StatusTypeDef I2C_Read(uint8_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(address << 1), reg,
                          I2C_MEMADD_SIZE_16BIT, data, length, 20U);
}

static const uint8_t vl53_default_configuration[] = {
  /* 0x2E/0x2F are 1 because this PCB pulls I2C/GPIO up to AVDD (3.3 V). */
  0x00,0x01,0x01,0x01,0x02,0x00,0x02,0x08,0x00,0x08,0x10,0x01,0x01,0x00,0x00,0x00,
  0x00,0xff,0x00,0x0f,0x00,0x00,0x00,0x00,0x00,0x20,0x0b,0x00,0x00,0x02,0x0a,0x21,
  0x00,0x00,0x05,0x00,0x00,0x00,0x00,0xc8,0x00,0x00,0x38,0xff,0x01,0x00,0x08,0x00,
  0x00,0x01,0xcc,0x0f,0x01,0xf1,0x0d,0x01,0x68,0x00,0x80,0x08,0xb8,0x00,0x00,0x00,
  0x00,0x0f,0x89,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x0f,0x0d,0x0e,0x0e,0x00,
  0x00,0x02,0xc7,0xff,0x9b,0x00,0x00,0x00,0x01,0x00,0x00
};

static uint8_t VL53_InitOne(GPIO_TypeDef *port, uint16_t pin, uint8_t address)
{
  uint8_t boot = 0U;
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  HAL_Delay(5);
  for (uint32_t start = HAL_GetTick(); HAL_GetTick() - start < 100U; )
  {
    if (I2C_Read(VL53_DEFAULT_ADDR, 0x00E5U, &boot, 1U) == HAL_OK && boot != 0U) break;
  }
  if (boot == 0U || I2C_Write8(VL53_DEFAULT_ADDR, 0x0001U, address) != HAL_OK) return 0U;
  for (uint16_t i = 0U; i < sizeof(vl53_default_configuration); i++)
  {
    if (I2C_Write8(address, (uint16_t)(0x002DU + i), vl53_default_configuration[i]) != HAL_OK) return 0U;
  }
  return (I2C_Write8(address, 0x0087U, 0x40U) == HAL_OK) ? 1U : 0U;
}

static void SensorHardware_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  ADC_ChannelConfTypeDef adc_channel = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_ADC3_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &gpio);

  hi2c1.Instance = I2C1;
  /* 100 kHz with the present 8 MHz I2C1 kernel clock. */
  hi2c1.Init.Timing = 0x30300909U;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();

  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpio);

  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK) Error_Handler();
  adc_channel.Channel = ADC_CHANNEL_10;
  adc_channel.Rank = ADC_REGULAR_RANK_1;
  adc_channel.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc3, &adc_channel);

  gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
             GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &gpio);
  HAL_GPIO_WritePin(GPIOE, gpio.Pin, GPIO_PIN_RESET);

  /* IR wall sensors: emitters PE2..PE5, receivers PC0..PC3 on ADC3 (ir.c). */
  IR_Init(&hadc3);

  /* LSM6DSO IMU on SPI2 (imu.c): 1.66 kHz, +/-2000 dps, +/-8 g. */
  if (IMU_Init()) sensor_flags |= FLAG_IMU;

  /* LIS2MDL breakout: absence is valid and is reported as not detected. */
  uint8_t who = 0U;
  if (HAL_I2C_Mem_Read(&hi2c1, LIS2MDL_ADDR << 1, 0x4FU, I2C_MEMADD_SIZE_8BIT, &who, 1U, 20U) == HAL_OK && who == 0x40U)
  {
    uint8_t config = 0x00U;
    HAL_I2C_Mem_Write(&hi2c1, LIS2MDL_ADDR << 1, 0x60U, I2C_MEMADD_SIZE_8BIT, &config, 1U, 20U);
    sensor_flags |= FLAG_MAG;
  }

  if (VL53_InitOne(GPIOE, GPIO_PIN_13, 0x2AU)) sensor_flags |= FLAG_TOF_1;
  if (VL53_InitOne(GPIOE, GPIO_PIN_14, 0x2BU)) sensor_flags |= FLAG_TOF_2;
  if (VL53_InitOne(GPIOE, GPIO_PIN_15, 0x2CU)) sensor_flags |= FLAG_TOF_3;
  sensor_flags |= FLAG_IR;
}

static void Sensors_Update(void)
{
  if ((sensor_flags & FLAG_IMU) != 0U)
  {
    /* The control loop reads the IMU every 1 ms; just copy the latest sample. */
    const IMU_Data_t *imu = IMU_GetData();
    for (uint8_t i = 0U; i < 3U; i++)
    {
      imu_values[i + 3U] = (int32_t)imu->gyro_raw[i] * 70L;            /* mdps */
      imu_values[i] = ((int32_t)imu->accel_raw[i] * 244L) / 1000L;    /* mg */
    }
  }

  if ((sensor_flags & FLAG_MAG) != 0U)
  {
    uint8_t raw[6] = {0};
    if (HAL_I2C_Mem_Read(&hi2c1, LIS2MDL_ADDR << 1, 0x68U, I2C_MEMADD_SIZE_8BIT, raw, 6U, 20U) == HAL_OK)
    {
      for (uint8_t i = 0U; i < 3U; i++)
      {
        int16_t value = (int16_t)(((uint16_t)raw[i * 2U + 1U] << 8) | raw[i * 2U]);
        mag_values[i] = ((int32_t)value * 3L) / 2L;
      }
    }
  }

  /* IR: pulsed, ambient-cancelled reading of all four sensors (ir.c).
   * Telemetry keeps sending the raw signal (lit - ambient) as before. */
  IR_Update();
  for (uint8_t i = 0U; i < IR_COUNT; i++)
  {
    ir_values[i] = IR_GetSignal(i);
  }

  const uint8_t addresses[3] = {0x2AU, 0x2BU, 0x2CU};
  const uint8_t flags[3] = {FLAG_TOF_1, FLAG_TOF_2, FLAG_TOF_3};
  for (uint8_t i = 0U; i < 3U; i++)
  {
    if ((sensor_flags & flags[i]) == 0U) continue;
    uint8_t ready = 0U, range[2] = {0};
    if (I2C_Read(addresses[i], 0x0031U, &ready, 1U) == HAL_OK && (ready & 1U) != 0U &&
        I2C_Read(addresses[i], 0x0096U, range, 2U) == HAL_OK)
    {
      tof_values[i] = (uint16_t)(((uint16_t)range[0] << 8) | range[1]);
      I2C_Write8(addresses[i], 0x0086U, 0x01U);
    }
  }
}

static void Protocol_Send(const char *text)
{
  uint8_t start_tx = 0U;
  size_t length = strlen(text);
  if (length > 511U) return;
  uint32_t interrupt_state = __get_PRIMASK();
  __disable_irq();
  uint16_t used = (uint16_t)((uart_tx_head - uart_tx_tail) & 0x01FFU);
  uint16_t free_bytes = (uint16_t)(511U - used);
  if (length <= free_bytes)
  {
    while (*text != '\0')
    {
      uart_tx_ring[uart_tx_head] = (uint8_t)*text++;
      uart_tx_head = (uint16_t)((uart_tx_head + 1U) & 0x01FFU);
    }
  }
  if (uart_tx_busy == 0U && uart_tx_tail != uart_tx_head)
  {
    uart_tx_busy = 1U;
    uart_tx_byte = uart_tx_ring[uart_tx_tail];
    uart_tx_tail = (uint16_t)((uart_tx_tail + 1U) & 0x01FFU);
    start_tx = 1U;
  }
  if (interrupt_state == 0U) __enable_irq();

  if (start_tx != 0U && HAL_UART_Transmit_IT(&huart1, &uart_tx_byte, 1U) != HAL_OK)
  {
    uart_tx_busy = 0U;
  }
}

static uint8_t UART_PopByte(uint8_t *value)
{
  uint8_t available = 0U;
  uint32_t interrupt_state = __get_PRIMASK();
  __disable_irq();
  if (uart_rx_tail != uart_rx_head)
  {
    *value = uart_rx_ring[uart_rx_tail];
    uart_rx_tail = (uint8_t)((uart_rx_tail + 1U) & 0x7FU);
    available = 1U;
  }
  if (interrupt_state == 0U) __enable_irq();
  return available;
}

static void Protocol_HandleLine(char *line, uint32_t command_generation)
{
  int left_tenths = 0, right_tenths = 0;
  int left_percent = 0, right_percent = 0;
  long duration = 0L;
  int motor_fields = sscanf(line, "M10 %d %d %ld",
                            &left_tenths, &right_tenths, &duration);
  if (motor_fields != 3 &&
      sscanf(line, "M %d %d %ld", &left_percent, &right_percent, &duration) == 3)
  {
    left_tenths = ClampMotorPercent(left_percent) * 10;
    right_tenths = ClampMotorPercent(right_percent) * 10;
    motor_fields = 3;
  }

  if (motor_fields == 3)
  {
    if (duration <= 0L)
    {
      Motors_Stop();
    }
    else
    {
      if (duration > (long)MOTOR_MAX_RUN_MS) duration = MOTOR_MAX_RUN_MS;
      if (Motors_SetSignedTenths(left_tenths, right_tenths,
                                 command_generation) != 0U)
      {
        motor_stop_tick = HAL_GetTick() + (uint32_t)duration;
      }
      else
      {
        Protocol_Send("ERR STALE_MOTOR_COMMAND\r\n");
        return;
      }
    }
    Protocol_Send("ACK M\r\n");
  }
  else if (strcmp(line, "STOP") == 0)
  {
    Motors_Stop();
    Protocol_Send("ACK STOP\r\n");
  }
  else if (strcmp(line, "ZERO_ENC") == 0)
  {
    /* Zero the distance AND the heading (handy before a test run). */
    Motor_ResetOdometry();
    Kalman_ResetHeading(&yaw_kf, 0.0f);
    target_heading_deg = 0.0f;
    turn_final_deg = 0.0f;
    Protocol_Send("ACK ZERO_ENC\r\n");
  }
  else if (sscanf(line, "V %d %d %ld", &left_percent, &right_percent, &duration) == 3)
  {
    /* V <speed mm/s> <turn rate deg/s> <ms>: closed-loop drive.
     * V 200 0 2000 = straight at 200 mm/s for 2 s, holding the heading. */
    if (left_percent > SPEED_LIMIT_MMS) left_percent = SPEED_LIMIT_MMS;
    if (left_percent < -SPEED_LIMIT_MMS) left_percent = -SPEED_LIMIT_MMS;
    if (right_percent > RATE_LIMIT_DEGS) right_percent = RATE_LIMIT_DEGS;
    if (right_percent < -RATE_LIMIT_DEGS) right_percent = -RATE_LIMIT_DEGS;
    if (duration <= 0L)
    {
      Motors_Stop();
    }
    else
    {
      if (duration > (long)MOTOR_MAX_RUN_MS) duration = MOTOR_MAX_RUN_MS;
      if (Motion_StartVelocity((float)left_percent, (float)right_percent,
                               command_generation) == 0U)
      {
        Protocol_Send("ERR STALE_MOTOR_COMMAND\r\n");
        return;
      }
      motor_stop_tick = HAL_GetTick() + (uint32_t)duration;
    }
    Protocol_Send("ACK V\r\n");
  }
  else if (sscanf(line, "TURN %d", &left_percent) == 1)
  {
    /* TURN <deg> [timeout ms]: turn on the spot, + = left. EVENT TURN_DONE
     * is sent when it arrives; EVENT MOTOR_TIMEOUT if it never does. */
    if (sscanf(line, "TURN %d %ld", &left_percent, &duration) != 2) duration = TURN_DEFAULT_MS;
    if (left_percent > 720) left_percent = 720;
    if (left_percent < -720) left_percent = -720;
    if (duration <= 0L || duration > (long)MOTOR_MAX_RUN_MS) duration = TURN_DEFAULT_MS;
    if (Motion_StartTurn((float)left_percent, command_generation) == 0U)
    {
      Protocol_Send("ERR STALE_MOTOR_COMMAND\r\n");
      return;
    }
    motor_stop_tick = HAL_GetTick() + (uint32_t)duration;
    Protocol_Send("ACK TURN\r\n");
  }
  else if (strcmp(line, "POSE") == 0)
  {
    /* Heading (deg, + = left), turn rate (deg/s), learnt gyro bias (deg/s),
     * distance (mm), speed (mm/s), and the raw encoder-only heading. */
    char head[16], rate[16], bias[16], dist[16], vel[16], enc[16], pose_text[160];
    FormatTenths(head, sizeof(head), Kalman_GetHeading_deg(&yaw_kf));
    FormatTenths(rate, sizeof(rate), Kalman_GetAngularVelocity_degs(&yaw_kf));
    FormatTenths(bias, sizeof(bias), Kalman_GetGyroBias_degs(&yaw_kf));
    FormatTenths(dist, sizeof(dist), Motor_GetLinearDistance_mm());
    FormatTenths(vel, sizeof(vel), Motor_GetLinearVelocity_mms());
    FormatTenths(enc, sizeof(enc), Motor_GetHeading_deg());
    int n = snprintf(pose_text, sizeof(pose_text),
                     "POSE HEAD %s RATE %s BIAS %s DIST %s VEL %s ENCHEAD %s IMU %u\r\n",
                     head, rate, bias, dist, vel, enc, IMU_IsPresent() ? 1U : 0U);
    if (n > 0 && n < (int)sizeof(pose_text)) Protocol_Send(pose_text);
  }
  else if (strncmp(line, "GAINS", 5) == 0)
  {
    /* GAINS                      -> print gains (all values x1000)
     * GAINS F <kp> <ki> <kd> <kf> -> forward speed loop, x1000
     * GAINS R <kp> <ki> <kd> <ff> -> heading loop, x1000
     * Example: GAINS F 20 200 0 70  means Kp=0.020 Ki=0.200 Kd=0 Kf=0.070 */
    long g[4] = {0};
    char which = 0;
    char gains_text[160];
    if (sscanf(line, "GAINS %c %ld %ld %ld %ld", &which, &g[0], &g[1], &g[2], &g[3]) == 5)
    {
      if (motors_enabled != 0U)
      {
        Protocol_Send("ERR GAINS_WHILE_MOVING\r\n");
        return;
      }
      if (which == 'F' || which == 'f')
      {
        PID_SetGains(&pid_forward, g[0] / 1000.0f, g[1] / 1000.0f, g[2] / 1000.0f, g[3] / 1000.0f);
      }
      else if (which == 'R' || which == 'r')
      {
        PID_SetGains(&pid_rotation, g[0] / 1000.0f, g[1] / 1000.0f, g[2] / 1000.0f, 0.0f);
        rot_feedforward = g[3] / 1000.0f;
      }
      else
      {
        Protocol_Send("ERR GAINS_ARGS (use GAINS F|R kp ki kd ff, x1000)\r\n");
        return;
      }
    }
    else if (strcmp(line, "GAINS") != 0)
    {
      Protocol_Send("ERR GAINS_ARGS (use GAINS F|R kp ki kd ff, x1000)\r\n");
      return;
    }
    int n = snprintf(gains_text, sizeof(gains_text),
                     "GAINS F %ld %ld %ld %ld R %ld %ld %ld %ld\r\n",
                     ToMilli(pid_forward.Kp), ToMilli(pid_forward.Ki),
                     ToMilli(pid_forward.Kd), ToMilli(pid_forward.Kf),
                     ToMilli(pid_rotation.Kp), ToMilli(pid_rotation.Ki),
                     ToMilli(pid_rotation.Kd), ToMilli(rot_feedforward));
    if (n > 0 && n < (int)sizeof(gains_text)) Protocol_Send(gains_text);
  }
  else if (sscanf(line, "TELEM %ld", &duration) == 1)
  {
    if (duration < 200L) duration = 200L;
    if (duration > 2000L) duration = 2000L;
    telemetry_period_ms = (uint32_t)duration;
    Protocol_Send("ACK TELEM\r\n");
  }
  else if (strcmp(line, "IR") == 0)
  {
    /* Fresh IR reading: signal, ambient, distance (mm) and wall bits. */
    char ir_text[200];
    IR_Update();
    int n = snprintf(ir_text, sizeof(ir_text),
      "IR SIG %u %u %u %u AMB %u %u %u %u MM %u %u %u %u WALL %u%u%u%u\r\n",
      IR_GetSignal(0), IR_GetSignal(1), IR_GetSignal(2), IR_GetSignal(3),
      IR_GetAmbient(0), IR_GetAmbient(1), IR_GetAmbient(2), IR_GetAmbient(3),
      IR_GetDistance_mm(0), IR_GetDistance_mm(1),
      IR_GetDistance_mm(2), IR_GetDistance_mm(3),
      IR_IsWall(0), IR_IsWall(1), IR_IsWall(2), IR_IsWall(3));
    if (n > 0 && n < (int)sizeof(ir_text)) Protocol_Send(ir_text);
  }
  else if (sscanf(line, "IRCAL %d %d", &left_percent, &right_percent) == 2)
  {
    /* IRCAL <sensor 1-4> <mm>: wall held at <mm> from that sensor. */
    char ir_text[64];
    if (left_percent < 1 || left_percent > (int)IR_COUNT ||
        right_percent < 10 || right_percent > (int)IR_DISTANCE_MAX_MM)
    {
      Protocol_Send("ERR IRCAL_ARGS (use IRCAL <1-4> <10-300>)\r\n");
    }
    else if (IR_Calibrate((uint8_t)(left_percent - 1), (uint16_t)right_percent) != 0U)
    {
      int n = snprintf(ir_text, sizeof(ir_text), "ACK IRCAL %d K=%lu\r\n", left_percent,
                       (unsigned long)IR_GetCalK((uint8_t)(left_percent - 1)));
      if (n > 0 && n < (int)sizeof(ir_text)) Protocol_Send(ir_text);
    }
    else
    {
      Protocol_Send("ERR IRCAL_SIGNAL (too weak or saturated)\r\n");
    }
  }
  else if (sscanf(line, "IRWALL %d %d", &left_percent, &right_percent) == 2)
  {
    /* IRWALL <sensor 1-4> <mm>: a wall closer than <mm> counts as a wall. */
    char ir_text[48];
    if (left_percent < 1 || left_percent > (int)IR_COUNT ||
        right_percent < 10 || right_percent > (int)IR_DISTANCE_MAX_MM)
    {
      Protocol_Send("ERR IRWALL_ARGS (use IRWALL <1-4> <10-300>)\r\n");
    }
    else
    {
      IR_SetWallThreshold_mm((uint8_t)(left_percent - 1), (uint16_t)right_percent);
      int n = snprintf(ir_text, sizeof(ir_text), "ACK IRWALL %d %d\r\n",
                       left_percent, right_percent);
      if (n > 0 && n < (int)sizeof(ir_text)) Protocol_Send(ir_text);
    }
  }
  else if (sscanf(line, "IRTEST %d", &left_percent) == 1)
  {
    /* IRTEST <1-4>: hold that IR LED on for 2 s. Point a phone camera at it:
     * a working emitter shows as a purple/white glow on the screen. */
    if (left_percent < 1 || left_percent > (int)IR_COUNT)
    {
      Protocol_Send("ERR IRTEST_ARGS (use IRTEST <1-4>)\r\n");
    }
    else if (motors_enabled != 0U)
    {
      Protocol_Send("ERR IRTEST_MOTORS_RUNNING\r\n");
    }
    else
    {
      IR_SetEmitter((uint8_t)(left_percent - 1), 1U);
      HAL_Delay(2000U);
      IR_SetEmitter((uint8_t)(left_percent - 1), 0U);
      Protocol_Send("ACK IRTEST\r\n");
    }
  }
  else if (strcmp(line, "IRK") == 0)
  {
    /* Print the calibration constants so they can be copied into ir.c. */
    char ir_text[96];
    int n = snprintf(ir_text, sizeof(ir_text), "IRK %lu %lu %lu %lu\r\n",
                     (unsigned long)IR_GetCalK(0), (unsigned long)IR_GetCalK(1),
                     (unsigned long)IR_GetCalK(2), (unsigned long)IR_GetCalK(3));
    if (n > 0 && n < (int)sizeof(ir_text)) Protocol_Send(ir_text);
  }
  else if (strcmp(line, "HELLO") == 0 || strcmp(line, "?") == 0)
  {
    Protocol_Send("HELLO MICROMOUSE_HUB 2 MAX_PWM=71.0\r\n");
  }
  else
  {
    Protocol_Send("ERR UNKNOWN_COMMAND\r\n");
  }
}

static void Protocol_SendTelemetry(void)
{
  char output[256];
  uint8_t flags = sensor_flags;
  if (HAL_GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin) == GPIO_PIN_SET) flags |= FLAG_BT_CONNECTED;
  if (motors_enabled != 0U) flags |= FLAG_MOTORS_ACTIVE;
  int n = snprintf(output, sizeof(output),
    "T,%lu,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
    (unsigned long)HAL_GetTick(), commanded_left_tenths, commanded_right_tenths,
    (long)Motor_GetState(MOTOR_LEFT)->total_ticks, (long)Motor_GetState(MOTOR_RIGHT)->total_ticks,
    (long)((sensor_flags & FLAG_IMU) ? imu_values[0] : SENSOR_MISSING),
    (long)((sensor_flags & FLAG_IMU) ? imu_values[1] : SENSOR_MISSING),
    (long)((sensor_flags & FLAG_IMU) ? imu_values[2] : SENSOR_MISSING),
    (long)((sensor_flags & FLAG_IMU) ? imu_values[3] : SENSOR_MISSING),
    (long)((sensor_flags & FLAG_IMU) ? imu_values[4] : SENSOR_MISSING),
    (long)((sensor_flags & FLAG_IMU) ? imu_values[5] : SENSOR_MISSING),
    (long)((sensor_flags & FLAG_MAG) ? mag_values[0] : SENSOR_MISSING),
    (long)((sensor_flags & FLAG_MAG) ? mag_values[1] : SENSOR_MISSING),
    (long)((sensor_flags & FLAG_MAG) ? mag_values[2] : SENSOR_MISSING),
    (unsigned)ir_values[0], (unsigned)ir_values[1],
    (unsigned)ir_values[2], (unsigned)ir_values[3],
    (unsigned)tof_values[0], (unsigned)tof_values[1],
    (unsigned)tof_values[2], (unsigned)flags);
  if (n > 0 && n < (int)sizeof(output)) Protocol_Send(output);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t received_byte = 0U;
  char command_line[64] = {0};
  uint8_t command_length = 0U;
  uint32_t command_generation = 0U;
  uint32_t observed_stop_flush = 0U;
  uint32_t last_telemetry_tick = 0U;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* motor.c: start PWM + encoders, then keep both drivers asleep until a
   * drive command arrives. */
  Motor_Init();
  Motors_Stop();

  HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  SensorHardware_Init();

  /* Gyro zero-offset: the mouse must sit still for ~0.5 s after power-up. */
  Protocol_Send("INFO CALIBRATING_GYRO keep the mouse still\r\n");
  Kalman_Init(&yaw_kf, IMU_CalibrateGyroBias(500U));
  Control_Init();
  Protocol_Send("HELLO MICROMOUSE_HUB 2 MAX_PWM=71.0\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Control_Tick();

    /* RX interrupts keep collecting bytes even during the 9600-baud telemetry TX. */
    while (UART_PopByte(&received_byte) != 0U)
    {
      uint32_t current_flush = uart_stop_flush_generation;
      if (observed_stop_flush != current_flush)
      {
        command_length = 0U;
        observed_stop_flush = current_flush;
        continue; /* This byte may have been popped just before the ISR flush. */
      }
      if (received_byte == '\n' || received_byte == '\r')
      {
        if (command_length > 0U)
        {
          command_line[command_length] = '\0';
          Protocol_HandleLine(command_line, command_generation);
          command_length = 0U;
        }
      }
      else if (command_length < sizeof(command_line) - 1U)
      {
        if (command_length == 0U) command_generation = emergency_stop_generation;
        command_line[command_length++] = (char)received_byte;
      }
      else
      {
        command_length = 0U;
        Protocol_Send("ERR COMMAND_TOO_LONG\r\n");
      }
    }

    if (motor_timeout_event_pending != 0U)
    {
      motor_timeout_event_pending = 0U;
      Protocol_Send("EVENT MOTOR_TIMEOUT\r\n");
    }

    if (turn_done_event_pending != 0U)
    {
      turn_done_event_pending = 0U;
      Protocol_Send("EVENT TURN_DONE\r\n");
    }

#if MOTOR_STALL_PROTECTION
    if (stall_event_pending != 0U)
    {
      stall_event_pending = 0U;
      Protocol_Send("EVENT MOTOR_STALL\r\n");
    }
#endif

    if (emergency_stop_ack_pending != 0U)
    {
      emergency_stop_ack_pending = 0U;
      Protocol_Send("ACK STOP\r\n");
    }

    if (HAL_GetTick() - last_telemetry_tick >= telemetry_period_ms)
    {
      last_telemetry_tick = HAL_GetTick();
      Sensors_Update();
      Protocol_SendTelemetry();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /* HC-05 module enable is active high on the installed breakout. */
  HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, NSLEEP_HB_1_Pin|NSLEEP_HB_2_Pin, GPIO_PIN_RESET);

  /* Keep the SPI device deselected until a transaction begins. */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NSLEEP_HB_1_Pin NSLEEP_HB_2_Pin */
  GPIO_InitStruct.Pin = NSLEEP_HB_1_Pin|NSLEEP_HB_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* HC-05 EN: keep high so the Bluetooth module remains powered/enabled. */
  GPIO_InitStruct.Pin = BT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_STATE_Pin */
  GPIO_InitStruct.Pin = BT_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT_STATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_INT_1_Pin IMU_INT_2_Pin */
  GPIO_InitStruct.Pin = IMU_INT_1_Pin|IMU_INT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Micromouse_SafetyTick(void)
{
  if (motors_enabled != 0U &&
      motor_stop_tick != 0U && (int32_t)(HAL_GetTick() - motor_stop_tick) >= 0)
  {
    emergency_stop_generation++;
    Motors_Stop();
    motor_timeout_event_pending = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    static const uint8_t stop_text[4] = {'S', 'T', 'O', 'P'};
    if (uart_rx_byte == '\r' || uart_rx_byte == '\n')
    {
      if (uart_stop_candidate != 0U && uart_stop_index == sizeof(stop_text))
      {
        emergency_stop_generation++;
        Motors_Stop();
        uart_rx_tail = uart_rx_head; /* Discard every command received before STOP. */
        uart_stop_flush_generation++;
        emergency_stop_ack_pending = 1U;
      }
      uart_stop_candidate = 1U;
      uart_stop_index = 0U;
    }
    else if (uart_stop_candidate != 0U)
    {
      if (uart_stop_index < sizeof(stop_text) && uart_rx_byte == stop_text[uart_stop_index])
      {
        uart_stop_index++;
      }
      else
      {
        uart_stop_candidate = 0U;
      }
    }

    uint8_t next = (uint8_t)((uart_rx_head + 1U) & 0x7FU);
    if (next != uart_rx_tail)
    {
      uart_rx_ring[uart_rx_head] = uart_rx_byte;
      uart_rx_head = next;
    }
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (uart_tx_tail != uart_tx_head)
    {
      uart_tx_byte = uart_tx_ring[uart_tx_tail];
      uart_tx_tail = (uint16_t)((uart_tx_tail + 1U) & 0x01FFU);
      if (HAL_UART_Transmit_IT(&huart1, &uart_tx_byte, 1U) != HAL_OK)
      {
        uart_tx_busy = 0U;
      }
    }
    else
    {
      uart_tx_busy = 0U;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
