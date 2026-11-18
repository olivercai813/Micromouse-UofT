/**
  ******************************************************************************
  * @file           : imu.c
  * @brief          : SPI2 IMU driver with stationary bias calibration and
  *                   off-center lever-arm compensation for STM32F746VETx.
  *
  * ============================================================================
  *                     COMPLETE IMU GPIO & FUNCTION CALLOUT TABLE
  * ============================================================================
  * Pin   | Schematic Net | Peripheral / Mode     | Hardware Function
  * ------|---------------|-----------------------|-----------------------------
  * PB12  | IMU_CS        | GPIO_Output (PushPull)| SPI2 Chip Select (Active LOW)
  * PB13  | IMU_CLK       | SPI2_SCK (AF5)        | SPI2 Serial Clock
  * PB14  | IMU_MISO      | SPI2_MISO (AF5)       | SPI2 Master In Slave Out
  * PB15  | IMU_MOSI      | SPI2_MOSI (AF5)       | SPI2 Master Out Slave In
  * PC6   | IMU_INT_1     | GPIO_EXTI6 (Input)    | IMU Data-Ready Interrupt 1
  * PC7   | IMU_INT_2     | GPIO_EXTI7 (Input)    | IMU Interrupt 2
  * ============================================================================
  */

#include "imu.h"

/* External SPI2 handle defined in main.c (enable SPI2 in CubeMX on PB13..PB15) */
extern SPI_HandleTypeDef hspi2;

static IMU_Data_t imu_data = {0};
static float      prev_omega_rads = 0.0f;

static inline void IMU_CS_Low(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_PORT, IMU_CS_GPIO_PIN, GPIO_PIN_RESET);
}

static inline void IMU_CS_High(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_PORT, IMU_CS_GPIO_PIN, GPIO_PIN_SET);
}

static void IMU_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), val };
    IMU_CS_Low();
    HAL_SPI_Transmit(IMU_SPI_HANDLE, tx, 2U, 5U);
    IMU_CS_High();
}

static void IMU_ReadRegs(uint8_t start_reg, uint8_t *rx_buf, uint16_t len)
{
    uint8_t reg_addr = (uint8_t)(start_reg | 0x80U); /* MSB = 1 for SPI Read */
    IMU_CS_Low();
    HAL_SPI_Transmit(IMU_SPI_HANDLE, &reg_addr, 1U, 5U);
    HAL_SPI_Receive(IMU_SPI_HANDLE, rx_buf, len, 5U);
    IMU_CS_High();
}

/**
  * @brief  Initializes the SPI IMU (1.66 kHz ODR, +-2000 dps gyro, +-8g accel).
  */
bool IMU_Init(void)
{
    IMU_CS_High();
    HAL_Delay(10);

    uint8_t who_am_i = 0U;
    IMU_ReadRegs(IMU_REG_WHO_AM_I, &who_am_i, 1U);

    /* Enable Block Data Update (BDU) and Register Address Auto-Increment */
    IMU_WriteReg(IMU_REG_CTRL3_C, 0x44U);

    /* Configure Accel: 1.66 kHz ODR, +-8g full-scale, internal LPF enabled */
    IMU_WriteReg(IMU_REG_CTRL1_XL, 0x8CU);

    /* Configure Gyro: 1.66 kHz ODR, +-2000 dps full-scale */
    IMU_WriteReg(IMU_REG_CTRL2_G, 0x8CU);

    HAL_Delay(20);
    return (who_am_i != 0x00U && who_am_i != 0xFFU);
}

/**
  * @brief  Samples the gyroscope while stationary at boot to find the zero-rate
  *         offset (bias). Keep the Micromouse still during this call!
  */
float IMU_CalibrateGyroBias(uint16_t num_samples)
{
    if (num_samples == 0U) num_samples = 500U;

    float sum_degs = 0.0f;
    for (uint16_t i = 0; i < num_samples; i++) {
        IMU_Update(0.001f);
        sum_degs += imu_data.gyro_z_raw_degs;
        HAL_Delay(1);
    }

    imu_data.gyro_z_bias_degs = sum_degs / (float)num_samples;
    return imu_data.gyro_z_bias_degs;
}

/**
  * @brief  Reads raw 6-axis data over SPI2, applies bias correction, and removes
  *         off-center centripetal and tangential acceleration lever-arm effects.
  *         Call at 1 kHz right before Kalman_Update().
  */
void IMU_Update(float dt_s)
{
    uint8_t raw_buf[12] = {0};
    IMU_ReadRegs(IMU_REG_OUTX_L_G, raw_buf, 12U);

    /* Combine little-endian 16-bit signed registers:
     * Bytes 0..5:  GX, GY, GZ
     * Bytes 6..11: AX, AY, AZ
     */
    int16_t gz_raw = (int16_t)(((uint16_t)raw_buf[5] << 8) | raw_buf[4]);
    int16_t ax_raw = (int16_t)(((uint16_t)raw_buf[7] << 8) | raw_buf[6]);
    int16_t ay_raw = (int16_t)(((uint16_t)raw_buf[9] << 8) | raw_buf[8]);

    /* 1. Convert Gyro Z to deg/s (Rigid-body invariant: unaffected by offset) */
    imu_data.gyro_z_raw_degs = (float)gz_raw * IMU_GYRO_SENS_DPS_LSB * IMU_GYRO_Z_INVERT;
    imu_data.gyro_z_degs     = imu_data.gyro_z_raw_degs - imu_data.gyro_z_bias_degs;

    /* 2. Convert Accel X and Y to m/s^2 at the physical IMU location */
    imu_data.accel_x_imu_ms2 = (float)ax_raw * IMU_ACCEL_SENS_G_LSB * IMU_GRAVITY_MS2;
    imu_data.accel_y_imu_ms2 = (float)ay_raw * IMU_ACCEL_SENS_G_LSB * IMU_GRAVITY_MS2;

    /* 3. Off-Center Lever-Arm Compensation for Accelerometer:
     *    a_center = a_imu + (omega^2 * r) + (alpha x r)
     */
    float omega_rads = imu_data.gyro_z_degs * IMU_DEG_TO_RAD;
    if (dt_s > 0.0f) {
        float inst_alpha = (omega_rads - prev_omega_rads) / dt_s;
        imu_data.alpha_z_rads2 = (0.2f * inst_alpha) + (0.8f * imu_data.alpha_z_rads2);
        prev_omega_rads = omega_rads;
    }

    float omega_sq = omega_rads * omega_rads;
    imu_data.accel_x_center_ms2 = imu_data.accel_x_imu_ms2
                                + (omega_sq * IMU_OFFSET_X_M)
                                + (imu_data.alpha_z_rads2 * IMU_OFFSET_Y_M);

    imu_data.accel_y_center_ms2 = imu_data.accel_y_imu_ms2
                                + (omega_sq * IMU_OFFSET_Y_M)
                                - (imu_data.alpha_z_rads2 * IMU_OFFSET_X_M);
}

const IMU_Data_t* IMU_GetData(void)
{
    return &imu_data;
}

float IMU_GetGyroZ_degs(void)
{
    return imu_data.gyro_z_degs;
}

float IMU_GetCenterAccelX_ms2(void)
{
    return imu_data.accel_x_center_ms2;
}

