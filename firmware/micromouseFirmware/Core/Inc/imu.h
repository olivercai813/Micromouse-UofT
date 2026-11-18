/**
  ******************************************************************************
  * @file           : imu.h
  * @brief          : SPI IMU driver with stationary bias calibration and
  *                   off-center lever-arm compensation for STM32F746VETx.
  ******************************************************************************
  */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                 EXPLICIT GPIO & PERIPHERAL PINOUT DEFINITIONS              */
/* ========================================================================== */
#define IMU_SPI_HANDLE            (&hspi2)
#define IMU_CS_GPIO_PORT          GPIOB
#define IMU_CS_GPIO_PIN           GPIO_PIN_12       /* PB12 -> IMU_CS         */
#define IMU_SCK_GPIO_PORT         GPIOB
#define IMU_SCK_GPIO_PIN          GPIO_PIN_13       /* PB13 -> SPI2_SCK       */
#define IMU_MISO_GPIO_PORT        GPIOB
#define IMU_MISO_GPIO_PIN         GPIO_PIN_14       /* PB14 -> SPI2_MISO      */
#define IMU_MOSI_GPIO_PORT        GPIOB
#define IMU_MOSI_GPIO_PIN         GPIO_PIN_15       /* PB15 -> SPI2_MOSI      */
#define IMU_INT1_GPIO_PORT        GPIOC
#define IMU_INT1_GPIO_PIN         GPIO_PIN_6        /* PC6  -> IMU_INT_1      */
#define IMU_INT2_GPIO_PORT        GPIOC
#define IMU_INT2_GPIO_PIN         GPIO_PIN_7        /* PC7  -> IMU_INT_2      */

/* ========================================================================== */
/*              CONFIGURABLE OFF-CENTER LEVER-ARM OFFSETS (Meters)            */
/* ========================================================================== */
/* Distance from midpoint between Left/Right wheels to IMU sensing element:   */
#define IMU_OFFSET_X_M            0.015f   /* +X = 15 mm forward of axle      */
#define IMU_OFFSET_Y_M            0.000f   /* +Y = 0 mm left of centerline    */
#define IMU_GYRO_Z_INVERT         1.0f     /* Set to -1.0f if CCW is negative */

/* ========================================================================== */
/*             CONFIGURABLE IMU REGISTER MAP (Default: ST LSM6DSx)            */
/* ========================================================================== */
#define IMU_REG_WHO_AM_I          0x0FU
#define IMU_REG_CTRL1_XL          0x10U    /* Accelerometer ODR & Full-scale  */
#define IMU_REG_CTRL2_G           0x11U    /* Gyroscope ODR & Full-scale      */
#define IMU_REG_CTRL3_C           0x12U    /* BDU & Auto-increment            */
#define IMU_REG_OUTX_L_G          0x22U    /* Start of 12-byte Gyro+Accel burst*/

/* Sensitivity for +-2000 dps (70 mdps/LSB) and +-8g (0.244 mg/LSB) */
#define IMU_GYRO_SENS_DPS_LSB     0.070f
#define IMU_ACCEL_SENS_G_LSB      0.000244f
#define IMU_GRAVITY_MS2           9.80665f
#define IMU_DEG_TO_RAD            0.01745329252f

/* ========================================================================== */
/*                              DATA STRUCTURES                               */
/* ========================================================================== */
typedef struct {
    float gyro_z_raw_degs;        /* Raw Z-axis turn rate (deg/s, CCW+)       */
    float gyro_z_bias_degs;       /* Stationary boot bias offset (deg/s)      */
    float gyro_z_degs;            /* Bias-corrected Z turn rate (deg/s)       */
    float alpha_z_rads2;          /* Filtered angular acceleration (rad/s^2)  */

    float accel_x_imu_ms2;        /* Raw forward accel at IMU location (m/s^2)*/
    float accel_y_imu_ms2;        /* Raw lateral accel at IMU location (m/s^2)*/
    float accel_x_center_ms2;     /* Lever-arm compensated CoR accel (m/s^2)  */
    float accel_y_center_ms2;     /* Lever-arm compensated CoR accel (m/s^2)  */
} IMU_Data_t;

/* ========================================================================== */
/*                            FUNCTION PROTOTYPES                             */
/* ========================================================================== */
bool              IMU_Init(void);
float             IMU_CalibrateGyroBias(uint16_t num_samples);
void              IMU_Update(float dt_s);
const IMU_Data_t* IMU_GetData(void);
float             IMU_GetGyroZ_degs(void);
float             IMU_GetCenterAccelX_ms2(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_H_ */

