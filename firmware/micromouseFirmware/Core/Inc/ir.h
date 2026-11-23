/**
  ******************************************************************************
  * @file           : ir.h
  * @brief          : IR wall-sensor driver for the micromouse (4 x SFH4550
  *                   emitter + SFH300 phototransistor boards on ADC3).
  ******************************************************************************
  *
  * How one IR board works (IR_sensor_project.kicad_sch):
  *   - IR_POW_n (STM32 output) drives an N-MOSFET (DMN3065) gate. HIGH turns
  *     the SFH4550 infrared LED on (~18 mA through a 100 ohm resistor).
  *     A 100k gate pull-down keeps the LED off when the pin is not driven.
  *   - The SFH300 phototransistor has a 10k resistor to GND on its emitter, so
  *     IR_REC_n is a voltage that RISES when more infrared light hits it.
  *
  * How a reading is taken (one sensor at a time, so they never see each
  * other's light):
  *   1. LED off  -> read IR_REC  = "ambient" (room light, sunlight)
  *   2. LED on, wait IR_SETTLE_US -> read IR_REC = "lit"
  *   3. LED off. signal = lit - ambient = light reflected by the wall only.
  * A closer wall reflects more light, so a bigger signal = a closer wall.
  * Reflected light falls off roughly with 1/distance^2, so
  *      distance_mm ~= sqrt(K / signal)
  * where K is found per sensor with IR_Calibrate() (or the IRCAL command).
  *
  * Pin map (README "KiCad-verified hardware map"):
  *   Sensor 1: emitter PE2  (IR_POW_1), receiver PC0 (ADC3_IN10)
  *   Sensor 2: emitter PE3  (IR_POW_2), receiver PC1 (ADC3_IN11)
  *   Sensor 3: emitter PE4  (IR_POW_3), receiver PC2 (ADC3_IN12)
  *   Sensor 4: emitter PE5  (IR_POW_4), receiver PC3 (ADC3_IN13)
  * TODO(team): note here which physical position (left / front-left /
  *             front-right / right) each sensor number is mounted at.
  ******************************************************************************
  */

#ifndef INC_IR_H_
#define INC_IR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ========================================================================== */
/*                              CONFIGURATION                                 */
/* ========================================================================== */
#define IR_COUNT                 4U

/* Time the LED is on before the "lit" reading. The SFH300 with a 10k load
 * settles in tens of microseconds; 80 us leaves margin. */
#define IR_SETTLE_US             80U

/* ADC readings averaged for each ambient / lit measurement (noise reduction). */
#define IR_SAMPLES               4U

/* Signals below this are treated as "nothing in front of the sensor". */
#define IR_SIGNAL_MIN            15U

/* A lit reading at or above this is clipped (wall too close / too bright). */
#define IR_SATURATION            4000U

/* Distance reported when no wall is seen. */
#define IR_DISTANCE_MAX_MM       300U

/* Default calibration constant K = signal * distance_mm^2.
 * PLACEHOLDER until each sensor is calibrated with IRCAL. */
#define IR_DEFAULT_K             5000000UL

/* Default "wall present" distance: a wall closer than this counts as a wall.
 * A maze cell is 180 mm; tune per sensor with IR_SetWallThreshold_mm(). */
#define IR_DEFAULT_WALL_MM       120U

/* ========================================================================== */
/*                                   API                                      */
/* ========================================================================== */
/* Call once at startup, after ADC3 and the PE2..PE5 outputs are initialised.
 * hadc: the ADC3 handle that samples PC0..PC3. */
void     IR_Init(ADC_HandleTypeDef *hadc);

/* Measure all four sensors (blocking, about 1 ms in total). */
void     IR_Update(void);

/* Results of the last IR_Update(). index = 0..3 for sensors 1..4. */
uint16_t IR_GetSignal(uint8_t index);       /* lit - ambient, ADC counts   */
uint16_t IR_GetAmbient(uint8_t index);      /* background light, counts    */
uint16_t IR_GetDistance_mm(uint8_t index);  /* estimated wall distance     */
uint8_t  IR_IsWall(uint8_t index);          /* 1 = wall closer than limit  */
uint8_t  IR_IsSaturated(uint8_t index);     /* 1 = reading clipped         */
uint8_t  IR_GetWallMask(void);              /* bit n set = sensor n+1 wall */

/* Test helper: force one emitter on (1) or off (0), e.g. to check with a
 * phone camera that the IR LED lights. Always turn it off again. */
void     IR_SetEmitter(uint8_t index, uint8_t on);

/* Calibration: hold a wall exactly distance_mm from sensor `index`, then call.
 * Measures now and sets K for that sensor. Returns 0 if the signal is too
 * weak or saturated to calibrate. Values are lost at reset: copy the printed
 * K values into ir_cal_k[] in ir.c once they are good. */
uint8_t  IR_Calibrate(uint8_t index, uint16_t distance_mm);
uint32_t IR_GetCalK(uint8_t index);
void     IR_SetCalK(uint8_t index, uint32_t k);
void     IR_SetWallThreshold_mm(uint8_t index, uint16_t distance_mm);
uint16_t IR_GetWallThreshold_mm(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* INC_IR_H_ */
