/**
  ******************************************************************************
  * @file           : ir.c
  * @brief          : IR wall-sensor driver: pulsed, ambient-cancelled readings,
  *                   distance estimate and wall detection. See ir.h.
  ******************************************************************************
  */

#include "ir.h"

/* ---------------------------------------------------------------- hardware */
static ADC_HandleTypeDef *ir_adc = NULL;

static const uint32_t ir_channel[IR_COUNT] = {
    ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13 /* PC0..PC3 */
};
static const uint16_t ir_emitter_pin[IR_COUNT] = {
    GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5                 /* PE2..PE5 */
};
#define IR_EMITTER_PORT GPIOE

/* ------------------------------------------------------------- calibration */
/* K = signal * distance_mm^2 for each sensor. Replace with the values that
 * the IRCAL command prints once each sensor is calibrated. */
static uint32_t ir_cal_k[IR_COUNT] = {
    IR_DEFAULT_K, IR_DEFAULT_K, IR_DEFAULT_K, IR_DEFAULT_K
};
static uint16_t ir_wall_mm[IR_COUNT] = {
    IR_DEFAULT_WALL_MM, IR_DEFAULT_WALL_MM, IR_DEFAULT_WALL_MM, IR_DEFAULT_WALL_MM
};

/* ----------------------------------------------------------------- results */
static uint16_t ir_signal[IR_COUNT];
static uint16_t ir_ambient[IR_COUNT];
static uint16_t ir_distance[IR_COUNT];
static uint8_t  ir_saturated[IR_COUNT];

/* ================================================================ helpers */
/* Microsecond delay using the Cortex-M7 cycle counter (DWT->CYCCNT). */
static void IR_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;               /* unlock DWT on Cortex-M7 */
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void IR_DelayUs(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < cycles) { }
}

/* Point ADC3 at one receiver channel. */
static uint8_t IR_SelectChannel(uint8_t index)
{
    ADC_ChannelConfTypeDef config = {0};
    config.Channel      = ir_channel[index];
    config.Rank         = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    return (HAL_ADC_ConfigChannel(ir_adc, &config) == HAL_OK) ? 1U : 0U;
}

/* Average IR_SAMPLES conversions of the currently selected channel. */
static uint16_t IR_ReadAverage(void)
{
    uint32_t sum = 0U;
    for (uint8_t n = 0U; n < IR_SAMPLES; n++)
    {
        if (HAL_ADC_Start(ir_adc) != HAL_OK) return 0U;
        if (HAL_ADC_PollForConversion(ir_adc, 2U) != HAL_OK)
        {
            HAL_ADC_Stop(ir_adc);
            return 0U;
        }
        sum += HAL_ADC_GetValue(ir_adc);
        HAL_ADC_Stop(ir_adc);
    }
    return (uint16_t)(sum / IR_SAMPLES);
}

/* Integer square root (no floating point or math library needed). */
static uint32_t IR_Isqrt(uint32_t value)
{
    uint32_t result = 0U;
    uint32_t bit = 1UL << 30;
    while (bit > value) bit >>= 2;
    while (bit != 0U)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

static uint16_t IR_SignalToDistance(uint8_t index, uint16_t signal)
{
    if (signal < IR_SIGNAL_MIN) return IR_DISTANCE_MAX_MM;
    uint32_t mm = IR_Isqrt(ir_cal_k[index] / signal);
    return (mm > IR_DISTANCE_MAX_MM) ? IR_DISTANCE_MAX_MM : (uint16_t)mm;
}

/* Measure one sensor: ambient, LED on, lit, LED off. */
static void IR_MeasureOne(uint8_t index)
{
    if (!IR_SelectChannel(index))
    {
        ir_signal[index] = 0U;
        ir_ambient[index] = 0U;
        ir_saturated[index] = 0U;
        ir_distance[index] = IR_DISTANCE_MAX_MM;
        return;
    }

    uint16_t ambient = IR_ReadAverage();

    HAL_GPIO_WritePin(IR_EMITTER_PORT, ir_emitter_pin[index], GPIO_PIN_SET);
    IR_DelayUs(IR_SETTLE_US);
    uint16_t lit = IR_ReadAverage();
    HAL_GPIO_WritePin(IR_EMITTER_PORT, ir_emitter_pin[index], GPIO_PIN_RESET);

    ir_ambient[index]   = ambient;
    ir_saturated[index] = (lit >= IR_SATURATION) ? 1U : 0U;
    ir_signal[index]    = (lit > ambient) ? (uint16_t)(lit - ambient) : 0U;
    ir_distance[index]  = IR_SignalToDistance(index, ir_signal[index]);
}

/* ===================================================================== API */
void IR_Init(ADC_HandleTypeDef *hadc)
{
    ir_adc = hadc;
    IR_DelayInit();

    for (uint8_t i = 0U; i < IR_COUNT; i++)
    {
        HAL_GPIO_WritePin(IR_EMITTER_PORT, ir_emitter_pin[i], GPIO_PIN_RESET);
        ir_signal[i] = 0U;
        ir_ambient[i] = 0U;
        ir_saturated[i] = 0U;
        ir_distance[i] = IR_DISTANCE_MAX_MM;
    }
}

void IR_Update(void)
{
    if (ir_adc == NULL) return;
    for (uint8_t i = 0U; i < IR_COUNT; i++)
    {
        IR_MeasureOne(i);
        /* Let the LED's light fully die away before the next sensor reads. */
        IR_DelayUs(20U);
    }
}

void IR_SetEmitter(uint8_t index, uint8_t on)
{
    if (index >= IR_COUNT) return;
    HAL_GPIO_WritePin(IR_EMITTER_PORT, ir_emitter_pin[index],
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint16_t IR_GetSignal(uint8_t index)
{
    return (index < IR_COUNT) ? ir_signal[index] : 0U;
}

uint16_t IR_GetAmbient(uint8_t index)
{
    return (index < IR_COUNT) ? ir_ambient[index] : 0U;
}

uint16_t IR_GetDistance_mm(uint8_t index)
{
    return (index < IR_COUNT) ? ir_distance[index] : IR_DISTANCE_MAX_MM;
}

uint8_t IR_IsSaturated(uint8_t index)
{
    return (index < IR_COUNT) ? ir_saturated[index] : 0U;
}

uint8_t IR_IsWall(uint8_t index)
{
    if (index >= IR_COUNT) return 0U;
    if (ir_saturated[index]) return 1U;              /* clipped = very close */
    if (ir_signal[index] < IR_SIGNAL_MIN) return 0U; /* nothing seen          */
    return (ir_distance[index] <= ir_wall_mm[index]) ? 1U : 0U;
}

uint8_t IR_GetWallMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < IR_COUNT; i++)
    {
        if (IR_IsWall(i)) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

uint8_t IR_Calibrate(uint8_t index, uint16_t distance_mm)
{
    if (index >= IR_COUNT || distance_mm == 0U || ir_adc == NULL) return 0U;

    /* Average several full measurements for a steady calibration point. */
    uint32_t sum = 0U;
    uint8_t saturated = 0U;
    for (uint8_t n = 0U; n < 8U; n++)
    {
        IR_MeasureOne(index);
        sum += ir_signal[index];
        saturated |= ir_saturated[index];
        HAL_Delay(2U);
    }
    uint32_t signal = sum / 8U;
    if (saturated || signal < IR_SIGNAL_MIN) return 0U;

    ir_cal_k[index] = signal * (uint32_t)distance_mm * (uint32_t)distance_mm;
    ir_distance[index] = IR_SignalToDistance(index, (uint16_t)signal);
    return 1U;
}

uint32_t IR_GetCalK(uint8_t index)
{
    return (index < IR_COUNT) ? ir_cal_k[index] : 0U;
}

void IR_SetCalK(uint8_t index, uint32_t k)
{
    if (index < IR_COUNT && k > 0U) ir_cal_k[index] = k;
}

void IR_SetWallThreshold_mm(uint8_t index, uint16_t distance_mm)
{
    if (index < IR_COUNT && distance_mm > 0U) ir_wall_mm[index] = distance_mm;
}

uint16_t IR_GetWallThreshold_mm(uint8_t index)
{
    return (index < IR_COUNT) ? ir_wall_mm[index] : 0U;
}
