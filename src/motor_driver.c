/**
 * \file motor_driver.h
 * \author Mav Cuyugan
 *
 * API to drive the burshless motors
 */

#include "motor_driver.h"
#include "pinconf.h"

/**
 * Global motor driver handle
 */
motor_driver_handle_t MOTOR_DRIVER;

/**
 * PWM Timer Clock Frequency (100kHz)
 * corresponds to each tick being 0.01ms
 */
#define PWM_CLK_FREQ (100U * 1000U)

/**
 * Convert pulse width from ms to clock ticks
 */
#define MS_TO_PWM_TICKS(ms) (ms * PWM_CLK_FREQ / 1000U)

/* Max and Min PWM pulse widths in clock ticks */
static pwmcnt_t MIN_PWM_PULSE_WIDTH = MS_TO_PWM_TICKS(1);
static pwmcnt_t MAX_PWM_PULSE_WIDTH = MS_TO_PWM_TICKS(2);

/**
 * \notapi
 * \brief Convert duty cycle percentage from input into appropriate pulse width in ticks
 */
static inline pwmcnt_t percent_to_ticks(uint32_t duty_cycle)
{
  return (duty_cycle*(MAX_PWM_PULSE_WIDTH - MIN_PWM_PULSE_WIDTH)/10000U) + MIN_PWM_PULSE_WIDTH;
}

static PWMConfig pwmcfg =
{
  PWM_CLK_FREQ,
  250U, /* period in ticks TODO: magic number */
  NULL, /* period callback, not needed */

  /* channels */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },

  0U, /* cr2 = 0 */
  0U  /* dier = 0 */
};

/**
 * \brief Initialize the Motor Driver
 * \param[in] handle - Motor driver handle
 */
void motorDriverInit(motor_driver_handle_t* handle)
{
  osalDbgCheck(handle != NULL);

  handle->state = MOTOR_DRIVER_STOPPED;
}

/**
 * \brief Start the Motor Driver
 * \param[in] handle - Motor driver handle
 */
void motorDriverStart(motor_driver_handle_t* handle)
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(handle->state == MOTOR_DRIVER_STOPPED);

  /* configure pins */
  palSetPadMode(PWM_NW_PORT, PWM_NW_PADNUM, PAL_MODE_ALTERNATE(PWM_NW_ALTMODE));
  palSetPadMode(PWM_NE_PORT, PWM_NE_PADNUM, PAL_MODE_ALTERNATE(PWM_NE_ALTMODE));
  palSetPadMode(PWM_SW_PORT, PWM_SW_PADNUM, PAL_MODE_ALTERNATE(PWM_SW_ALTMODE));
  palSetPadMode(PWM_SE_PORT, PWM_SE_PADNUM, PAL_MODE_ALTERNATE(PWM_SE_ALTMODE));

  /* initialize PWM driver */
  pwmStart(&PWMD2, &pwmcfg);

  handle->state = MOTOR_DRIVER_READY;
}

/**
 * Set the duty cycle for each of the motors
 *
 * Each motor can be driven to either 0% to 100%, take note that:
 * 100%   -> 10000
 * 40%    ->  4000
 * 35.75% ->  3575
 *
 * \param[in] handle - Motor driver handle
 * \pamar[in] duty_cycles - Duty cycles to be used to drive each motor
 */
void motorDriverSetDutyCycles(motor_driver_handle_t* handle, uint32_t duty_cycles[MOTOR_DRIVER_MOTORS])
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(handle->state == MOTOR_DRIVER_READY);
  osalDbgCheck(duty_cycles != NULL);

  /* convert duty cycles to their appropriate pulse width in ticks */
  for(size_t i = 0U ; i < MOTOR_DRIVER_MOTORS ; i++) {
    pwmcnt_t pulse_width = percent_to_ticks(duty_cycles[i]);
    pwmEnableChannel(&PWMD2, i, pulse_width);
  }
}