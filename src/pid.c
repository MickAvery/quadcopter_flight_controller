/**
 * \file   pid.c
 * \author Mav Cuyugan
 *
 * PID Controller API
 */

#include "pid.h"
#include "hal.h"

/**
 * \brief Initialize the PID controller
 * \param[in] pid - PID controller handle
 * \param[in] k_p - Proportional constant
 * \param[in] k_i - Integral constant
 * \param[in] k_d - Derivative constant
 */
void pidInit(pid_ctrl_handle_t* pid, float k_p, float k_i, float k_d)
{
  osalDbgCheck(pid != NULL);

  pid->k_p = k_p;
  pid->k_i = k_i;
  pid->k_d = k_d;

  pid->previous_in = 0.0f;
  pid->integral_err = 0.0f;
}

/**
 * Apply correction to an error value based on desired output
 * and actual output from stability process
 *
 * \param[in] pid      - PID controller handle
 * \param[in] setpoint - the desired output from a process
 * \param[in] input    - the value value from a process,
 *                       most likely different from desired
 *
 * \return Amount of correction needed to achieve desired output,
 *         based on Proportional (P), Integral (I), and
 *         Derivative (D) terms.
 **/
float pidCompute(pid_ctrl_handle_t* pid, float setpoint, float input)
{
  osalDbgCheck(pid != NULL);

  float proportional, integral, derivative;

  /* get error */
  float error = setpoint - input;

  /* get derivative of input */
  float input_deriv = input - pid->previous_in;
  pid->previous_in = input;

  /* update integral error */
  pid->integral_err += error;
  integral = pid->k_i * pid->integral_err;

  /**
   * design choice : proportional on measurement
   * http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
   */
  proportional = pid->k_p * input_deriv;

  /**
   * design choice : derivative on measurement
   * https://controlguru.com/pid-control-and-derivative-on-measurement/
   * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
   */
  derivative = pid->k_d * input_deriv;

  return ( -proportional + integral - derivative );
}

/**
 * \brief Reset the PID controller
 * \param[in] pid - PID controller handle
 */
void pidReset(pid_ctrl_handle_t* pid)
{
  osalDbgCheck(pid != NULL);

  pid->previous_in = 0.0f;
  pid->integral_err = 0.0f;
}
