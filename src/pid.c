/**
 * \file   pid.c
 * \author Mav Cuyugan
 *
 * PID Controller API
 */

#include <math.h>
#include <stdbool.h>
#include "pid.h"
#include "hal.h"

/**
 * \brief Initialize the PID controller
 * \param[in] pid - PID controller handle
 * \param[in] cfg - PID configurations
 */
void pidInit(pid_ctrl_handle_t* pid, const pid_cfg_t* cfg)
{
  osalDbgCheck(pid != NULL);
  osalDbgCheck(cfg != NULL);

  pid->cfg = cfg;

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
  float ret;
  float clamped_ret = 0.0f;

  /* get error */
  float error = setpoint - input;

  /* get derivative of input */
  float input_deriv = input - pid->previous_in;
  pid->previous_in = input;

  /* update integral error */
  if(pid->cfg->clamping_enable && pid->saturated) {
    pid->integral_err += 0.0f;
  } else {
    pid->integral_err += error;
  }

  integral = pid->cfg->k_i * pid->integral_err;

  /**
   * design choice : proportional on measurement
   * http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
   */
  proportional = pid->cfg->k_p * input_deriv;

  /**
   * design choice : derivative on measurement
   * https://controlguru.com/pid-control-and-derivative-on-measurement/
   * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
   */
  derivative = pid->cfg->k_d * input_deriv;

  ret = ( -proportional + integral - derivative );

  /* if clamping enabled, enforce saturation limit */
  if(pid->cfg->clamping_enable) {
    if(ret > pid->cfg->saturation_point_max)
    {
      clamped_ret = pid->cfg->saturation_point_max;
    }
    else if(ret < pid->cfg->saturation_point_min)
    {
      clamped_ret = pid->cfg->saturation_point_min;
    }
    else
    {
      clamped_ret = ret;
    }

    /* check #1  */
    bool clamped = (clamped_ret != ret);

    /* check # 2 */
    bool same_sign = (error >= 0.0f) ^ (ret < 0.0f);

    pid->saturated = (clamped && same_sign) ? true : false;
  }

  return clamped_ret;
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
