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
#include "utils.h"

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

  pid->P = 0.0f;
  pid->I = 0.0f;
  pid->D = 0.0f;
  pid->F = 0.0f;
  pid->sum = 0.0f;
  pid->previous_sp = 0.0f;
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

  float error = setpoint - input;

  /*******************************
   * P
   *******************************/
  pid->P = pid->cfg->Kp * error;

  /*******************************
   * I
   *******************************/
  pid->I = pid->I + (pid->cfg->Ki * pid->cfg->dT * error);
  pid->I = constrainf(pid->I, -pid->cfg->iterm_max, pid->cfg->iterm_max); /* saturate, integral windup avoidance */

  /*******************************
   * D
   *******************************/
  float pid_frequency = 1.0f / pid->cfg->dT;
  float delta = -(input - pid->previous_in) * pid_frequency;
  pid->D = delta * pid->cfg->Kd; /* TODO might need throttle PID attenuation here */

  pid->previous_in = input;

  // TODO:not implemented yet
  /*******************************
   * F
   *******************************/
  // float setpoint_delta = setpoint - pid->previous_sp;
  // pid->previous_sp = setpoint;
  pid->F = 0.0f;

  /*******************************
   * Sum
   *******************************/
  pid->sum = pid->P + pid->I + pid->D + pid->F;

  return pid->sum;
}

/**
 * \brief Reset the PID controller
 * \param[in] pid - PID controller handle
 */
void pidReset(pid_ctrl_handle_t* pid)
{
  osalDbgCheck(pid != NULL);

  pid->P = 0.0f;
  pid->I = 0.0f;
  pid->D = 0.0f;
  pid->F = 0.0f;
  pid->sum = 0.0f;
  pid->previous_sp = 0.0f;
  pid->previous_in = 0.0f;
  pid->integral_err = 0.0f;
}
