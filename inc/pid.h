/**
 * \file   pid.h
 * \author Mav Cuyugan
 *
 * PID Controller API
 **/

#ifndef PID_H
#define PID_H

/**
 * PID configurations
 */
typedef struct
{
  float Kp; /*!<  */
  float Ki; /*!<  */
  float Kd; /*!<  */
  float Kf; /*!<  */

  float dT; /*!< PID looptime */

  float iterm_max; /*!< upper saturation limit */
} pid_cfg_t;

/**
 * PID controller handle
 */
typedef struct
{
  float P;   /*!<  */
  float I;   /*!<  */
  float D;   /*!<  */
  float F;   /*!<  */
  float sum; /*!< P + I + D + F */

  float previous_sp;  /*!< keep track of previous setpoint */
  float previous_in;  /*!< keep track of previous input */
  float integral_err; /*!< keep track of error over time */

  const pid_cfg_t* cfg; /*!< PID configurations */
} pid_ctrl_handle_t;

/**
 * \brief Initialize the PID controller
 * \param[in] pid - PID controller handle
 * \param[in] cfg - PID configurations
 */
void pidInit(pid_ctrl_handle_t* pid, const pid_cfg_t* cfg);

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
float pidCompute(pid_ctrl_handle_t* pid, float setpoint, float input);

/**
 * \brief Reset the PID controller
 * \param[in] pid - PID controller handle
 */
void pidReset(pid_ctrl_handle_t* pid);

#endif /* PID_H */