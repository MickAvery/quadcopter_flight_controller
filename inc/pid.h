/**
 * \file   pid.h
 * \author Mav Cuyugan
 *
 * PID Controller API
 **/

#ifndef PID_H
#define PID_H

/**
 * PID controller handle
 */
typedef struct
{
  float k_p; /*!< proportional constant */
  float k_i; /*!< integral constant */
  float k_d; /*!< derivative constant */

  float previous_in;  /*!< keep track of previous input */
  float integral_err; /*!< keep track of error over time */
} pid_ctrl_handle_t;

/**
 * \brief Initialize the PID controller
 * \param[in] pid - PID controller handle
 * \param[in] k_p - Proportional constant
 * \param[in] k_i - Integral constant
 * \param[in] k_d - Derivative constant
 */
void pidInit(pid_ctrl_handle_t* pid, float k_p, float k_i, float k_d);

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