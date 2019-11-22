#include "pid.h"

static uint32_t k_p = 1U; /* proportional constant */
static uint32_t k_i = 0U; /* integral constant */
static uint32_t k_d = 0U; /* derivative constant */

static int32_t previous_error = 0; /* previous recorded error */
static int32_t integral_error = 0; /* summation of integral errors */

/**
 * Apply correction to an error value based on desired output
 * and actual output from stability process
 *
 * \param[in] desired_output - the desired output from a process
 * \param[in] actual_output  - the actual output from a process,
 *                             most likely different from desired
 *
 * \return Amount of correction needed to achieve desired output,
 *         based on Proportional (P), Integral (I), and
 *         Derivative (D) terms.
 **/
int32_t pid_algorithm(int32_t desired_output, int32_t actual_output)
{
  int32_t error;
  int32_t prop, integ, deriv;

  error = desired_output - actual_output;
  integral_error += error;

  prop = k_p * ( error );
  integ = k_i * ( integral_error );
  deriv = k_d * ( error - previous_error );

  previous_error = error;

  return ( prop + integ + deriv );
}
