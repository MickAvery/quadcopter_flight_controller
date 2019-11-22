/**
 * \file pid.h
 * \author Mav Cuyugan
 *
 * Algorithms for PID calculation
 **/

#include <stdint.h>

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
int32_t pid_algorithm(int32_t desired_output, int32_t actual_output);
