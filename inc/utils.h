/**
 * \file   utils.h
 * \author Mav Cuyugan (mav.cuyugan@gmail.com)
 * \brief  System-wide utility functions
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

/**
 * @brief x^3
 */
#define power3(x) (x*x*x)

/**
 * @brief Set constraint on floating-point input, saturate if below or above minimum or maximum respectively
 * 
 * \param in  - Input to constrain
 * \param min - Minimum
 * \param max - Maximum
 * \return float
 */
float constrainf(float in, float min, float max);

/**
 * \brief Set constraint on integer input, saturate if below or above minimum or maximum respectively
 * 
 * \param in 
 * \param min 
 * \param max 
 * \return int32_t 
 */
int32_t constrain(int32_t in, int32_t min, int32_t max);

#endif /* UTILS_H */