/**
 * \file   utils.c
 * \author Mav Cuyugan (mav.cuyugan@gmail.com)
 * \brief  System-wide utility functions
 */

#include "utils.h"

/**
 * @brief Set constraint on floating-point input, saturate if below or above minimum or maximum respectively
 * 
 * \param in  - Input to constrain
 * \param min - Minimum
 * \param max - Maximum
 * \return float
 */
float constrainf(float in, float min, float max)
{
    if(in < min)
        return min;
    if(in > max)
        return max;
    return in;
}

/**
 * \brief Set constraint on integer input, saturate if below or above minimum or maximum respectively
 * 
 * \param in 
 * \param min 
 * \param max 
 * \return int32_t 
 */
int32_t constrain(int32_t in, int32_t min, int32_t max)
{
    if(in < min)
        return min;
    if(in > max)
        return max;
    return in;
}