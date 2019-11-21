/**
 * \file motor_driver.h
 * \author Mav Cuyugan
 *
 * API to drive the burshless motors
 */

#include "motor_driver.h"
#include "hal.h"

motor_driver_handle_t MOTOR_DRIVER;

/**
 * \brief Initialize the motor driver
 * \param[in] handle - Motor driver handle
 */
void motorDriverInit(motor_driver_handle_t* handle)
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(handle->state == MOTOR_DRIVER_UNINIT);

  handle->state = MOTOR_DRIVER_READY;
}