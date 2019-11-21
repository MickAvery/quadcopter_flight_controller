/**
 * \file motor_driver.h
 * \author Mav Cuyugan
 *
 * API to drive the burshless motors
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

typedef enum
{
  MOTOR_DRIVER_UNINIT = 0,
  MOTOR_DRIVER_READY
} motor_driver_state_t;

/**
 * Describes the indeces corresponding to each motor
 */
typedef enum
{
  MOTOR_DRIVER_NW = 0, /*!< Northwest motor */
  MOTOR_DRIVER_NE,     /*!< Northeast motor */
  MOTOR_DRIVER_SW,     /*!< Southwest motor */
  MOTOR_DRIVER_SE,     /*!< Southeast motor */
  MOTOR_DRIVER_MOTORS  /*!< Number of motors being driven */
} motor_driver_positions_t;

typedef struct
{
  motor_driver_state_t state;
} motor_driver_handle_t;

/**
 * Global motor driver handle
 */
extern motor_driver_handle_t MOTOR_DRIVER;

/**
 * \brief Initialize the motor driver
 * \param[in] handle - Motor driver handle
 */
void motorDriverInit(motor_driver_handle_t* handle);

#endif /* MOTOR_DRIVER_H */