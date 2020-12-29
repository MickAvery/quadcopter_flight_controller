/**
 * \file motor_driver.h
 * \author Mav Cuyugan
 *
 * API to drive the burshless motors
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/**
 * Motor driver states
 */
typedef enum
{
  MOTOR_DRIVER_UNINIT = 0,
  MOTOR_DRIVER_STOPPED,
  MOTOR_DRIVER_READY
} motor_driver_state_t;

/**
 * Describes the indeces corresponding to each motor
 *
 *  0  1
 *   \/
 *   /\
 *  3  2
 *
 */
typedef enum
{
  MOTOR_DRIVER_NW = 0, /*!< Northwest motor (CW) */
  MOTOR_DRIVER_NE,     /*!< Northeast motor (CCW) */
  MOTOR_DRIVER_SE,     /*!< Southeast motor (CW) */
  MOTOR_DRIVER_SW,     /*!< Southwest motor (CCW) */
  MOTOR_DRIVER_MOTORS  /*!< Number of motors being driven */
} motor_driver_positions_t;

/**
 * Scaling used when applying PID sum results to motors
 */
typedef struct
{
  float throttle;
  float roll;
  float pitch;
  float yaw;
} motor_scales_t;

/**
 * Motor driver handle
 */
typedef struct
{
  motor_scales_t scales[MOTOR_DRIVER_MOTORS];
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

/**
 * \brief Start the Motor Driver
 * \param[in] handle - Motor driver handle
 */
void motorDriverStart(motor_driver_handle_t* handle);

/**
 * Set the duty cycle for each of the motors
 *
 * Each motor can be driven to either 0% to 100%, take note that:
 * 100%   -> 10000
 * 40%    ->  4000
 * 35.75% ->  3575
 *
 * \param[in] handle - Motor driver handle
 * \pamar[in] duty_cycles - Duty cycles to be used to drive each motor
 */
void motorDriverSetDutyCycles(motor_driver_handle_t* handle, uint32_t duty_cycles[MOTOR_DRIVER_MOTORS]);

#endif /* MOTOR_DRIVER_H */