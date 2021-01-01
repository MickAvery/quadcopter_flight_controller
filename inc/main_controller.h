/**
 * \file main_controller.h
 * \author Mav Cuyugan
 *
 * This is the main controller module.
 * It takes data from the IMU engine and the Radio transceiver
 * and sets drives the motors accordingly.
 */

#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Main controller states
 */
typedef enum
{
  MAIN_CTRL_UNINIT = 0,
  MAIN_CTRL_STOPPED,
  MAIN_CTRL_RUNNING
} main_ctrl_state_t;

/**
 * Main controller handle definition
 */
typedef struct
{
  main_ctrl_state_t state;
} main_ctrl_handle_t;

/**
 * Global main controller handle
 */
extern main_ctrl_handle_t MAIN_CTRL;

/**
 * \brief Initialize the Main Controller module
 * \param[in] handle - Main Controller handle
 */
void mainControllerInit(main_ctrl_handle_t* handle);

/**
 * \brief Start running the Main Controller module
 * \param[in] handle - Main Controller handle
 */
void mainControllerStart(main_ctrl_handle_t* handle);

/**
 * If enable is true, request to short-circuit throttle input and motor output and bypass main controller.
 * If false then disable throttle bypassing.
 * \param[in] handle - Main Controller handle
 * \param[in] enable - If true then enable bypassing, if false then disable bypassing
 * \param[in] motor  - Motor number to bypass to. If equal to number of motors, bypass to all motors
 */
void mainControllerThrottleBypass(main_ctrl_handle_t* handle, bool enable, uint32_t motor);

#endif /* MAIN_CONTROLLER_H */