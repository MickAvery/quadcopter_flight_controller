/**
 * \file main_controller.h
 * \author Mav Cuyugan
 *
 * This is the main controller module.
 * It takes data from the IMU engine and the Radio transceiver
 * and sets drives the motors accordingly.
 */

#include "main_controller.h"
#include "imu_engine.h"
#include "radio_tx_rx.h"
#include "motor_driver.h"

/**
 * Global main controller handle
 */
main_ctrl_handle_t MAIN_CTRL;

/**
 * flight states
 */
typedef enum
{
  GROUNDED = 0, /*!< multirotor is grounded, pull throttle above threshold to commence flight */
  FLYING,      /*!< multirotor is flying, pull throttle down to minimum threshold to ground the multirotor */
  HYSTERESIS_STATES
} hysteresis_states_t;

/**
 * hysteresis range definition
 */
typedef struct
{
  uint32_t min;
  uint32_t max;
} hysteresis_range_t;

/**
 * hysteresis ranges for flight state transitions
 */
static hysteresis_range_t hysteresis_ranges[HYSTERESIS_STATES] =
{
  /* MIN = 0%, MAX = 25% */
  { 0U,    2500U }, /* grounded */

  /* MIN = 15%, MAX = 100% */
  { 1500U, 10000U } /* liftoff */
};

/**
 * Main Controller Thread.
 * It takes data from the IMU engine and Radio Transceiver modules
 * to determine how to drive the motors.
 */
THD_WORKING_AREA(mainControllerThreadWorkingArea, 1024U);
THD_FUNCTION(mainControllerThread, arg)
{
  (void)arg;

  /* keep track of hysteresis state */
  static hysteresis_states_t flight_state = GROUNDED;

  /**
   * wait for radio transceiver to read first frame
   */
  while(radioTxRxGetState(&RADIO_TXRX) != RADIO_TXRX_ACTIVE) {
    chThdSleepMilliseconds(5U);
  }

  /* wait for first frame to be parsed */
  chThdSleepMilliseconds(RADIO_PPM_LENGTH_MS);

  /**
   * Arming sequence
   * make sure throttle is at the bottom position
   * (or at least less than the throttle threshold when multirotor is grounded)
   */

  uint32_t throttle_signal = 0U;

  do {

    uint32_t channels[MOTOR_DRIVER_MOTORS] = {0U};
    radioTxRxReadInputs(&RADIO_TXRX, channels);

    throttle_signal = channels[RADIO_TXRX_THROTTLE];

    chThdSleepMilliseconds(RADIO_PPM_LENGTH_MS);

  } while(throttle_signal >= hysteresis_ranges[flight_state].max);

  /**
   * Main logic
   */

  while(true)
  {
    uint32_t channels[RADIO_TXRX_CHANNELS] = {0U};
    uint32_t duty_cycles[MOTOR_DRIVER_MOTORS] = {0U};

    radioTxRxReadInputs(&RADIO_TXRX, channels);

    /* hysteresis check */
    throttle_signal = channels[RADIO_TXRX_THROTTLE];

    if(throttle_signal > hysteresis_ranges[flight_state].max) {

      /* move up next flight state */
      flight_state++;

    } else if(throttle_signal < hysteresis_ranges[flight_state].min) {

      /* move down previous flight state */
      flight_state--;

    }

    for(size_t i = 0 ; i < MOTOR_DRIVER_MOTORS ; i++) {

      duty_cycles[i] = (flight_state == FLYING) ? channels[RADIO_TXRX_THROTTLE] : 0U;

    }

    motorDriverSetDutyCycles(&MOTOR_DRIVER, duty_cycles);

    chThdSleepMilliseconds(10);
  }
}

/**
 * \brief Initialize the Main Controller module
 * \param[in] handle - Main Controller handle
 */
void mainControllerInit(main_ctrl_handle_t* handle)
{
  osalDbgCheck(handle != NULL);

  handle->state = MAIN_CTRL_STOPPED;
}

/**
 * \brief Start running the Main Controller module
 * \param[in] handle - Main Controller handle
 */
void mainControllerStart(main_ctrl_handle_t* handle)
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(handle->state == MAIN_CTRL_STOPPED);

  handle->state = MAIN_CTRL_RUNNING;

  /* start the main controller thread */
  chThdCreateStatic(
    mainControllerThreadWorkingArea,
    sizeof(mainControllerThreadWorkingArea),
    NORMALPRIO,
    mainControllerThread,
    NULL);
}