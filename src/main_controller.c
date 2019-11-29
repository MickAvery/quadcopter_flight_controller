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
#include "pid.h"

// debug use only TODO: remove when done
#include "chprintf.h"

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
 * define PID controllers
 */
static pid_ctrl_handle_t roll_pid;
static pid_ctrl_handle_t pitch_pid;
// static pid_ctrl_handle_t yaw_pid;

/**
 * \notapi
 * \brief Get desired euler angle setpoint based on signal from transceiver
 */
static float signal_to_euler_angle(uint32_t signal)
{
  float percent = (float)signal / 100.0f / 100.0f;

  /* TODO: magic numbers */
  return percent * (30.0f - (-30.0f)) + (-30.0f);
}

/**
 * \notapi
 * \brief Invert the signal coming from the transceiver
 */
static inline uint32_t signal_invert(uint32_t signal)
{
  return 10000U - signal;
}

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

    throttle_signal = channels[RADIO_TXRX_THROTTLE];

    /**
     * Flight state machine
     */
    switch(flight_state)
    {
      case GROUNDED:
      {

        /* don't drive the motors */
        for(size_t i = 0 ; i < MOTOR_DRIVER_MOTORS ; i++) {
          duty_cycles[i] = 0U;
        }

        /* reset PID controllers */
        pidReset(&roll_pid);
        pidReset(&pitch_pid);
        // pidReset(&yaw_pid);

        /* perform hysteresis */
        if(throttle_signal > hysteresis_ranges[GROUNDED].max) {
          flight_state = FLYING;
        }

        break;
      }

      case FLYING:
      {
        /* determine setpoints */
        uint32_t roll_signal = channels[RADIO_TXRX_ROLL];
        uint32_t pitch_signal = signal_invert(channels[RADIO_TXRX_PITCH]); /* invert just like in fps games */
        // uint32_t yaw_signal = channels[RADIO_TXRX_YAW];

        float roll_setpoint = signal_to_euler_angle(roll_signal);
        float pitch_setpoint = signal_to_euler_angle(pitch_signal);

        /* read imu data */
        float euler_angles[IMU_DATA_AXES] = {0.0f};
        imuEngineGetData(&IMU_ENGINE, euler_angles, IMU_ENGINE_EULER);

        /* run PID control loop  */
        float roll_correction = pidCompute(&roll_pid, roll_setpoint, euler_angles[IMU_ENGINE_ROLL]);
        float pitch_correction = pidCompute(&pitch_pid, pitch_setpoint, euler_angles[IMU_ENGINE_PITCH]);

        /* convert correction to PWM duty cycles */
        (void)roll_correction;
        (void)pitch_correction;

        for(size_t i = 0 ; i < MOTOR_DRIVER_MOTORS ; i++) {
          duty_cycles[i] = channels[RADIO_TXRX_THROTTLE];
        }

        /* perform hysteresis */
        if(throttle_signal < hysteresis_ranges[FLYING].min) {
          flight_state = GROUNDED;
        }

        break;
      }

      default:
        break;
    }

    /* drive motors with appropriate duty cycles */
    motorDriverSetDutyCycles(&MOTOR_DRIVER, duty_cycles);

    chThdSleepMilliseconds(10U);
  }
}

/**
 * \brief Initialize the Main Controller module
 * \param[in] handle - Main Controller handle
 */
void mainControllerInit(main_ctrl_handle_t* handle)
{
  osalDbgCheck(handle != NULL);

  /* initialize our PID controllers */
  pidInit(&roll_pid,  1.0f, 0.1f, 0.05f);
  pidInit(&pitch_pid, 1.0f, 0.1f, 0.05f);
  // pidInit(&yaw_pid,   1.0f, 0.0f, 1.0f);

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