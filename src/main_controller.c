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
  { 0U,    1500U }, /* grounded */

  /* MIN = 15%, MAX = 100% */
  { 1000U, 10000U } /* liftoff */
};

/* TODO: maybe put these in a config file? */
#define EULER_ANGLE_MAX 30.0f
#define EULER_ANGLE_MIN -30.0f
static float PWM_MAX = 10.0f;
static float PWM_MIN = -10.0f;

/**
 * define PID controllers
 */
static pid_ctrl_handle_t roll_pid;
static const pid_cfg_t roll_pid_cfg =
{
  /* PID constants */
  3.0f, 5.5f, 4.0f,

  true, /* clamping enabled */
  EULER_ANGLE_MAX, /* upper saturation point */
  EULER_ANGLE_MIN /* lower saturation point */
};

static pid_ctrl_handle_t pitch_pid;
static const pid_cfg_t pitch_pid_cfg =
{
  /* PID constants */
  3.0f, 5.5f, 4.0f,

  true, /* clamping enabled */
  EULER_ANGLE_MAX, /* upper saturation point */
  EULER_ANGLE_MIN /* lower saturation point */
};

// static pid_ctrl_handle_t yaw_pid;

/**
 * \notapi
 * \brief Get desired euler angle setpoint based on signal from transceiver
 */
static float signal_to_euler_angle(uint32_t signal)
{
  float percent = (float)signal / 10000.0f; // (signal / 100 / 100)

  /* normalize */
  return percent * (EULER_ANGLE_MAX - EULER_ANGLE_MIN) + EULER_ANGLE_MIN;
}

/**
 * \notapi
 * \brief Get desired PWM duty cycle fed to motors from euler angle
 */
static int32_t euler_angle_to_signal(float angle)
{
  /* normalize */
  float percent = (angle - EULER_ANGLE_MIN) / (EULER_ANGLE_MAX - EULER_ANGLE_MIN) * (PWM_MAX - PWM_MIN) + PWM_MIN;

  return (int32_t)(percent * 100.0f);
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
        uint32_t pitch_signal = channels[RADIO_TXRX_PITCH];
        // uint32_t yaw_signal = channels[RADIO_TXRX_YAW];

        float roll_setpoint = signal_to_euler_angle(roll_signal);
        float pitch_setpoint = signal_to_euler_angle(pitch_signal);

        /* read imu data */
        float euler_angles[IMU_DATA_AXES] = {0.0f};
        imuEngineGetData(&IMU_ENGINE, euler_angles, IMU_ENGINE_EULER);

        /* run PID control loop  */
        float roll_correct_angle = pidCompute(&roll_pid, roll_setpoint, euler_angles[IMU_ENGINE_ROLL]);
        float pitch_correct_angle = pidCompute(&pitch_pid, pitch_setpoint, euler_angles[IMU_ENGINE_PITCH]);

        /* convert correction to PWM duty cycles */
        int32_t roll_correct_pwm = euler_angle_to_signal(roll_correct_angle);
        int32_t pitch_correct_pwm = euler_angle_to_signal(pitch_correct_angle);

        /**
         * https://robotics.stackexchange.com/questions/2964/quadcopter-pid-output?lq=1
         */
        for(size_t i = 0 ; i < MOTOR_DRIVER_MOTORS ; i++) {
          uint32_t pwm = 0U;
          uint32_t throttle = channels[RADIO_TXRX_THROTTLE];

          if(i == MOTOR_DRIVER_NW) {
            pwm = throttle + (pitch_correct_pwm / 2) - (roll_correct_pwm / 2);
          } else if(i == MOTOR_DRIVER_NE) {
            pwm = throttle + (pitch_correct_pwm / 2) + (roll_correct_pwm / 2);
          } else if(i == MOTOR_DRIVER_SE) {
            pwm = throttle - (pitch_correct_pwm / 2) + (roll_correct_pwm / 2);
          } else if(i == MOTOR_DRIVER_SW) {
            pwm = throttle - (pitch_correct_pwm / 2) - (roll_correct_pwm / 2);
          }

          duty_cycles[i] = pwm;
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
  pidInit(&roll_pid,  &roll_pid_cfg);
  pidInit(&pitch_pid, &pitch_pid_cfg);
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