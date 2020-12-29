/**
 * \file main_controller.h
 * \author Mav Cuyugan
 *
 * This is the main controller module.
 * It takes data from the IMU engine and the Radio transceiver
 * and sets drives the motors accordingly.
 */

#include <string.h>
#include <math.h>
#include "main_controller.h"
#include "imu_engine.h"
#include "radio_tx_rx.h"
#include "motor_driver.h"
#include "pid.h"
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
  UNARMED = 0, /*!< multirotor is unarmed, must flight arming switch */
  ARMED,       /*!< multirotor is armed, ready for liftoff */
  FLYING       /*!< multirotor is flying, PID loops running */
} fc_states_t;

/* TODO: maybe put these in a config file? */
#define THROTTLE_MIN  1000 /* 10% */
#define BODY_TILT_MAX 45.0f
#define PID_ITERM_MAX 400.0f

/**
 * define PID controllers
 */
static pid_ctrl_handle_t roll_pid;
static const pid_cfg_t roll_pid_cfg =
{
  /* PID constants */
  3.0f, 5.5f, 4.0f, 0.0f,

  10e-3f,

  PID_ITERM_MAX /* I-term max for saturation */
};

static pid_ctrl_handle_t pitch_pid;
static const pid_cfg_t pitch_pid_cfg =
{
  /* PID constants */
  3.0f, 5.5f, 4.0f, 0.0f,

  10e-3f,

  PID_ITERM_MAX /* I-term max for saturation */
};

static pid_ctrl_handle_t yaw_pid;
static const pid_cfg_t yaw_pid_cfg =
{
  /* PID constants */
  3.0f, 5.5f, 4.0f, 0.0f,

  10e-3f,

  PID_ITERM_MAX /* I-term max for saturation */
};

// TODO: Put into library!
/**
 * @brief Set constraint on floating-point input, saturate if below or above minimum or maximum respectively
 * 
 * @param in  - Input to constrain
 * @param min - Minimum
 * @param max - Maximum
 * @return float
 */
static float constrainf(float in, float min, float max)
{
  if(in < min)
    return min;
  if(in > max)
    return max;
  return in;
}

// TODO: Put into library!
/**
 * @brief Set constraint on integer input, saturate if below or above minimum or maximum respectively
 * 
 * @param in 
 * @param min 
 * @param max 
 * @return int32_t 
 */
static int32_t constrain(int32_t in, int32_t min, int32_t max)
{
  if(in < min)
    return min;
  if(in > max)
    return max;
  return in;
}

#define RC_EXPO             0.0f
#define RC_RATE             0.80f
#define SUPER_RATE          0.65f
#define RC_RATE_INCREMENTAL 14.54f
#define PID_SUM_LIMIT       500.0f
#define power3(x) (x*x*x)

/**
 * @brief 
 * 
 * @param rc_setpoint 
 * @return float 
 */
static float calculate_setpoint_rate(float rc_setpoint)
{
  float rc_sp_abs = fabs(rc_setpoint);

  /* RC Expo */
  rc_setpoint = ( rc_setpoint * power3(rc_sp_abs) * RC_EXPO ) + ( rc_setpoint * (1 - RC_EXPO) );

  /* RC Rates */
  float rc_rate = RC_RATE;
  if (rc_rate > 2.0f)
    rc_rate += RC_RATE_INCREMENTAL * (rc_rate - 2.0f);

  float angle_rate = 200.0f * rc_rate * rc_setpoint;

  /* Super Rates */
  float rc_superfactor = 1.0f / (constrainf(1.0f - (rc_sp_abs * SUPER_RATE), 0.01f, 1.00f));
  angle_rate *= rc_superfactor;

  return angle_rate;
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

  /* keep track of FC state */
  static fc_states_t flight_state = UNARMED;

  /* wait for first frame to be parsed */
  chThdSleepMilliseconds(RADIO_PPM_LENGTH_MS);

  /**
   * Main logic
   */
  while(true)
  {
    uint32_t channels[RADIO_TXRX_CHANNELS] = {0U};
    uint32_t duty_cycles[MOTOR_DRIVER_MOTORS] = {0U};

    radioTxRxReadInputs(&RADIO_TXRX, channels);

    radio_tx_rx_state_t rc_state = radioTxRxGetState(&RADIO_TXRX);
    int32_t arm_switch = channels[RADIO_TXRX_AUXA];
    int32_t throttle_pcnt = channels[RADIO_TXRX_THROTTLE];
    float throttle_pcnt_f = (float)throttle_pcnt / 10000.0f;
    float yaw_rc_sp = RADIO_TXRX.rc_deflections[RADIO_TXRX_YAW];

    /**
     * Flight state machine
     */
    switch(flight_state)
    {
      case UNARMED:
        /* don't drive motors */
        memset(duty_cycles, 0, sizeof(duty_cycles));

        /* Three conditions need to be met to arm the quad:
         *   1. RC must be receiving PPM signals from receiver
         *   2. Flip the arming switch (i.e. signal at arming channel from RC is >50%)
         *   3. Throttle needs to be down
         */
        if( (rc_state == RADIO_TXRX_ACTIVE) && (throttle_pcnt < THROTTLE_MIN) && (arm_switch > 5000) ) /* 50000 == 50% */
        {
          flight_state = ARMED;
          chprintf(
            (BaseSequentialStream*)&SD4,
            "UNARMED -> ARMED\n");
        }

        break;

      case ARMED:
        /* don't drive motors */
        memset(duty_cycles, 0, sizeof(duty_cycles));

        /* reset PID controllers */
        pidReset(&roll_pid);
        pidReset(&pitch_pid);
        pidReset(&yaw_pid);

        /* Switch states when appropriate */
        if(throttle_pcnt > THROTTLE_MIN)
        {
          flight_state = FLYING;
          chprintf(
            (BaseSequentialStream*)&SD4,
            "ARMED -> FLYING\n");
        }
        else if(arm_switch < 5000)
        {
          /* arm switch flipped to unarmed position */
          flight_state = UNARMED;
          chprintf(
            (BaseSequentialStream*)&SD4,
            "ARMED -> UNARMED\n");
        }

        break;

      case FLYING:
      {
        /* read imu data */
        float attitude[IMU_DATA_AXES] = {0.0f};
        float gyro[IMU_DATA_AXES] = {0.0f};
        imuEngineGetData(&IMU_ENGINE, attitude, IMU_ENGINE_EULER);
        imuEngineGetData(&IMU_ENGINE, gyro, IMU_ENGINE_GYRO);

        /* get setpoints from RC input */
        float target_roll_angle = -1.0f * RADIO_TXRX.rc_deflections[RADIO_TXRX_ROLL] * BODY_TILT_MAX;
        float target_pitch_angle = -1.0f * RADIO_TXRX.rc_deflections[RADIO_TXRX_PITCH] * BODY_TILT_MAX;
        float target_yaw_rate = -1.0f * calculate_setpoint_rate(yaw_rc_sp);

        /* get setpoints as degree delta (target_angle - actual_euler_angle) */
        target_roll_angle = target_roll_angle - attitude[RADIO_TXRX_ROLL];
        target_pitch_angle = target_pitch_angle - attitude[RADIO_TXRX_PITCH];

        /* multiple setpoint angles by "Level Strength" */
        target_roll_angle  *= 5.0f; /* TODO: magic numbers */
        target_pitch_angle *= 5.0f; /* TODO: magic numbers */

        /* run iteration of PID loop */
        float roll  = pidCompute(&roll_pid,  target_roll_angle,  gyro[IMU_ENGINE_ROLL] /1000.0f);
        float pitch = pidCompute(&pitch_pid, target_pitch_angle, gyro[IMU_ENGINE_PITCH]/1000.0f);
        float yaw   = pidCompute(&yaw_pid,   target_yaw_rate,    gyro[IMU_ENGINE_YAW]  /1000.0f);

        /* limit the PID sums */
        roll  = constrainf(roll,  -PID_SUM_LIMIT, PID_SUM_LIMIT) / 1000.0f;
        pitch = constrainf(pitch, -PID_SUM_LIMIT, PID_SUM_LIMIT) / 1000.0f;
        yaw   = constrainf(yaw,   -PID_SUM_LIMIT, PID_SUM_LIMIT) / 1000.0f;
        // chprintf(
        //   (BaseSequentialStream*)&SD4,
        //   "throttle = %d\troll = %f\tpitch = %f\tyaw = %f\n", throttle_rc_sp, roll, pitch, yaw);

        /* determine motor duty cycles */
        float motor_cycles[MOTOR_DRIVER_MOTORS];
        float motor_range;
        float motor_max = 0.0f;
        float motor_min = 0.0f;
        for(size_t i = 0 ; i < MOTOR_DRIVER_MOTORS ; i++)
        {
          float motor =
            roll  * MOTOR_DRIVER.scales[i].roll  +
            pitch * MOTOR_DRIVER.scales[i].pitch +
            yaw   * MOTOR_DRIVER.scales[i].yaw;

            if( motor < motor_min )
              motor_min = motor;
            else if( motor > motor_max )
              motor_max = motor;

            motor_cycles[i] = motor;
        }

        motor_range = motor_max - motor_min;

        /* TODO: remove when done
         * applyMixerAdjustment() in betaflight */
        if(motor_range > 1.0f)
        {
          for(size_t i = 0 ; i < MOTOR_DRIVER_MOTORS ; i++)
            motor_cycles[i] /= motor_range;
        }
        else if(throttle_pcnt > 5000) /* throttle over 50% */
          throttle_pcnt_f = constrainf(throttle_pcnt_f, -motor_min, 1.0f - motor_max);

        // chprintf(
        //   (BaseSequentialStream*)&SD4,
        //   "range = %0.2f\tthrottle = %0.2f\t0 = %0.2f\t1 = %0.2f\t2 = %0.2f\t3 = %0.2f\n",
        //   motor_range, throttle_pcnt_f, motor_cycles[0], motor_cycles[1], motor_cycles[2], motor_cycles[3]);

        /* TODO: remove when done
         * applyMixToMotors() in betaflight */
        for(size_t i = 0 ; i < MOTOR_DRIVER_MOTORS ; i++)
        {
          float motor_output_f = motor_cycles[i] + throttle_pcnt_f;

          /* convert motor output to PWM duty cycle */
          int32_t motor_output = (int32_t)(100.0f * motor_output_f);

          /* constrain output to interval [0%, 100%] */
          motor_output = constrain(motor_output, 0, 100);

          /* set duty cycle */
          duty_cycles[i] = (uint32_t)motor_output;

          // chprintf(
          //   (BaseSequentialStream*)&SD4,
          //   "%d = %d\t", i, motor_output);
        }
        // chprintf(
        //   (BaseSequentialStream*)&SD4,
        //   "range = %0.2f\n", motor_range);

        /* Throttle stick low, quad grounded */
        if(throttle_pcnt < THROTTLE_MIN)
        {
          flight_state = ARMED;
          chprintf(
            (BaseSequentialStream*)&SD4,
            "FLYING -> ARMED\n");
        }

        break;
      }

      default:
        break;
    }

    /* drive motors with appropriate duty cycles */
    motorDriverSetDutyCycles(&MOTOR_DRIVER, duty_cycles);

    chThdSleepMilliseconds(1U);
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
  pidInit(&yaw_pid,   &yaw_pid_cfg);

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
