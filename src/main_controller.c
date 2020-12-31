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
#include "fcconf.h"
#include "utils.h"

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
  FLYING,      /*!< multirotor is flying, PID loops running */
  CALIBRATING  /*!< calibrating accelerometer and gyroscope */
} fc_states_t;

/* max allowable body tilt about roll and pitch */
static float body_tilt_max = (float)BODY_TILT_MAX;

/* PID level strength, when using betaflight ANGLE mode */
static float pid_level_strength = (float)PID_ROLL_KP / 10.0f;

/* RC rates */
static float rc_expo    = (float)RC_EXPO / 100.0f;
static float rc_rate    = (float)RC_RATE / 100.0f;
static float super_rate = (float)SUPER_RATE / 100.0f;
// static float rc_rate_incremental = (float)RC_RATE_INCREMENTAL / 100.0f;

/* max PID sums for driving motors */
static float pid_sum_limit = (float)PID_SUM_LIMIT;

/* Controller loop periods based on state */
static uint32_t imu_sampling_period_us = (uint32_t)(1.0f / ((float)IMU_ENGINE_SAMPLING_RATE / (1000.0f*1000.0f)));
static uint32_t pid_period_us = (uint32_t)(1.0f / ((float)PID_FREQUENCY / (1000.0f*1000.0f)));

/* When calibrating, read these many datapoints to compute average */
#define CALIBRATION_NUM_DATAPOINTS 256U

/**
 * Roll PID configs
 */
static pid_ctrl_handle_t roll_pid;
static const pid_cfg_t roll_pid_cfg =
{
  .Kp        = (float)PID_ROLL_KP * PTERM_SCALE,
  .Ki        = (float)PID_ROLL_KI * ITERM_SCALE,
  .Kd        = (float)PID_ROLL_KD * DTERM_SCALE,
  .dT        = 1.0f / (float)PID_FREQUENCY,
  .iterm_max = (float)PID_ROLL_ITERM_MAX /* I-term max for saturation */
};

/**
 * Pitch PID configs
 */
static pid_ctrl_handle_t pitch_pid;
static const pid_cfg_t pitch_pid_cfg =
{
  .Kp        = (float)PID_PITCH_KP * PTERM_SCALE,
  .Ki        = (float)PID_PITCH_KI * ITERM_SCALE,
  .Kd        = (float)PID_PITCH_KD * DTERM_SCALE,
  .dT        = 1.0f / (float)PID_FREQUENCY,
  .iterm_max = (float)PID_PITCH_ITERM_MAX /* I-term max for saturation */
};

/**
 * Yaw PID configs
 */
static pid_ctrl_handle_t yaw_pid;
static const pid_cfg_t yaw_pid_cfg =
{
  .Kp        = (float)PID_YAW_KP * PTERM_SCALE,
  .Ki        = (float)PID_YAW_KI * ITERM_SCALE,
  .Kd        = (float)PID_YAW_KD * DTERM_SCALE,
  .dT        = 1.0f / (float)PID_FREQUENCY,
  .iterm_max = (float)PID_YAW_ITERM_MAX /* I-term max for saturation */
};

/**
 * \notapi
 * \brief Calculate setpoint angular velocity rate based on stick position
 * \param rc_setpoint - Desired setpoint based on stick position
 * \return float - Setpoint rate in degs/sec
 */
static float calculate_setpoint_rate(float rc_setpoint)
{
  float rc_sp_abs = fabs(rc_setpoint);

  /* RC Expo */
  rc_setpoint = ( rc_setpoint * power3(rc_sp_abs) * rc_expo ) + ( rc_setpoint * (1 - rc_expo) );

  /* RC Rates */
  float rc_rate_local = rc_rate;
  // if (rc_rate_local > 2.0f)
  //   rc_rate_local += rc_rate_incremental * (rc_rate_local - 2.0f);

  float angle_rate = 200.0f * rc_rate_local * rc_setpoint;

  /* Super Rates */
  float rc_superfactor = 1.0f / (constrainf(1.0f - (rc_sp_abs * super_rate), 0.01f, 1.00f));
  angle_rate *= rc_superfactor;

  return angle_rate;
}

/**
 * \notapi
 * \brief Detect rising edge of calibration switch signal
 * \param calib_switch Position of calibration switch
 */
static bool calibration_request_detected(uint32_t calib_switch)
{
  static uint32_t calib_switch_prev = 0U;
  bool ret = false;

  /* if current is above 50% while previous is below 50% */
  if( (calib_switch > 5000) && (calib_switch_prev < 5000) )
    ret = true;

  calib_switch_prev = calib_switch;

  return ret;
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
  fc_states_t flight_state = UNARMED;

  /* if calibrating, keep track of datapoints read */
  size_t calib_numpoints_read = 0;

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
    uint32_t calib_switch = channels[RADIO_TXRX_AUXB];
    int32_t throttle_pcnt = channels[RADIO_TXRX_THROTTLE];
    float throttle_pcnt_f = (float)throttle_pcnt / 10000.0f;
    float yaw_rc_sp = RADIO_TXRX.rc_deflections[RADIO_TXRX_YAW];
    float gyro[IMU_DATA_AXES] = {0.0f};

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

        /*
         * Calibration request tied to SWB.
         * Request granted only once after switch is flipped on (i.e. on "rising edge" of SWB channel).
         * For repeat calibration requests, flip SWB switch off and on again
         */
        if(calibration_request_detected(calib_switch))
        {
          flight_state = CALIBRATING;
          chprintf(
            (BaseSequentialStream*)&SD4,
            "UNARMED -> CALIBRATING\n");
        }

        break;

      case CALIBRATING:
      {
        static float accel_zero_g_avg[IMU_DATA_AXES]   = {0.0f};
        static float gyro_zero_rate_avg[IMU_DATA_AXES] = {0.0f};

        /* don't drive motors */
        memset(duty_cycles, 0, sizeof(duty_cycles));

        if(calib_numpoints_read < CALIBRATION_NUM_DATAPOINTS)
        {
          float accel[IMU_DATA_AXES] = {0.0f};

          imuEngineGetData(&IMU_ENGINE, accel, IMU_ENGINE_ACCEL);
          imuEngineGetData(&IMU_ENGINE, gyro, IMU_ENGINE_GYRO);

          for(size_t i = 0 ; i < IMU_DATA_AXES ; i++)
          {
            accel_zero_g_avg[i]   += accel[i];
            gyro_zero_rate_avg[i] += gyro[i];
          }

          calib_numpoints_read++;
        }
        else
        {
          /* calculate averages, subtract setpoint to get offsets */
          for(size_t i = 0 ; i < IMU_DATA_AXES ; i++)
          {
            accel_zero_g_avg[i]   /= (float)CALIBRATION_NUM_DATAPOINTS;
            gyro_zero_rate_avg[i] /= (float)CALIBRATION_NUM_DATAPOINTS;

            if(i == IMU_ENGINE_YAW)
              accel_zero_g_avg[IMU_ENGINE_YAW] -= 1000.0f; /* at zero-g, YAW is at 1g */
          }

          /* set calibration offsets */
          imuEngineZeroRateCalibrate(&IMU_ENGINE, gyro_zero_rate_avg);
          imuEngineZeroGCalibrate(&IMU_ENGINE, accel_zero_g_avg);

          /* reset counter */
          calib_numpoints_read = 0;

          flight_state = UNARMED;
          chprintf(
            (BaseSequentialStream*)&SD4,
            "CALIBRATING -> UNARMED\n");
        }
        break;
      }

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
        float attitude[IMU_DATA_AXES] = {0.0f};

        /* read imu data */
        imuEngineGetData(&IMU_ENGINE, attitude, IMU_ENGINE_EULER);
        imuEngineGetData(&IMU_ENGINE, gyro, IMU_ENGINE_GYRO);

        /* get setpoints from RC input */
        float target_roll_angle = -1.0f * RADIO_TXRX.rc_deflections[RADIO_TXRX_ROLL] * body_tilt_max;
        float target_pitch_angle = -1.0f * RADIO_TXRX.rc_deflections[RADIO_TXRX_PITCH] * body_tilt_max;
        float target_yaw_rate = -1.0f * calculate_setpoint_rate(yaw_rc_sp);

        /* get setpoints as degree delta (target_angle - actual_euler_angle) */
        target_roll_angle = target_roll_angle - attitude[RADIO_TXRX_ROLL];
        target_pitch_angle = target_pitch_angle - attitude[RADIO_TXRX_PITCH];

        /* multiple setpoint angles by "Level Strength" */
        target_roll_angle  *= pid_level_strength;
        target_pitch_angle *= pid_level_strength;

        /* run iteration of PID loop */
        float roll  = pidCompute(&roll_pid,  target_roll_angle,  gyro[IMU_ENGINE_ROLL]  / 1000.0f);
        float pitch = pidCompute(&pitch_pid, target_pitch_angle, gyro[IMU_ENGINE_PITCH] / 1000.0f);
        float yaw   = pidCompute(&yaw_pid,   target_yaw_rate,    gyro[IMU_ENGINE_YAW]   / 1000.0f);

        /* limit the PID sums */
        roll  = constrainf(roll,  -pid_sum_limit, pid_sum_limit) / 1000.0f;
        pitch = constrainf(pitch, -pid_sum_limit, pid_sum_limit) / 1000.0f;
        yaw   = constrainf(yaw,   -pid_sum_limit, pid_sum_limit) / 1000.0f;
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

        /* applyMixerAdjustment() in betaflight */
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

        /* applyMixToMotors() in betaflight */
        for(size_t i = 0 ; i < MOTOR_DRIVER_MOTORS ; i++)
        {
          float motor_output_f = motor_cycles[i] + throttle_pcnt_f;

          /* convert motor output to PWM duty cycle */
          int32_t motor_output = (int32_t)(100.0f * motor_output_f) * 100;

          /* constrain output to interval [0%, 100%] */
          motor_output = constrain(motor_output, 0, 10000);

          /* set duty cycle */
          duty_cycles[i] = (uint32_t)motor_output;

          // chprintf(
            // (BaseSequentialStream*)&SD4,
            // "%d = %3d\t", i, motor_output/100);
        }
        // chprintf(
          // (BaseSequentialStream*)&SD4,
          // "range = %0.2f\n", motor_range);

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

    if(flight_state == CALIBRATING)
      chThdSleepMicroseconds(imu_sampling_period_us); /* when calibrating, match speed of IMU engine */
    else
      chThdSleepMicroseconds(pid_period_us);
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
