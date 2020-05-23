/**
 * \file   imu_engine.h
 * \author Mav Cuyugan
 *
 * Header file for IMU Engine.
 *
 * This module is responsible for acquiring data from the IMU sensors (Accelerometer, Gyroscope, Magnetometer).
 * It then calculates the Euler angles from the sensor readings
 */

#include <math.h>

#include "imu_engine.h"
#include "lsm6dsl.h"
#include "iis2mdc.h"

/* let's define PI */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * Global IMU Engine handler
 */
imu_engine_handle_t IMU_ENGINE;

/**
 * Sensor handles and configs
 */

static lsm6dsl_handle_t lsm6dsl;
static iis2mdc_handle_t iis2mdc;

static const lsm6dsl_config_t lsm6dsl_cfg =
{
  &I2CD2,
  LSM6DSL_104_Hz,
  LSM6DSL_ACCEL_2G,
  LSM6DSL_GYRO_250DPS
};

static const iis2mdc_config_t iis2mdc_cfg =
{
  &I2CD2,
  IIS2MDC_ODR_100_Hz
};

/**
 * \notapi
 * \brief Get the state of the IMU handle
 */
static bool imuIsRunning(imu_engine_handle_t* handle)
{
  bool ret;

  /* acquire mutex */
  osalMutexLock(&handle->lock);

  ret = (handle->state == IMU_ENGINE_RUNNING);

  /* release mutex */
  osalMutexUnlock(&handle->lock);

  return ret;
}

/**
 * Main IMU Engine Thread.
 * It's purpose is to just periodically read sensor data and compute Euler angles,
 * so as to always have valid sensor data.
 */
THD_WORKING_AREA(imuEngineThreadWorkingArea, 1024U);
THD_FUNCTION(imuEngineThread, arg)
{
  imu_engine_handle_t* handle = (imu_engine_handle_t*)arg;

  while(imuIsRunning(handle)) {

    lsm6dsl_sensor_readings_t readings;
    iis2mdc_sensor_readings_t mag_readings;

    if(lsm6dslRead(&lsm6dsl, &readings) != LSM6DSL_OK) {
      /* failed to read accel and gyro data */
    } else if(iis2mdcRead(&iis2mdc, &mag_readings) != IIS2MDC_STATUS_OK) {
      /* failed to read mag data */
    } else {

      /* euler angle measurements */
      /* https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data */
      /* https://sites.google.com/site/myimuestimationexperience/sensors/magnetometer */
      static bool first_reading = true;

      osalMutexLock(&handle->lock);

      /* TODO: poor design choice with the return types of the sensors API,
               should've just passed in float arrays instead of the structs to avoid shit like this... */
      handle->accel_data[IMU_DATA_X] = readings.acc_x;
      handle->accel_data[IMU_DATA_Y] = readings.acc_y;
      handle->accel_data[IMU_DATA_Z] = readings.acc_z;

      handle->gyro_data[IMU_DATA_X] = readings.gyro_x;
      handle->gyro_data[IMU_DATA_Y] = readings.gyro_y;
      handle->gyro_data[IMU_DATA_Z] = readings.gyro_z;

      handle->mag_data[IMU_DATA_X] = mag_readings.mag_x;
      handle->mag_data[IMU_DATA_Y] = mag_readings.mag_y;
      handle->mag_data[IMU_DATA_Z] = mag_readings.mag_z;

      if(first_reading) {
        handle->euler_angles[IMU_ENGINE_ROLL] = atan2f(readings.acc_x, readings.acc_z) * 180.0f / M_PI; /* roll */
        handle->euler_angles[IMU_ENGINE_PITCH] = atan2f(readings.acc_y, readings.acc_z) * 180.0f / M_PI; /* pitch */

        first_reading = false;

      } else {
        float roll_gyro = handle->euler_angles[IMU_ENGINE_ROLL] + readings.gyro_y * 0.00001f; /* roll (gyro / 1000 * 0.01 -> dt)*/
        float pitch_gyro = handle->euler_angles[IMU_ENGINE_PITCH] + readings.gyro_x * 0.00001f; /* pitch (gyro / 1000 * 0.01 -> dt) */

        float roll_acc = atan2f(readings.acc_x, readings.acc_z) * 180.0f / M_PI;
        float pitch_acc = atan2f(readings.acc_y, readings.acc_z) * 180.0f / M_PI;

        /* apply complementary filter */
        /* TODO: magic numbers */
        handle->euler_angles[IMU_ENGINE_ROLL] = (roll_gyro * 0.55f) + (roll_acc * 0.45f);
        handle->euler_angles[IMU_ENGINE_PITCH] = (pitch_gyro * 0.98f) + (pitch_acc * 0.02f);

        /* convert angles to radians for yaw calculation */
        /* TODO: there's gotta be a better way to do this... */
        float roll  = handle->euler_angles[IMU_ENGINE_ROLL] * M_PI / 180.0f;
        float pitch = handle->euler_angles[IMU_ENGINE_PITCH] * M_PI / 180.0f;
        float mag_x = mag_readings.mag_x;
        float mag_y = mag_readings.mag_y;
        float mag_z = mag_readings.mag_z;

        /* finally, compute yaw */
        /* https://community.st.com/s/question/0D50X00009XkX3s/lsm303agr-conversion-accelerometers-and-magnetometers-output-to-pitch-roll-and-yaw */

        // float By2 = mag_z * sinf(roll) - mag_y * cosf(roll);
        // float Bz2 = mag_y * sinf(roll) + mag_z * cosf(roll);
        // float Bx3 = mag_x * cosf(pitch) + Bz2 * sinf(pitch);
        // handle->euler_angles[IMU_ENGINE_YAW] = atan2f(By2, Bx3);// * 180.0f / M_PI;

        float y = (-mag_y * cosf(roll)) + (mag_z * sinf(roll));
        float x = (mag_x * cosf(pitch)) + (mag_y * sinf(pitch) * sinf(roll)) + (mag_z * sinf(pitch) * cosf(roll));
        handle->euler_angles[IMU_ENGINE_YAW] = atan2f(y, x) * 180.0f / M_PI;
      }

      osalMutexUnlock(&handle->lock);

      /* TODO: this might need to be changed... */
      chThdSleepMilliseconds(10);

    }
  }
}

/**
 * \brief Initialize the IMU Engine
 * \param[in] handle - pointer to engine handle
 */
void imuEngineInit(imu_engine_handle_t* handle)
{
  lsm6dslObjectInit(&lsm6dsl);
  iis2mdcObjectInit(&iis2mdc);

  if(lsm6dslStart(&lsm6dsl, &lsm6dsl_cfg) != LSM6DSL_OK) {
    /* failed to start driver */
    chSysHalt("Failed to start IMUs");
  } else if(iis2mdcStart(&iis2mdc, &iis2mdc_cfg) != IIS2MDC_STATUS_OK) {
    /* failed to start driver */
    chSysHalt("Failed to start IMUs");
  } else {

    osalMutexObjectInit(&handle->lock);
    handle->state = IMU_ENGINE_RUNNABLE;

  }
}

/**
 * \brief Start the IMU Engine after calling imuEngineInit()
 * \param[in] handle - point to engine handle
 */
void imuEngineStart(imu_engine_handle_t* handle)
{
  osalDbgAssert(handle->state == IMU_ENGINE_RUNNABLE, "IMU Engine Invalid State");

  handle->state = IMU_ENGINE_RUNNING;

  /* start the IMU Engine Thread */
  chThdCreateStatic(
    imuEngineThreadWorkingArea,
    sizeof(imuEngineThreadWorkingArea),
    NORMALPRIO,
    imuEngineThread,
    handle);
}

/**
 * \brief Get data from any of the sources from IMU engine
 * \param[in] handle - IMU Engine handle
 * \param[out] data  - buffer to store data
 * \param[in] source - what kind of data to get (either accel, gyro, mag, or euler angles)
 */
void imuEngineGetData(imu_engine_handle_t* handle, float data[IMU_DATA_AXES], imu_engine_data_t type)
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(data != NULL);
  osalDbgCheck(handle->state == IMU_ENGINE_RUNNING);

  osalMutexLock(&handle->lock);

  volatile float* src = NULL;
  float* dest = data;

  switch(type)
  {
    case IMU_ENGINE_ACCEL:
      src = handle->accel_data;
      break;
    case IMU_ENGINE_GYRO:
      src = handle->gyro_data;
      break;
    case IMU_ENGINE_MAG:
      src = handle->mag_data;
      break;
    case IMU_ENGINE_EULER:
      src = handle->euler_angles;
      break;
    default:
      break;
  }

  if(src != NULL) {
    for(size_t i = 0U ; i < IMU_DATA_AXES ; i++) {
      dest[i] = src[i];
    }
  }

  osalMutexUnlock(&handle->lock);
}

/**
 * \brief Calibrate the magnetometer
 * \param[in] handle - IMU Engine handle
 * \param[in] offsets - offsets to apply to each axis
 * \return IMU_ENGINE_OK if successful
 */
imu_engine_status_t imuEngineMagCalibrate(imu_engine_handle_t* handle, float offsets[IMU_DATA_AXES])
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(handle->state == IMU_ENGINE_RUNNING);
  osalDbgCheck(offsets != NULL);

  imu_engine_status_t ret = IMU_ENGINE_ERROR;

  if(iis2mdcCalibrate(&iis2mdc, offsets[0], offsets[1], offsets[2]) != IIS2MDC_STATUS_OK) {
    /* failed to calibrate mag */
  } else {
    ret = IMU_ENGINE_OK;
  }

  return ret;
}