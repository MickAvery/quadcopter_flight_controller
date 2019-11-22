/**
 * \file   imu_engine.h
 * \author Mav Cuyugan
 *
 * Header file for IMU Engine.
 *
 * This module is responsible for acquiring data from the IMU sensors (Accelerometer, Gyroscope, Magnetometer).
 * It then calculates the Euler angles from the sensor readings
 */

#ifndef IMU_ENGINE_H
#define IMU_ENGINE_H

#include "hal.h"

typedef enum
{
  IMU_DATA_X = 0,
  IMU_DATA_Y,
  IMU_DATA_Z,
  IMU_DATA_AXES
} imu_engine_axis_t;

typedef enum
{
  IMU_ENGINE_ROLL = 0,
  IMU_ENGINE_PITCH,
  IMU_ENGINE_YAW
} imu_engine_euler_index_t;

/**
 * \brief Use these symbols when
 */
typedef enum
{
  IMU_ENGINE_ACCEL = 0,
  IMU_ENGINE_GYRO,
  IMU_ENGINE_MAG,
  IMU_ENGINE_EULER
} imu_engine_data_t;

typedef enum
{
  IMU_ENGINE_UNINIT = 0,
  IMU_ENGINE_RUNNABLE,
  IMU_ENGINE_RUNNING,
} imu_engine_state_t;

typedef enum
{
  IMU_ENGINE_OK = 0,
  IMU_ENGINE_ERROR
} imu_engine_status_t;

typedef struct
{
  volatile imu_engine_state_t state;
  volatile float accel_data[3U];
  volatile float gyro_data[3U];
  volatile float mag_data[3U];
  volatile float euler_angles[3U];

  mutex_t lock;
} imu_engine_handle_t;

/**
 * Global IMU Engine handler
 */
extern imu_engine_handle_t IMU_ENGINE;

/**
 * \brief Initialize the IMU Engine
 * \param[in] handle - pointer to engine handle
 */
void imuEngineInit(imu_engine_handle_t* handle);

/**
 * \brief Start the IMU Engine after calling imuEngineInit()
 * \param[in] handle - point to engine handle
 * \return IMU Engine status
 */
void imuEngineStart(imu_engine_handle_t* handle);

/**
 * \brief Get data from any of the sources from IMU engine
 * \param[in] handle - IMU Engine handle
 * \param[out] data  - buffer to store data
 * \param[in] source - what kind of data to get (either accel, gyro, mag, or euler angles)
 */
void imuEngineGetData(imu_engine_handle_t* handle, float data[IMU_DATA_AXES], imu_engine_data_t type);

/**
 * \brief Calibrate the magnetometer
 * \param[in] handle - IMU Engine handle
 * \param[in] offsets - offsets to apply to each axis
 * \return IMU_ENGINE_OK if successful
 */
imu_engine_status_t imuEngineMagCalibrate(imu_engine_handle_t* handle, float offsets[IMU_DATA_AXES]);

#endif /* IMU_ENGINE_H */