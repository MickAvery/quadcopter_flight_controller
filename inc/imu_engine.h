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

#define ACCEL_OFF_WEIGHT 0.9765625f /*!< Accel offset weight in mg/LSB */

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
  volatile imu_engine_state_t state;          /**< IMU engine state */
  volatile float accel_data[IMU_DATA_AXES];   /**< linear acceleration from accelerometer */
  volatile float gyro_data[IMU_DATA_AXES];    /**< angular velocity from gyroscope */
  volatile float mag_data[IMU_DATA_AXES];     /**< magnetometer data */
  volatile float euler_angles[IMU_DATA_AXES]; /**< quadcopter attitude */
  volatile float gyro_offset[IMU_DATA_AXES];  /**< angular velocity offset at zero-rate */

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

/**
 * \brief Set angular rate offsets at zero-rate, these will be subtracted from angular rates read from sensor
 * \param[in] handle - IMU Engine handle
 * \param[in] ang_rate_offsets - offsets to apply to each axis
 */
void imuEngineZeroRateCalibrate(imu_engine_handle_t* handle, float ang_rate_offsets[IMU_DATA_AXES]);

/**
 * \brief Set linear velocity offsets at zero-rate
 * \note These offsets will be stored directly to the sensors for on-chip offsetting 
 * \param[in] handle - IMU Engine handle
 * \param[in] lin_velocity_offsets - offsets to apply to each axis
 */
void imuEngineZeroGCalibrate(imu_engine_handle_t* handle, float lin_velocity_offsets[IMU_DATA_AXES]);

#endif /* IMU_ENGINE_H */