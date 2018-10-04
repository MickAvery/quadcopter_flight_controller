/**
 * \file lsm6dsl.h
 *
 * \author Mav Cuyugan
 * \brief  LSM6DSL driver header
 **/

#ifndef _LSM6DSL_H
#define _LSM6DSL_H

#include "hal.h"

/**
 * \brief Driver states
 */
typedef enum
{
  LSM6DSL_STATE_STOP = 0,
  LSM6DSL_STATE_RUNNING,
  LSM6DSL_STATE_LOWPOWER
} lsm6dsl_state_t;

/**
 * \brief Driver return codes
 */
typedef enum
{
  LSM6DSL_OK = 0, /**< Function call successful */
  LSM6DSL_ERROR = -1, /**< Generic error code */
  LSM6DSL_SERIAL_ERROR = -2, /**< Serial bus issue */
  LSM6DSL_DATA_NOT_AVAILABLE = -3 /**< Sensor readings unavailable */
} lsm6dsl_status_t;

/**
 * \brief Possible sensor sampling rates
 */
typedef enum
{
  LSM6DSL_12_5_Hz = 1,
  LSM6DSL_26_Hz,
  LSM6DSL_52_Hz,
  LSM6DSL_104_Hz,
  LSM6DSL_208_Hz,
  LSM6DSL_416_Hz,
  LSM6DSL_833_Hz,
  LSM6DSL_1_66_KHz,
  LSM6DSL_3_33_KHz,
  LSM6DSL_6_66_KHz,
  LSM6DSL_ODR_MAX
} lsm6dsl_odr_t;

/**
 * \brief Possible accelerometer fullscales
 */
typedef enum
{
  LSM6DSL_ACCEL_2G = 0,
  LSM6DSL_ACCEL_16G,
  LSM6DSL_ACCEL_4G,
  LSM6DSL_ACCEL_8G,
  LSM6DSL_ACCEL_FS_MAX
} lsm6dsl_accel_fullscale_t;

/**
 * \brief Possible gyroscope fullscales
 */
typedef enum
{
  LSM6DSL_GYRO_250DPS = 0,
  LSM6DSL_GYRO_500DPS,
  LSM6DSL_GYRO_1000DPS,
  LSM6DSL_GYRO_2000DPS,
  LSM6DSL_GYRO_FS_MAX
} lsm6dsl_gyro_fullscale_t;

/**
 * \brief Output struct storing sensor readings
 */
typedef struct
{
  float acc_x; /**< Accelerometer X reading in mg */
  float acc_y; /**< Accelerometer Y reading in mg */
  float acc_z; /**< Accelerometer Z reading in mg */

  float gyro_x; /**< Gyroscope X reading in mdps */
  float gyro_y; /**< Gyroscope Y reading in mdps */
  float gyro_z; /**< Gyroscope Z reading in mdps */
} lsm6dsl_sensor_readings_t;

/**
 * \brief Driver configurations
 */
typedef struct
{
  I2CDriver* i2c_drv; /**< Pointer to I2C driver handle */
  lsm6dsl_odr_t odr;  /**< Accelerometer and gyroscope sampling rate */

  lsm6dsl_accel_fullscale_t accel_fs; /**< Accelerometer fullscale */
  lsm6dsl_gyro_fullscale_t gyro_fs; /**< Gyroscope fullscale */
} lsm6dsl_config_t;

/**
 * \brief Driver handle
 */
typedef struct
{
  const lsm6dsl_config_t* cfg; /**< Driver configurations */
  lsm6dsl_state_t state; /**< Driver state */

  float accel_sensitivity; /**< Accelerometer sensitivity */
  float gyro_sensitivity; /**< Gyroscope sensitivity */
} lsm6dsl_handle_t;

extern lsm6dsl_handle_t LSM6DSL_HANDLE;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Start LSM6DSL device
 *
 * \param[out] handle - LSM6DSL handle
 * \param[in]  cfg    - driver configurations
 *
 * \return Driver status
 * \retval LSM6DSL_OK if call successful
 **/
lsm6dsl_status_t lsm6dslStart(lsm6dsl_handle_t* handle, const lsm6dsl_config_t* cfg);

/**
 * \brief Read gyroscope and accelerometer data in mdps and mg respectively
 *
 * \param[in]  handle - driver handle
 * \param[out] vals - output struct to store sensor readings
 *
 * \return Driver status
 * \retval LSM6DSL_OK if call successful
 */
lsm6dsl_status_t lsm6dslRead(lsm6dsl_handle_t* handle, lsm6dsl_sensor_readings_t* vals);

#ifdef __cplusplus
}
#endif

#endif /* _LSM6DSL_H */
