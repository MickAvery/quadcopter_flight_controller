/**
 * \file lsm6dsl.h
 *
 * \author Mav Cuyugan
 * \brief  LSM6DSL driver header
 **/

#ifndef _LSM6DSL_H
#define _LSM6DSL_H

#include "hal.h"

typedef enum
{
  LSM6DSL_STATE_STOP = 0,
  LSM6DSL_STATE_RUNNING,
  LSM6DSL_STATE_LOWPOWER
} lsm6dsl_state_t;

typedef enum
{
  LSM6DSL_OK = 0,
  LSM6DSL_ERROR = -1,
  LSM6DSL_SERIAL_ERROR = -2
} lsm6dsl_status_t;

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

typedef enum
{
  LSM6DSL_ACCEL_2G = 0,
  LSM6DSL_ACCEL_16G,
  LSM6DSL_ACCEL_4G,
  LSM6DSL_ACCEL_8G,
  LSM6DSL_ACCEL_FS_MAX
} lsm6dsl_accel_fullscale_t;

typedef enum
{
  LSM6DSL_GYRO_250DPS = 0,
  LSM6DSL_GYRO_500DPS,
  LSM6DSL_GYRO_1000DPS,
  LSM6DSL_GYRO_2000DPS,
  LSM6DSL_GYRO_FS_MAX
} lsm6dsl_gyro_fullscale_t;

/**
 *
 */
typedef struct
{
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} lsm6dsl_sensor_readings_t;

/**
 *
 */
typedef struct
{
  I2CDriver* i2c_drv; /**< Pointer to I2C driver handle */
  lsm6dsl_odr_t odr;  /**< Accelerometer and gyroscope sampling rate */

  lsm6dsl_accel_fullscale_t accel_fs;
  lsm6dsl_gyro_fullscale_t gyro_fs;
} lsm6dsl_config_t;

/**
 *
 */
typedef struct
{
  const lsm6dsl_config_t* cfg;
  lsm6dsl_state_t state;
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
 **/
lsm6dsl_status_t lsm6dslStart(lsm6dsl_handle_t* handle, const lsm6dsl_config_t* cfg);

#ifdef __cplusplus
}
#endif

#endif /* _LSM6DSL_H */
