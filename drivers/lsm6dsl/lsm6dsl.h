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
  LSM6DSL_STATE_LOWPOWER,
  LSM6DSL_STATE_PASSTHROUGH
} lsm6dsl_state_t;

/**
 * \brief Driver return codes
 */
typedef enum
{
  LSM6DSL_OK = 0,                 /**< Function call successful */
  LSM6DSL_ERROR = -1,             /**< Generic error code */
  LSM6DSL_SERIAL_ERROR = -2,      /**< Serial bus issue */
  LSM6DSL_DATA_NOT_AVAILABLE = -3 /**< Sensor readings unavailable */
} lsm6dsl_status_t;

/**
 * \brief Accelerometer sampling rates
 */
typedef enum
{
  LSM6DSL_ACCEL_12_5_Hz = 1, /**< 12.5 Hz */
  LSM6DSL_ACCEL_26_Hz,       /**< 26   Hz */
  LSM6DSL_ACCEL_52_Hz,       /**< 52   Hz */
  LSM6DSL_ACCEL_104_Hz,      /**< 104  Hz */
  LSM6DSL_ACCEL_208_Hz,      /**< 208  Hz */
  LSM6DSL_ACCEL_416_Hz,      /**< 416  Hz */
  LSM6DSL_ACCEL_833_Hz,      /**< 833  Hz */
  LSM6DSL_ACCEL_1_66_KHz,    /**< 1.66 KHz */
  LSM6DSL_ACCEL_3_33_KHz,    /**< 3.33 KHz */
  LSM6DSL_ACCEL_6_66_KHz,    /**< 6.66 KHz */
  LSM6DSL_ACCEL_ODR_MAX
} lsm6dsl_accel_odr_t;

/**
 * \brief Gyroscope sampling rates
 */
typedef enum
{
  LSM6DSL_GYRO_12_5_Hz = 1, /**< 12.5 Hz */
  LSM6DSL_GYRO_26_Hz,       /**< 26   Hz */
  LSM6DSL_GYRO_52_Hz,       /**< 52   Hz */
  LSM6DSL_GYRO_104_Hz,      /**< 104  Hz */
  LSM6DSL_GYRO_208_Hz,      /**< 208  Hz */
  LSM6DSL_GYRO_416_Hz,      /**< 416  Hz */
  LSM6DSL_GYRO_833_Hz,      /**< 833  Hz */
  LSM6DSL_GYRO_1_66_KHz,    /**< 1.66 KHz */
  LSM6DSL_GYRO_3_33_KHz,    /**< 3.33 KHz */
  LSM6DSL_GYRO_6_66_KHz,    /**< 6.66 KHz */
  LSM6DSL_GYRO_ODR_MAX
} lsm6dsl_gyro_odr_t;

/**
 * \brief Gyroscope LPF bandwidth, each selection (A,B,C,D) depends on ODR,
 *        see datasheet for details
 *        (look for FTYPE[1:0] in CTRL6_C register)
 */
typedef enum
{
  LSM6DSL_GYRO_LPF_BW_A, /**< [ODR=800Hz -> 245Hz] ; [ODR=1.6kHz -> 315Hz] ; [ODR=3.3kHz -> 343Hz] ; [ODR=6.6kHz -> 351Hz] */
  LSM6DSL_GYRO_LPF_BW_B, /**< [ODR=800Hz -> 195Hz] ; [ODR=1.6kHz -> 224Hz] ; [ODR=3.3kHz -> 234Hz] ; [ODR=6.6kHz -> 237Hz] */
  LSM6DSL_GYRO_LPF_BW_C, /**< [ODR=800Hz -> 155Hz] ; [ODR=1.6kHz -> 168Hz] ; [ODR=3.3kHz -> 172Hz] ; [ODR=6.6kHz -> 173Hz] */
  LSM6DSL_GYRO_LPF_BW_D  /**< [ODR=800Hz -> 293Hz] ; [ODR=1.6kHz -> 505Hz] ; [ODR=3.3kHz -> 925Hz] ; [ODR=6.6kHz -> 937Hz] */
} lsm6dsl_gyro_lpf_bw_t;

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
  I2CDriver* i2c_drv;                 /**< Pointer to I2C driver handle */
  lsm6dsl_accel_odr_t accel_odr;      /**< Accelerometer and gyroscope sampling rate */
  lsm6dsl_gyro_odr_t  gyro_odr;       /**< Accelerometer and gyroscope sampling rate */

  lsm6dsl_accel_fullscale_t accel_fs; /**< Accelerometer fullscale */
  lsm6dsl_gyro_fullscale_t gyro_fs;   /**< Gyroscope fullscale */

  bool gyro_lpf_en; /**< If set, low-pass filter is applied to gyroscope fullscale */
  lsm6dsl_gyro_lpf_bw_t gyro_lpf_bw;
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

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initialize driver handle
 *
 * \param[out] handle - LSM6DSL handle
 */
void lsm6dslObjectInit(lsm6dsl_handle_t* handle);

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

/**
 * \brief Set accelerometer offsets to trim linear velocity readings
 * 
 * \param[in] handle - driver handle
 * \param[in] offsets - offsets to save to sensor offset registers
 *
 * \return Driver status
 * \retval LSM6DSL_OK if call successful
 */
lsm6dsl_status_t lsm6dslSetAccelOffset(lsm6dsl_handle_t* handle, int8_t offsets[3U]);

/**
 * \brief Enable I2C passthrough to allow host MCU to communicate with external magnetometer
 *
 * \param[in] handle - LSM6DSL handle
 *
 * \return Driver status
 * \retval LSM6DSL_OK if call successful
 */
lsm6dsl_status_t lsm6dslPassthroughEnable(lsm6dsl_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif /* _LSM6DSL_H */
