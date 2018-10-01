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
  LSM6DSL_STATE_START,
  LSM6DSL_STATE_LOWPOWER
} lsm6dsl_state_t;

typedef enum
{
  LSM6DSL_OK = 0,
  LSM6DSL_ERROR = -1,
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

typedef struct
{
  I2CDriver* i2c_drv;
} lsm6dsl_config_t;

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
