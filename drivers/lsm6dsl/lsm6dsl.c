/**
 * \file lsm6dsl.c
 *
 * \author Mav Cuyugan
 * \brief  LSM6DSL driver source
 **/

#include <stddef.h>
#include <stdint.h>
#include "osal.h"
#include "lsm6dsl.h"

lsm6dsl_handle_t LSM6DSL_HANDLE =
{
  NULL,
  LSM6DSL_STATE_STOP
};

/**
 * \brief Start LSM6DSL device
 *
 * \param[out] handle - LSM6DSL handle
 * \param[in]  cfg    - driver configurations
 *
 * \return Driver status
 **/
lsm6dsl_status_t lsm6dslStart(lsm6dsl_handle_t* handle, const lsm6dsl_config_t* cfg)
{
  osalDbgCheck((handle != NULL) && (cfg != NULL));
  osalDbgCheck(cfg->i2c_drv != NULL);

  handle->cfg = cfg;

  lsm6dsl_status_t ret = LSM6DSL_ERROR;

  // i2cAcquireBus(handle->cfg->i2c_drv);
  // i2cReleaseBus(handle->cfg->i2c_drv);

  return ret;
}
