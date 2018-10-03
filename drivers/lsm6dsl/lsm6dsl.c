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

static systime_t timeout = MS2ST(500U);

static i2caddr_t ctrl1_xl_addr = 0x0010U;
static i2caddr_t ctrl2_g_addr  = 0x0011U;

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

  osalDbgAssert((handle->state == LSM6DSL_STATE_STOP) || (handle->state == LSM6DSL_STATE_LOWPOWER),
    "lsm6dslStart() called at invalid state");

  uint8_t ctrl1_xl = 0U;
  uint8_t ctrl2_g = 0U;
  handle->cfg = cfg;
  I2CDriver* i2c = handle->cfg->i2c_drv;
  lsm6dsl_status_t ret = LSM6DSL_SERIAL_ERROR;

  i2cAcquireBus(i2c);

  if(i2cMasterReceiveTimeout(i2c, ctrl1_xl_addr, &ctrl1_xl, 1U, timeout) != MSG_OK) {
    /* I2C read failed */
  } else if(i2cMasterReceiveTimeout(i2c, ctrl2_g_addr, &ctrl2_g, 1U, timeout) != MSG_OK) {
    /* I2C read failed */
  } else {
    ctrl1_xl &= ~(0xF << 4);
    ctrl1_xl |= (handle->cfg->odr << 4);
    ctrl1_xl |= (handle->cfg->accel_fs << 2);

    ctrl2_g &= ~(0xF << 4);
    ctrl2_g |= (handle->cfg->odr << 4);
    ctrl2_g |= (handle->cfg->gyro_fs << 2);

    if(i2cMasterTransmitTimeout(i2c, ctrl1_xl_addr, &ctrl1_xl, 1, NULL, 0, timeout) != MSG_OK) {
      /* I2C write failed */
    } else if(i2cMasterTransmitTimeout(i2c, ctrl2_g_addr, &ctrl2_g, 1, NULL, 0, timeout) != MSG_OK) {
      /* I2C write failed */
    } else {
      handle->state = LSM6DSL_STATE_RUNNING;
      ret = LSM6DSL_OK;
    }
  }

  i2cReleaseBus(i2c);

  return ret;
}


/**
 *
 */
lsm6dsl_status_t lsm6dslRead(lsm6dsl_handle_t* handle, lsm6dsl_sensor_readings_t* vals)
{
  osalDbgCheck((handle != NULL) && (vals != NULL));

  osalDbgAssert(handle->state == LSM6DSL_STATE_RUNNING, "lsm6dslRead called at invalid state");

  lsm6dsl_status_t ret = LSM6DSL_SERIAL_ERROR;

  return ret;
}