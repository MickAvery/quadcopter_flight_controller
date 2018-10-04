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
 * \brief Accelerometer sensitivities corresponding to
 *        configured fullscale
 */
static float accel_sensitivities_list[LSM6DSL_ACCEL_FS_MAX] =
{
  0.061f, 0.488f, 0.122f, 0.244f
};

/**
 * \brief Gyroscope sensitivities corresponding to
 *        configured fullscale
 */
static float gyro_sensitivities_list[LSM6DSL_GYRO_FS_MAX] =
{
  8.75f, 17.50f, 35.0f, 70.0f
};

/**
 * \brief Global timeout for i2c calls
 */
static systime_t timeout = MS2ST(500U);

/*****************************************
 * Register addresses
 *****************************************/

static i2caddr_t ctrl1_xl_addr   = 0x0010U;
static i2caddr_t ctrl2_g_addr    = 0x0011U;
static i2caddr_t status_addr     = 0x001EU;
static i2caddr_t data_start_addr = 0x0022U;

/*****************************************
 * API
 *****************************************/

/**
 * \brief Start LSM6DSL device
 *
 * \param[out] handle - LSM6DSL handle
 * \param[in]  cfg    - driver configurations
 *
 * \return Driver status
 * \retval LSM6DSL_OK if call successful
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
      handle->accel_sensitivity = accel_sensitivities_list[handle->cfg->accel_fs];
      handle->gyro_sensitivity = gyro_sensitivities_list[handle->cfg->gyro_fs];
      handle->state = LSM6DSL_STATE_RUNNING;
      ret = LSM6DSL_OK;
    }
  }

  i2cReleaseBus(i2c);

  return ret;
}


/**
 * \brief Read gyroscope and accelerometer data in mdps and mg respectively
 *
 * \param[in]  handle - driver handle
 * \param[out] vals - output struct to store sensor readings
 *
 * \return Driver status
 * \retval LSM6DSL_OK if call successful
 */
lsm6dsl_status_t lsm6dslRead(lsm6dsl_handle_t* handle, lsm6dsl_sensor_readings_t* vals)
{
  osalDbgCheck((handle != NULL) && (vals != NULL));

  osalDbgAssert(handle->state == LSM6DSL_STATE_RUNNING, "lsm6dslRead called at invalid state");

  I2CDriver* i2c = handle->cfg->i2c_drv;
  uint8_t accel_ready_flag = 0x01U;
  uint8_t gyro_ready_flag  = 0x02U;
  uint8_t status = 0U;
  int16_t rawbytes[6] = {0U};
  size_t numbytes = sizeof(rawbytes);

  lsm6dsl_status_t ret = LSM6DSL_SERIAL_ERROR;

  i2cAcquireBus(i2c);

  if(i2cMasterReceiveTimeout(i2c, status_addr, &status, 1U, timeout) != MSG_OK) {
    /* I2C read failed */
  } else if((status & (accel_ready_flag | gyro_ready_flag)) <= 0) {
    ret = LSM6DSL_DATA_NOT_AVAILABLE;
  } else if(i2cMasterReceiveTimeout(i2c, data_start_addr, (uint8_t*)rawbytes, numbytes, timeout) != MSG_OK) {
    /* failed to read raw data bytes */
  } else {
    /* process data */

    vals->gyro_x = rawbytes[0] * handle->gyro_sensitivity;
    vals->gyro_y = rawbytes[1] * handle->gyro_sensitivity;
    vals->gyro_z = rawbytes[2] * handle->gyro_sensitivity;

    vals->acc_x = rawbytes[3] * handle->accel_sensitivity;
    vals->acc_y = rawbytes[4] * handle->accel_sensitivity;
    vals->acc_z = rawbytes[5] * handle->accel_sensitivity;

    ret = LSM6DSL_OK;
  }

  i2cReleaseBus(i2c);

  return ret;
}