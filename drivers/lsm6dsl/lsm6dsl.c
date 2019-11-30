/**
 * \file lsm6dsl.c
 *
 * \author Mav Cuyugan
 * \brief  LSM6DSL driver source
 **/

#include <stddef.h>
#include <stdint.h>
#include "ch.h"
#include "osal.h"
#include "lsm6dsl.h"

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
static sysinterval_t timeout = TIME_MS2I(10U);

/**
 * \brief LSM6DSL slave address when SDA is grounded
 */
static uint8_t lsm6dsl_addr = 0b01101010;

/*****************************************
 * Register addresses
 *****************************************/

static uint8_t ctrl1_xl_addr      = 0x10U;
static uint8_t ctrl2_g_addr       = 0x11U;
// static uint8_t ctrl10_c_addr      = 0x19U;
static uint8_t status_addr        = 0x1EU;
static uint8_t data_start_addr    = 0x22U;
static uint8_t master_config_addr = 0x1AU;

/*****************************************
 * API
 *****************************************/

/**
 * \brief Initialize driver handle
 *
 * \param[out] handle - LSM6DSL handle
 */
void lsm6dslObjectInit(lsm6dsl_handle_t* handle)
{
  chDbgCheck(handle!= NULL);

  handle->state = LSM6DSL_STATE_STOP;
  handle->cfg = NULL;
}

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

  if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, &ctrl1_xl_addr, 1U, &ctrl1_xl, 1U, timeout) != MSG_OK) {
    /* I2C read failed */
  } else if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, &ctrl2_g_addr, 1U, &ctrl2_g, 1U, timeout) != MSG_OK) {
    /* I2C read failed */
  } else {

    ctrl1_xl &= ~(0xF << 4);
    ctrl1_xl |= (handle->cfg->odr << 4);
    ctrl1_xl |= (handle->cfg->accel_fs << 2);

    ctrl2_g &= ~(0xF << 4);
    ctrl2_g |= (handle->cfg->odr << 4);
    ctrl2_g |= (handle->cfg->gyro_fs << 2);

    uint8_t xl_rx[2] = {ctrl1_xl_addr, ctrl1_xl};
    uint8_t g_rx[2] = {ctrl2_g_addr, ctrl2_g};

    if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, xl_rx, 2, NULL, 0, timeout) != MSG_OK) {
      /* I2C write failed */
    } else if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, g_rx, 2, NULL, 0, timeout) != MSG_OK) {
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

  if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, &status_addr, 1U, &status, 1U, timeout) != MSG_OK) {
    /* I2C read failed */
  } else if((status & (accel_ready_flag | gyro_ready_flag)) <= 0) {
    ret = LSM6DSL_DATA_NOT_AVAILABLE;
  } else if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, &data_start_addr, 1, (uint8_t*)rawbytes, numbytes, timeout) != MSG_OK) {
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

/**
 * \brief Enable I2C passthrough to allow host MCU to communicate with external magnetometer
 *
 * \param[in] handle - LSM6DSL handle
 *
 * \return Driver status
 * \retval LSM6DSL_OK if call successful
 */
lsm6dsl_status_t lsm6dslPassthroughEnable(lsm6dsl_handle_t* handle)
{
  chDbgCheck(handle != NULL);

  chDbgAssert(handle->state != LSM6DSL_STATE_PASSTHROUGH, "lsm6dslPassthroughEnable called in invalid state");

  lsm6dsl_status_t ret = LSM6DSL_SERIAL_ERROR;

  I2CDriver* i2c = handle->cfg->i2c_drv;

  uint8_t rx = 0U;

  i2cAcquireBus(i2c);

  if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, &master_config_addr, 1, &rx, 1, timeout) == MSG_OK) {
    rx |= (1 << 4);

    uint8_t tx[2] = {master_config_addr, rx};

    if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, tx, 2, NULL, 0, timeout) == MSG_OK) {
      /* 5ms delay stated in datasheet */
      chThdSleepMilliseconds(5U);

      tx[1] &= ~((1) | (1 << 3) | (1 << 4)); /* reset MASTER_ON, PULL_UP_EN, START_CONFIG */
      tx[1] |= (1 << 2); /* set PASS_THROUGH_MODE */

      if(i2cMasterTransmitTimeout(i2c, lsm6dsl_addr, tx, 2, NULL, 0, timeout) == MSG_OK) {
        handle->state = LSM6DSL_STATE_PASSTHROUGH;
        ret = LSM6DSL_OK;
      }
    }
  }

  i2cReleaseBus(i2c);

  return ret;
}