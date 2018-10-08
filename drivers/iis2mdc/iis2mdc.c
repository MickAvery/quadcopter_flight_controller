/**
 * \file   iis2mdc.c
 * \author Mav Cuyugan
 * \brief  IIS2MDC API implementation
 */

#include <stddef.h>
#include <string.h>
#include "osal.h"
#include "iis2mdc.h"

static i2caddr_t iis2mdc_addr = 0b00011110;

static systime_t timeout = MS2ST(10U);

static uint8_t offset_x_reg_l_addr = 0x45U;
static uint8_t status_addr         = 0x67U;
static uint8_t outx_l_addr         = 0x68U;
static uint8_t cfg_reg_a_addr      = 0x60U;

static uint8_t zxyda_flag = (1 << 3);

static float sensitivity = 1.5f;

/**
 * \brief Initialize IIS2MDC driver handle object
 *
 * \param[out] handle - driver handle to initialize
 */
void iis2mdcObjectInit(iis2mdc_handle_t* handle)
{
  chDbgCheck(handle != NULL);

  handle->state = IIS2MDC_STOP;
  handle->cfg = NULL;
}

/**
 * \brief Configure and start IIS2MDC magnetometer
 *
 * \param[in] handle - driver handle
 * \param[in] cfg - configurations
 *
 * \return Driver status
 * \retval IIS2MDC_STATUS_OK if call successful
 */
iis2mdc_status_t iis2mdcStart(iis2mdc_handle_t* handle, const iis2mdc_config_t* cfg)
{
  chDbgCheck((handle != NULL) && (cfg != NULL));

  chDbgAssert(handle->state != IIS2MDC_RUNNING, "iis2mdcStart called from invalid state");

  iis2mdc_status_t ret = IIS2MDC_STATUS_SERIAL_ERROR;

  handle->cfg = cfg;

  uint8_t cfg_reg_a = 0U;

  i2cAcquireBus(cfg->i2c);

  if(i2cMasterTransmitTimeout(cfg->i2c, iis2mdc_addr, &cfg_reg_a_addr, 1, &cfg_reg_a, 1, timeout) == MSG_OK) {

    cfg_reg_a &= ~(0x3U); /* reset bits[1:0] for continuous mode */
    cfg_reg_a |= (cfg->odr << 2);

    uint8_t tx[] = {cfg_reg_a_addr, cfg_reg_a};

    if(i2cMasterTransmitTimeout(cfg->i2c, iis2mdc_addr, tx, 2, NULL, 0, timeout) == MSG_OK) {
      handle->state = IIS2MDC_RUNNING;
      ret = IIS2MDC_STATUS_OK;
    }
  }

  i2cReleaseBus(cfg->i2c);

  return ret;
}

/**
 * \brief Read magnetometer data in mgauss
 *
 * \param[in]  handle - driver handle
 * \param[out] vals   - output struct to store sensor readings
 *
 * \return Driver status
 * \retval IIS2MDC_STATUS_OK if call successful
 */
iis2mdc_status_t iis2mdcRead(iis2mdc_handle_t* handle, iis2mdc_sensor_readings_t* vals)
{
  chDbgCheck((handle != NULL) && (vals != NULL));

  chDbgAssert(handle->state == IIS2MDC_RUNNING, "iis2mdcRead called from invalid state");

  iis2mdc_status_t ret = IIS2MDC_STATUS_SERIAL_ERROR;

  I2CDriver* i2c = handle->cfg->i2c;

  uint8_t status = 0U;

  i2cAcquireBus(i2c);

  if(i2cMasterTransmitTimeout(i2c, iis2mdc_addr, &status_addr, 1, &status, 1, timeout) != MSG_OK) {

  } else if((status & zxyda_flag) > 0) {
    int16_t raw[3] = {0};

    if(i2cMasterTransmitTimeout(i2c, iis2mdc_addr, &outx_l_addr, 1, (uint8_t*)raw, sizeof(raw), timeout) == MSG_OK) {
      vals->mag_x = raw[0] * sensitivity;
      vals->mag_y = raw[1] * sensitivity;
      vals->mag_z = raw[2] * sensitivity;

      ret = IIS2MDC_STATUS_OK;
    }
  }

  i2cReleaseBus(i2c);

  return ret;
}

/**
 * \brief Calibrate magnetometer with calculated offsets on all axes
 *
 * Calibrate magnetometer with observed offsets. After calibration,
 * output registers will be:
 *   mag_reg = mag_reg - offset
 *
 * \param[in] handle   - driver handle
 * \param[in] x_offset - x-axis offset in mgauss
 * \param[in] y_offset - y-axis offset in mgauss
 * \param[in] z_offset - z-axis offset in mgauss
 *
 * \return Driver status
 * \retval IIS2MDC_STATUS_OK if call successful
 */
iis2mdc_status_t iis2mdcCalibrate(iis2mdc_handle_t* handle, float x_offset, float y_offset, float z_offset)
{
  chDbgCheck(handle != NULL);

  iis2mdc_status_t ret = IIS2MDC_STATUS_SERIAL_ERROR;

  I2CDriver* i2c = handle->cfg->i2c;

  int16_t raw_offsets[3] = {
    (int16_t)(x_offset / sensitivity),
    (int16_t)(y_offset / sensitivity),
    (int16_t)(z_offset / sensitivity)
  };

  uint8_t tx[7] = {0U};
  tx[0] = offset_x_reg_l_addr;

  (void)memcpy(&tx[1], raw_offsets, sizeof(raw_offsets));

  i2cAcquireBus(i2c);

  if(i2cMasterTransmitTimeout(i2c, iis2mdc_addr, tx, sizeof(tx), NULL, 0, timeout) == MSG_OK) {
    ret = IIS2MDC_STATUS_OK;
  }

  i2cReleaseBus(i2c);

  return ret;
}