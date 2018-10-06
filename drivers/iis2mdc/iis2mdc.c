/**
 * \file   iis2mdc.c
 * \author Mav Cuyugan
 * \brief  IIS2MDC API implementation
 */

#include <stddef.h>
#include "osal.h"
#include "iis2mdc.h"

static i2caddr_t iis2mdc_addr = 0b00011110;

static systime_t timeout = MS2ST(10U);

static uint8_t cfg_reg_a_addr = 0x60U;

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