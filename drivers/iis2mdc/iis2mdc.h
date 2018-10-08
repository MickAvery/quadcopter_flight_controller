/**
 * \file   iis2mdc.h
 * \author Mav Cuyugan
 * \brief  IIS2MDC driver API
 */

#ifndef IIS2MDC_H
#define IIS2MDC_H

#include "hal.h"

typedef enum
{
  IIS2MDC_STATUS_OK = 0,
  IIS2MDC_STATUS_ERROR = -1,
  IIS2MDC_STATUS_SERIAL_ERROR = -2
} iis2mdc_status_t;

typedef enum
{
  IIS2MDC_UNINIT = 0,
  IIS2MDC_STOP,
  IIS2MDC_RUNNING
} iis2mdc_state_t;

typedef enum
{
  IIS2MDC_ODR_10_Hz = 0,
  IIS2MDC_ODR_20_Hz,
  IIS2MDC_ODR_50_Hz,
  IIS2MDC_ODR_100_Hz
} iis2mdc_odr_t;

typedef struct
{
  float mag_x;
  float mag_y;
  float mag_z;
} iis2mdc_sensor_readings_t;

typedef struct
{
  I2CDriver* i2c;
  iis2mdc_odr_t odr;
} iis2mdc_config_t;

typedef struct
{
  const iis2mdc_config_t* cfg;
  iis2mdc_state_t state;
} iis2mdc_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initialize IIS2MDC driver handle object
 *
 * \param[out] handle - driver handle to initialize
 */
void iis2mdcObjectInit(iis2mdc_handle_t* handle);

/**
 * \brief Configure and start IIS2MDC magnetometer
 *
 * \param[in] handle - driver handle
 * \param[in] cfg - configurations
 *
 * \return Driver status
 * \retval IIS2MDC_STATUS_OK if call successful
 */
iis2mdc_status_t iis2mdcStart(iis2mdc_handle_t* handle, const iis2mdc_config_t* cfg);

/**
 * \brief Read magnetometer data in mgauss
 *
 * \param[in]  handle - driver handle
 * \param[out] vals   - output struct to store sensor readings
 *
 * \return Driver status
 * \retval IIS2MDC_STATUS_OK if call successful
 */
iis2mdc_status_t iis2mdcRead(iis2mdc_handle_t* handle, iis2mdc_sensor_readings_t* vals);

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
iis2mdc_status_t iis2mdcCalibrate(iis2mdc_handle_t* handle, float x_offset, float y_offset, float z_offset);

#ifdef __cplusplus
}
#endif

#endif /* IIS2MDC */