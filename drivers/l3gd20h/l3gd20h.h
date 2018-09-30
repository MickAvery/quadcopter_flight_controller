/**
 * \file   l3gd20h.h
 * \author Mav Cuyugan
 * \brief  ST L3GD20H MEMS gyroscope module header
 *
 * \addtogroup L3GD20H
 * \{
 */

#ifndef L3GD20H_H
#define L3GD20H_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#include "l3gd20h_reg.h"

#define L3GD20H_AXES             3U
#define L3GD20H_X_AXIS           0U
#define L3GD20H_Y_AXIS           1U
#define L3GD20H_Z_AXIS           2U

/* Full-scale float values */
#define L3GD20H_245DPS           245.0f
#define L3GD20H_500DPS           500.0f
#define L3GD20H_2000DPS          2000.0f

/* Sensitivity float values,
 * see Table 3 of L3GD20H datasheet */
#define L3GD20H_245DPS_SENS      0.00875f
#define L3GD20H_500DPS_SENS      0.01750f
#define L3GD20H_2000DPS_SENS     0.07000f

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

#if !defined(L3GD20H_USE_SPI) || defined(__DOXYGEN__)
#define L3GD20H_USE_SPI          TRUE
#endif

#if !defined(L30GD20H_USE_I2C) || defined (__DOXYGEN__)
#define L3GD20H_USE_I2C          FALSE
#endif

/**
 * \brief Set to TRUE if you want gyroscope data-ready signal
 *        to be driven to INT2 (active when high)
 */
#if !defined(L3GD20H_USE_INT2) || defined (__DOXYGEN__)
#define L3GD20H_USE_INT2         FALSE
#endif

/**
 * \brief Set to TRUE if you want to use the device's internal FIFO.
 *        Once set, you can configure FIFO settings in @ref L3GD20HConfig 
 */
#if !defined(L3GD20H_USE_FIFO) || defined (__DOXYGEN__)
#define L3GD20H_USE_FIFO         FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(L3GD20H_USE_SPI ^ L3GD20H_USE_I2C)
#error "Can't use both SPI and I2C for L3GD20H module, you gotta pick one"
#endif

#if !(L3GD20H_USE_SPI || L3GD20H_USE_I2C)
#error "No serial interface chosen"
#endif

#if L3GD20H_USE_SPI && !SPI_USE_WAIT
#error "SPI needs synchronous API, SPI_USE_WAIT has to be set to TRUE"
#endif

#if L3GD20H_USE_I2C
#error "I2C not yet implemented"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * \brief Device states
 */
typedef enum
{
  L3GD20H_STOP = 0, /*<! device not configured */
  L3GD20H_START,    /*<! device in normal oeprating mode */
  L3GD20H_SLEEPING  /*<! device sleeping */
} l3gd20h_state_t;

/**
 * \brief Device fullscale selection
 */
typedef enum
{
  L3GD20H_FULLSCALE_UNDERFLOW = -1,

  L3GD20H_FULLSCALE_245DPS = 0, /*<! +/- 245 dps */
  L3GD20H_FULLSCALE_500DPS,     /*<! +/- 500 dps */
  L3GD20H_FULLSCALE_2000DPS,    /*<! +/- 2000 dps */
  L3GD20H_FULLSCALE_MAX
} l3gd20h_fullscale_t;

/**
 * \brief Operating Data Rates,
 *        aka sampling rate of gyroscope
 */
typedef enum
{
  L3GD20H_ODR_UNDERFLOW = -1,

  /* Low operational data rates (ODR) */
  L3GD20H_LOW_ODR_12_5Hz = 0, /*<! 12.5 Hz ODR */
  L3GD20H_LOW_ODR_25Hz,       /*<! 25 Hz ODR */
  L3GD20H_LOW_ODR_50Hz,       /*<! 50 Hz ODR */

  /* Normal operational data rates (ODR) */
  L3GD20H_ODR_100Hz, /*<! 100 Hz ODR */
  L3GD20H_ODR_200Hz, /*<! 200 Hz ODR */
  L3GD20H_ODR_400Hz, /*<! 400 Hz ODR */
  L3GD20H_ODR_800Hz, /*<! 800 Hz ODR */
  L3GD20H_ODR_MAX
} l3gd20h_odr_value_t;

/**
 * \brief Device configurations
 */
typedef struct
{
  l3gd20h_fullscale_t fullscale_val;
  l3gd20h_odr_value_t odr_value;

#if L3GD20H_USE_SPI || defined(__DOXYGEN__)
  SPIDriver *spi_drv_p;     /*<! SPI driver pointer */
#elif L3GD20H_USE_I2C || defined(__DOXYGEN__)
#error "I2C not yet implemented"
#endif
} L3GD20HConfig;

/**
 * \brief Device driver struct
 */
typedef struct
{
  l3gd20h_state_t state;    /*<!  */
  const L3GD20HConfig *cfg; /*<!  */
  float bias[L3GD20H_AXES];
  float sensitivity[L3GD20H_AXES];
  l3gd20h_fullscale_t fullscale;
} L3GD20HDriver;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

void l3gd20hObjectInit(L3GD20HDriver *l3gd20h_drv_p);
void l3gd20hStart(L3GD20HDriver *l3gd20h_drv_p, const L3GD20HConfig *cfg);
void l3gd20hWhoAmI(L3GD20HDriver *l3gd20h_drv_p, uint8_t *byte_buf);
bool l3gd20hIsDataReady(L3GD20HDriver *l3gd20h_drv_p);
void l3gd20hReadCooked(L3GD20HDriver *l3gd20h_drv_p, float axes[L3GD20H_AXES]);
void l3gd20hCalibrate(L3GD20HDriver *l3gd20h_drv_p);
void l3gd20hStop(L3GD20HDriver *l3gd20h_drv_p);

#ifdef __cplusplus
}
#endif

#endif /* L3GD20H_H */
/**
 * \}
 */
