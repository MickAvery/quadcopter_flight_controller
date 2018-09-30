/**
 * \file   lis3dsh.c
 * \author Mav Cuyugan
 * \brief  LIS3DSH accelerometer driver header
 **/

#ifndef LIS3DSH_H
#define LIS3DSH_H

#define LIS3DSH_AXES        3U

/**
 * \brief LIS3DSH states
 **/
typedef enum
{
  LIS3DSH_STOP = 0,
  LIS3DSH_START,
  LIS3DSH_SLEEPING
} lis3dsh_state_t;

/**
 * \brief LIS3DSH fullscale selection
 **/
typedef enum
{
  LIS3DSH_FS_UNDERFLOW = -1,
  LIS3DSH_FS_2G,  /*<! +/- 2G */
  LIS3DSH_FS_4G,  /*<! +/- 4G */
  LIS3DSH_FS_6G,  /*<! +/- 6G */
  LIS3DSH_FS_8G,  /*<! +/- 8G */
  LIS3DSH_FS_16G, /*<! +/- 16G */
  LIS3DSH_FS_MAX
} lis3dsh_fullscale_t;

/**
 * \brief LIS3DSH ODR selection
 **/
typedef enum
{
  LIS3DSH_ODR_UNDERFLOW = -1,
  LIS3DSH_ODR_POWERDOWN, /*<! device powered-down */
  LIS3DSH_ODR_3_125HZ,   /*<! 3.125 Hz ODR */
  LIS3DSH_ODR_6_25HZ,    /*<! 6.25 Hz ODR */
  LIS3DSH_ODR_12_5HZ,    /*<! 12.5 Hz ODR */
  LIS3DSH_ODR_25HZ,      /*<! 25 Hz ODR */
  LIS3DSH_ODR_50HZ,      /*<! 50 Hz ODR */
  LIS3DSH_ODR_100HZ,     /*<! 100 Hz ODR */
  LIS3DSH_ODR_400HZ,     /*<! 400 Hz ODR */
  LIS3DSH_ODR_800HZ,     /*<! 800 Hz ODR */
  LIS3DSH_ODR_1600HZ,    /*<! 1600 Hz ODR */
  LIS3DSH_ODR_MAX
} lis3dsh_odr_value_t;

/**
 * \brief LIS3DSH Configurations
 **/
typedef struct
{
  lis3dsh_fullscale_t fullscale_val; /*<! Accelerometer fullscale */
  lis3dsh_odr_value_t odr_value;     /*<! Accelerometer sampling rate */

  SPIDriver *spi_drv_p;              /*<! SPI driver pointer */
} LIS3DSHConfig;

/**
 * \brief LIS3DSH Driver struct
 **/
typedef struct
{
  lis3dsh_state_t state;           /*<! Driver state */
  const LIS3DSHConfig *cfg;        /*<! Driver configurations */
  float sensitivity[LIS3DSH_AXES]; /*<! Output sensitivity */
} LIS3DSHDriver;

/*===========================================================================*/
/* Driver API                                                                */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void lis3dshObjectInit(LIS3DSHDriver* lis3dsh);
void lis3dshStart(LIS3DSHDriver* lis3dsh, const LIS3DSHConfig* cfg);
void lis3dshStop(LIS3DSHDriver* lis3dsh);
void lis3dshWhoAmI(LIS3DSHDriver* lis4dsh, uint8_t* buf);
bool lis3dshIsDataReady(LIS3DSHDriver* lis3dsh);
void lis3dshGetData(LIS3DSHDriver* lis3dsh, float axes[LIS3DSH_AXES]);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DSH_H */
