/**
 * \file   lis3dsh.c
 * \author Mav Cuyugan
 * \brief  LIS3DSH accelerometer driver source
 **/

#include "hal.h"
#include "lis3dsh.h"
#include "lis3dsh_reg.h"

/*===========================================================================*/
/* Driver local definitions                                                  */
/*===========================================================================*/

#define LIS3DSH_SPI_READ    0x80U
#define LIS3DSH_SPI_WRITE   0x00U

#define LIS3DSH_FS_IS_VALID(fs)   ((fs > LIS3DSH_FS_UNDERFLOW) && (fs < LIS3DSH_FS_MAX))
#define LIS3DSH_ODR_IS_VALID(odr) ((odr > LIS3DSH_ODR_UNDERFLOW) && (odr < LIS3DSH_ODR_MAX))

#define LIS3DSH_2G_SENS     0.00006f
#define LIS3DSH_4G_SENS     0.00012f
#define LIS3DSH_6G_SENS     0.00018f
#define LIS3DSH_8G_SENS     0.00024f
#define LIS3DSH_16G_SENS    0.00073f
#define LIS3DSH_SENS_MAX    5U

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

typedef uint8_t register_addr_t;

static const float sensitivities[LIS3DSH_SENS_MAX] =
{
  LIS3DSH_2G_SENS,
  LIS3DSH_4G_SENS,
  LIS3DSH_6G_SENS,
  LIS3DSH_8G_SENS,
  LIS3DSH_16G_SENS
};

/*===========================================================================*/
/* Driver local functions                                                    */
/*===========================================================================*/

/**
 * \brief Read n-number of bytes from a register and store them to buf
 * \note  For multi-byte reads, register address is incremented
 *        because if you're reading multipe bytes, it's usually from
 *        the output registers anyway
 * \param[in]  handle - LIS3DSH handle
 * \param[in]  addr   - register address
 * \param[out] buf    - buffer to store bytes to
 * \param[in]  n      - number of bytes to read
 * \notapi
 **/
static void
_read_register(
    LIS3DSHDriver* handle,
    register_addr_t addr,
    uint8_t* buf,
    size_t n)
{
  osalDbgCheck(handle != NULL);

#if SPI_USE_MUTUAL_EXCLUSION
  spiAcquireBus(handle->cfg->spi_drv_p);
#endif
  uint8_t tx_frame =
    LIS3DSH_SPI_READ | /* we're reading, not writing */
    addr;

  spiSelect(handle->cfg->spi_drv_p);
  spiSend(handle->cfg->spi_drv_p, 1U, &tx_frame);
  spiReceive(handle->cfg->spi_drv_p, n, buf);
  spiUnselect(handle->cfg->spi_drv_p);

#if SPI_USE_MUTUAL_EXCLUSION
  spiReleaseBus(handle->cfg->spi_drv_p);
#endif
}

/**
 * \brief Write byte into a register
 * \param[in] handle - LIS3DSH handle
 * \param[in] addr   - register address
 * \param[in] byte   - byte to write
 * \notapi
 **/
static void
_write_register(
    LIS3DSHDriver* handle,
    register_addr_t addr,
    uint8_t byte)
{
  osalDbgCheck(handle != NULL);
  uint8_t tx_frame_lsb =       /* LSB of 16-bit frame */
      LIS3DSH_SPI_WRITE      | /* we're gonna write */
      addr;
  uint16_t tx_frame = (byte << 8) | tx_frame_lsb;

#if SPI_USE_MUTUAL_EXCLUSION
  spiAcquireBus(handle->cfg->spi_drv_p);
#endif
  spiSelect(handle->cfg->spi_drv_p);
  spiSend(handle->cfg->spi_drv_p, 2U, &tx_frame);
  spiUnselect(handle->cfg->spi_drv_p);
#if SPI_USE_MUTUAL_EXCLUSION
  spiReleaseBus(handle->cfg->spi_drv_p);
#endif
}

/**
 * Set bits in a register, ignoring other bits you don't intend to modify.
 * Bits to be set are specified in the byte argument.
 * \param[in] handle - LIS3DSH handle
 * \param[in] addr   - register address
 * \param[in] byte   - specifies bits to set
 * \notapi
 **/
static void
_set_bits_in_reg(
    LIS3DSHDriver* handle,
    register_addr_t addr,
    uint8_t byte)
{
  osalDbgCheck(handle != NULL);
  uint8_t rx_tx_frame = 0U;

  _read_register(handle, addr, &rx_tx_frame, 1U);
  rx_tx_frame |= byte;
  _write_register(handle, addr, rx_tx_frame);
}

/**
 * Reset bits in a register, ignoring other bits you don't intend to modify.
 * Bits to be reset are specified in the byte argument.
 * \param[in] handle - LIS3DSH handle
 * \param[in] addr   - register address
 * \param[in] byte   - specifies bits to reset
 * \notapi
 **/
static void
_reset_bits_in_reg(
    LIS3DSHDriver* handle,
    register_addr_t addr,
    uint8_t byte)
{
  osalDbgCheck(handle != NULL);
  uint8_t rx_tx_frame = 0U;

  _read_register(handle, addr, &rx_tx_frame, 1U);
  rx_tx_frame &= ~byte;
  _write_register(handle, addr, rx_tx_frame);
}

/**
 * \brief Read raw data from output registers
 * \param[in]  handle - LIS3DSH handle
 * \param[out] axes   - buffer to store raw data
 * \notapi
 **/
static void
_read_raw(LIS3DSHDriver* handle, int32_t axes[LIS3DSH_AXES])
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(axes != NULL);
  uint8_t raw[LIS3DSH_AXES * 2U] = {0};
  size_t i;

  _read_register(handle, OUT_X_L, raw, LIS3DSH_AXES * 2U);

  for(i = 0U ; i < LIS3DSH_AXES ; i++) {
    int16_t temp = raw[i * 2] + (raw[(i * 2) + 1] << 8);
    axes[i] = (int32_t)temp;
  }
}

/**
 * \brief Enable auto-address increment when accessing multiple bytes
 * \param[in] handle - LIS3DSH handle
 * \notapi
 **/
static void
_enable_auto_addr_increment(LIS3DSHDriver* handle)
{
  osalDbgCheck(handle != NULL);
  uint8_t tx_frame = ADD_INC;
  _set_bits_in_reg(handle, CTRL_REG6_ADDR, tx_frame);
}

/**
 * \brief Set accelerometer fullscale
 * \param[in] handle - LIS3DSH handle
 * \notapi
 **/
static void
_set_acc_fullscale(LIS3DSHDriver* handle)
{
  osalDbgCheck(handle != NULL);
  osalDbgAssert(LIS3DSH_FS_IS_VALID(handle->cfg->fullscale_val), "LIS3DSH invalid fullscale selection");
  uint8_t tx_frame = LIS3DSH_FS_SHIFT(handle->cfg->fullscale_val);
  _set_bits_in_reg(handle, CTRL_REG5_ADDR, tx_frame);
}

/**
 * \brief Set accelerometer ODR and set device in normal mode
 * \param[in] handle - LIS3DSH handle
 * \notapi
 **/
static void
_set_acc_odr(LIS3DSHDriver* handle)
{
  osalDbgCheck(handle != NULL);
  osalDbgAssert(LIS3DSH_ODR_IS_VALID(handle->cfg->odr_value), "LIS3DSH invalid ODR selection");
  uint8_t tx_frame = LIS3DSH_ODR_SHIFT(handle->cfg->odr_value);
  _set_bits_in_reg(handle, CTRL_REG4_ADDR, tx_frame);
}

/**
 * \brief Sequence to start device
 * \param[in] handle - LIS3DSH handle
 * \notapi
 **/
static void
_startup_sequence(LIS3DSHDriver* handle)
{
  osalDbgCheck(handle != NULL);
  
  /* enable auto-increment of register address for multi-byte reads */
  _enable_auto_addr_increment(handle);

  /* set accelerometer fullscale */
  _set_acc_fullscale(handle);

  /* set accelerometer ODR, setting device in normal mode */
  _set_acc_odr(handle);
}

/**
 * \brief Disable device and set it to low-power mode
 * \param[in] handle - LIS3DSH handle
 **/
static void
_sleep_sequence(LIS3DSHDriver* handle)
{
  osalDbgCheck(handle != NULL);
  uint8_t tx_frame = ODR;
  _reset_bits_in_reg(handle, CTRL_REG4_ADDR, tx_frame);
}

/**
 * \brief Power device back up from sleep
 * \param[in] handle - LIS3DSH handle
 **/
static void
_wakeup_sequence(LIS3DSHDriver* handle)
{
  osalDbgCheck(handle != NULL);
  uint8_t tx_frame = LIS3DSH_ODR_SHIFT(handle->cfg->odr_value);
  _set_bits_in_reg(handle, CTRL_REG4_ADDR, tx_frame);
}

/*===========================================================================*/
/* Driver API                                                                */
/*===========================================================================*/

/**
 * \brief Initialize LIS3DSH driver object
 * \param[in] lis3dsh - LIS3DSH handle
 **/
void lis3dshObjectInit(LIS3DSHDriver* lis3dsh)
{
  osalDbgCheck(lis3dsh != NULL);
  lis3dsh->cfg = NULL;
  lis3dsh->state = LIS3DSH_STOP;
}

/**
 * \brief Start LIS3DSH device
 * \param[in] lis3dsh - LIS3DSH handle
 * \param[in] cfg     - Device configurations
 **/
void lis3dshStart(LIS3DSHDriver* lis3dsh, const LIS3DSHConfig* cfg)
{
  osalDbgCheck(lis3dsh != NULL);
  osalDbgCheck(cfg != NULL);
  osalDbgAssert((lis3dsh->state == LIS3DSH_STOP) || (lis3dsh->state == LIS3DSH_SLEEPING),
      "LIS3DSH invalid state");

  if(lis3dsh->state == LIS3DSH_STOP) {
    lis3dsh->cfg = cfg;
    size_t i;

    for(i = 0U ; i < LIS3DSH_AXES ; i++) {
      lis3dsh->sensitivity[i] = sensitivities[cfg->fullscale_val];
    }

    _startup_sequence(lis3dsh);
  } else {
    _wakeup_sequence(lis3dsh);
  }

  lis3dsh->state = LIS3DSH_START;
}

/**
 * \brief Set device on low-power mode
 * \param[in] lis3dsh - LIS3DSH handle
 **/
void lis3dshStop(LIS3DSHDriver* lis3dsh)
{
  osalDbgCheck(lis3dsh != NULL);
  osalDbgAssert(lis3dsh->state == LIS3DSH_START, "LIS3DSH invalid state");
  _sleep_sequence(lis3dsh);
}

/**
 * \brief Get WHO_AM_I register value
 * \param[in]  lis3dsh - LIS3DSH handle
 * \param[out] buf     - byte buffer to store value
 **/
void lis3dshWhoAmI(LIS3DSHDriver* lis3dsh, uint8_t* buf)
{
  osalDbgCheck(lis3dsh != NULL);
  osalDbgCheck(buf != NULL);
  osalDbgAssert(lis3dsh->state == LIS3DSH_START, "LIS3DSH invalid state");

  _read_register(lis3dsh, LIS3DSH_WHO_AM_I_ADDR, buf, 1U);
}

/**
 * \brief Check to see if data is ready to retrieve
 * \param[in] lis3dsh - LIS3DSH handle
 * \return TRUE if data is ready to retrieve, FALSE otherwise
 **/
bool lis3dshIsDataReady(LIS3DSHDriver* lis3dsh)
{
  uint8_t rx_frame = 0U;
  _read_register(lis3dsh, STATUS_ADDR, &rx_frame, 1U);
  return ((rx_frame & ZYXDA) > 0U);
}

void lis3dshGetData(LIS3DSHDriver* lis3dsh, float axes[LIS3DSH_AXES])
{
  size_t i;
  int32_t raw_data[LIS3DSH_AXES] = {0};
  int8_t  offset[LIS3DSH_AXES] = {0};

  /* read raw data */
  _read_raw(lis3dsh, raw_data);
  /* read offset for each axis */
  _read_register(lis3dsh, OFF_X_ADDR, (uint8_t*)offset, LIS3DSH_AXES);

  for(i = 0U ; i < LIS3DSH_AXES ; i++) {
    //raw_data[i] -= (raw_data[i] - ((int32_t)offset[i] * 32));
    axes[i] = raw_data[i] * lis3dsh->sensitivity[i];
  }
}
