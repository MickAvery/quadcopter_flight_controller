/**
 * \file   l3gd20h.c
 * \author Mav Cuyugan
 * \brief  ST L3GD20H MEMS gyroscope module source file
 */

#include "ch.h"
#include "hal.h"
#include "l3gd20h.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if L3GD20H_USE_SPI
#define L3GD20H_SPI_FRAME_MASK

#define L3GD20H_SPI_FRAME_READ           0x80U
#define L3GD20H_SPI_FRAME_WRITE          0x00U

#define L3GD20H_SPI_FRAME_INC_ADDR       0x40U
#define L3GD20H_SPI_FRAME_CONST_ADDR     0x00U

#define L3GD20H_SPI_FRAME_ADDR_MASK      0x3FU
#endif

#define L3GD20H_MULTI_REG_READING        TRUE
#define L3GD20H_SINGLE_REG_READING       FALSE

#define L3GD20H_ODR_IS_VALID(odr)       ((odr > L3GD20H_ODR_UNDERFLOW) && (odr < L3GD20H_ODR_MAX)) 
#define L3GD20H_FS_IS_VALID(fs)         ((fs > L3GD20H_FULLSCALE_UNDERFLOW) && (fs < L3GD20H_FULLSCALE_MAX))

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

typedef uint8_t register_addr_t;

static const float sensitivities[L3GD20H_AXES] =
{
  L3GD20H_245DPS_SENS,
  L3GD20H_500DPS_SENS,
  L3GD20H_2000DPS_SENS
};

/*===========================================================================*/
/* Driver local function declarations.                                       */
/*===========================================================================*/

/**
 * \brief Read contents of a register
 * \param[out] l3gd20h_drv_p     - pointer to L3GD20H Driver object
 * \param[in]  addr              - address of register to read
 * \param[in]  multi_reg_reading - TRUE if reading from consecutive registers
 * \param[out] byte_buf          - buffer to store data from sensor
 * \param[in]  n_bytes           - number of bytes to read
 * \notapi
 */
static void
_read_register(L3GD20HDriver *l3gd20h_drv_p,
        register_addr_t addr,
        bool multi_reg_reading,
        uint8_t *byte_buf,
        size_t n_bytes)
{
  osalDbgCheck(l3gd20h_drv_p != NULL);


#if L3GD20H_USE_SPI
#if SPI_USE_MUTUAL_EXCLUSION
  spiAcquireBus(l3gd20h_drv_p->cfg->spi_drv_p);
#endif
  uint8_t tx_frame =
    L3GD20H_SPI_FRAME_READ | /* we're reading, not writing */
    addr;

  if(multi_reg_reading) {
    tx_frame |= L3GD20H_SPI_FRAME_INC_ADDR; /* we wanna read from multiple
                                             * consecutive registers */
  }

  spiSelect(l3gd20h_drv_p->cfg->spi_drv_p);
  spiSend(l3gd20h_drv_p->cfg->spi_drv_p, 1U, &tx_frame);
  spiReceive(l3gd20h_drv_p->cfg->spi_drv_p, n_bytes, byte_buf);
  spiUnselect(l3gd20h_drv_p->cfg->spi_drv_p);

#if SPI_USE_MUTUAL_EXCLUSION
  spiReleaseBus(l3gd20h_drv_p->cfg->spi_drv_p);
#endif
#elif L3GD20H_USE_I2C
  /* I2C not yet implemented */
#endif
}

/**
 * \brief Write to a register in the device
 * \param[in] l3gd20h_drv_p - pointer to L3GD20H Driver object
 * \param[in] addr          - address of register to read
 * \param[in] byte_buf      - pointer to byte that will be written in register
 * \notapi
 */
static void
_write_register(L3GD20HDriver *l3gd20h_drv_p,
        register_addr_t addr,
        uint8_t *byte_buf)
{
  osalDbgCheck(l3gd20h_drv_p != NULL);
  uint8_t tx_frame_msb =             /* top byte of 16-bit frame */
      L3GD20H_SPI_FRAME_WRITE      | /* we're gonna write */
      L3GD20H_SPI_FRAME_CONST_ADDR | /* don't increment address */
      addr;
  // uint16_t tx_frame = (tx_frame_msb << 8) | *byte_buf;
  uint16_t tx_frame = (*byte_buf << 8) | tx_frame_msb;

#if L3GD20H_USE_SPI
#if SPI_USE_MUTUAL_EXCLUSION
  spiAcquireBus(l3gd20h_drv_p->cfg->spi_drv_p);
#endif
  spiSelect(l3gd20h_drv_p->cfg->spi_drv_p);
  spiSend(l3gd20h_drv_p->cfg->spi_drv_p, 2U, &tx_frame);
  spiUnselect(l3gd20h_drv_p->cfg->spi_drv_p);
#if SPI_USE_MUTUAL_EXCLUSION
  spiReleaseBus(l3gd20h_drv_p->cfg->spi_drv_p);
#endif
#elif L3GD20H_USE_I2C
  /* I2C not yet implemented */
#endif
}

/**
 * Set bits of a reg.
 * tx_frame specifies the bits to be set, performing an OR
 * operation on the bits of the register
 * \param[in] imu_drv  - L3GD20H handle
 * \param[in] addr     - address of register
 * \param[in] tx_frame - specifies the bits to be set
 */
static void
_set_bits_in_reg(L3GD20HDriver* imu_drv, register_addr_t addr, uint8_t tx_frame)
{
  osalDbgCheck(imu_drv);
  uint8_t tx_rx_frame = 0U;
  _read_register(imu_drv, addr, L3GD20H_SINGLE_REG_READING, &tx_rx_frame, 1U);
  tx_rx_frame |= tx_frame;
  _write_register(imu_drv, addr, &tx_rx_frame);
}

/**
 * Reet bits of a reg.
 * tx_frame specifies the bits to be reset, inverting its bits
 * and performing an AND operation on the bits of the register
 * \param[in] imu_drv  - L3GD20H handle
 * \param[in] addr     - address of register
 * \param[in] tx_frame - specifies the bits to be reset
 */
static void
_reset_bits_in_reg(L3GD20HDriver* imu_drv, register_addr_t addr, uint8_t tx_frame)
{
  osalDbgCheck(imu_drv);
  uint8_t tx_rx_frame = 0U;
  _read_register(imu_drv, addr, L3GD20H_SINGLE_REG_READING, &tx_rx_frame, 1U);
  tx_rx_frame &= ~(tx_frame);
  _write_register(imu_drv, addr, &tx_rx_frame);
}

/**
 * \brief Read raw bytes from device output registers
 * \param[in]  imu_drv - L3GD20H handle
 * \param[out] axes    - array to contain raw data
 * \notapi
 */
static void
_read_raw(L3GD20HDriver* imu_drv, int32_t axes[L3GD20H_AXES])
{
  osalDbgCheck(imu_drv != NULL);
  osalDbgAssert(imu_drv->state == L3GD20H_START, "L3GD20H invalid state");

  uint8_t buffer[2 * L3GD20H_AXES] = {0};
  uint8_t i;

  #if L3GD20H_USE_SPI
  /* read from OUT_X_L register up to OUT_Z_H register
   * all in one SPI read */
  _read_register(
      imu_drv,
      OUT_X_L_REG_ADDR,
      L3GD20H_MULTI_REG_READING,
      buffer,
      2 * L3GD20H_AXES);

  for(i = 0 ; i < L3GD20H_AXES ; i++) {
    int16_t temp = buffer[i * 2] + (buffer[(i * 2) + 1] << 8);
    axes[i] = (int32_t)temp;
  }
  #elif L3GD20H_USE_I2C
    #error "I2C interfacing not yet implemented"
  #else
    #error "invalid interface"
  #endif
}

/**
 * \brief Start gyroscope and set device in normal operating or low-ODR mode
 *        depending on ODR selected
 * \param[in] imu_drv - L3GD20H handle
 * \notapi
 */
static void
_start_gyroscope(L3GD20HDriver* imu_drv)
{
  osalDbgCheck(imu_drv != NULL);
  osalDbgAssert(L3GD20H_ODR_IS_VALID(imu_drv->cfg->odr_value), "L3GD20H ODR is invalid");
  osalDbgAssert(L3GD20H_FS_IS_VALID(imu_drv->cfg->fullscale_val), "L3GD20H fullscale is invalid");

  uint8_t odr = 0U;
  uint8_t fs = FS_SHIFT(imu_drv->cfg->fullscale_val);
  uint8_t tx_frame = 0U;

  /* ODR selection */
  if(imu_drv->cfg->odr_value < L3GD20H_ODR_100Hz) {
    /* low ODR */
    odr = (uint8_t)ODR_SHIFT(imu_drv->cfg->odr_value);
    tx_frame = LOW_ODR;
    _set_bits_in_reg(imu_drv, LOW_ODR_ADDR, tx_frame);
  } else {
    odr = (uint8_t)ODR_SHIFT((imu_drv->cfg->odr_value - 3));
    tx_frame = LOW_ODR;
    _reset_bits_in_reg(imu_drv, LOW_ODR_ADDR, tx_frame);
  }

  /* config CTRL1 reg to power up device */
  tx_frame = (odr | PD | ZEN | YEN | XEN);
  _set_bits_in_reg(imu_drv, CTRL1_ADDR, tx_frame);

  /* config CTRL4 for fullscale config */
  tx_frame = fs | BDU;
  _set_bits_in_reg(imu_drv, CTRL4_ADDR, tx_frame);
}

#if L3GD20H_USE_FIFO

/**
 * \brief Configure and enable FIFO
 * \param[in] imu_drv - L3GD20H handle
 * \notapi
 **/
static void
_enable_fifo(L3GD20HDriver* imu_drv)
{
  osalDbgCheck(imu_drv != NULL);
}

#endif /* L3GD20H_USE_FIFO */

#if L3GD20H_USE_INT2

/**
 * \brief Enable interrupt generation (either DRDY or FIFO thershold)
 * \param[in] imu_drv - L3GD20H handle
 * \notapi
 */
static void
_enable_interrupt(L3GD20HDriver* imu_drv)
{
  osalDbgCheck(imu_drv != NULL);
}

#endif /* L3GD20H_USE_INT2 */

/**
 * \brief Initiate the device startup sequence by configuring
 *        the registers
 * \param[in] imu_drv - L3GD20H handle
 * \notapi
 */
static void
_startup_sequence(L3GD20HDriver* imu_drv)
{
  osalDbgCheck(imu_drv != NULL);

  /* start gyroscope */
  _start_gyroscope(imu_drv);

#if L3GD20H_USE_FIFO
  /* configure FIFO */
  _enable_fifo(imu_drv);
#endif

#if L3GD20H_USE_INT2
  /* configure interrupt generation */
  _enable_interrupt(imu_drv);
#endif
}

/**
 * \brief Puts device to sleep
 * \param[in] imu_drv - L3GD20H handle
 */
static void
_sleep_sequence(L3GD20HDriver* imu_drv)
{
  osalDbgCheck(imu_drv != NULL);
  uint8_t tx_frame = 0U;

  tx_frame = PD;
  _set_bits_in_reg(imu_drv, CTRL1_ADDR, tx_frame);
  tx_frame = (ZEN | YEN | XEN);
  _reset_bits_in_reg(imu_drv, CTRL1_ADDR, tx_frame);
}

/**
 * \brief Wake device up from sleeping mode
 * \param[in] imu_drv - L3GD20H handle
 * \notapi
 **/
static void
_wake_sequence(L3GD20HDriver* imu_drv)
{
  osalDbgCheck(imu_drv != NULL);
  uint8_t tx_frame = 0U;

  tx_frame = PD;
  _reset_bits_in_reg(imu_drv, CTRL1_ADDR, tx_frame);
  tx_frame = (ZEN | YEN | XEN);
  _set_bits_in_reg(imu_drv, CTRL1_ADDR, tx_frame);
}

/*===========================================================================*/
/* Driver API functions.                                                     */
/*===========================================================================*/

/**
 * \brief Initialize L3GD20H driver object
 * \param[out] l3gd20h_drv_p - pointer to L3GD20H driver object
 */
void l3gd20hObjectInit(L3GD20HDriver *l3gd20h_drv_p)
{
  osalDbgCheck(l3gd20h_drv_p != NULL);
  l3gd20h_drv_p->cfg = NULL;
  l3gd20h_drv_p->state = L3GD20H_STOP;
}

/**
 * \brief Start the L3GD20H peripheral driver
 * \param[out] l3gd20h_drv_p - pointer to L3GD20H driver
 * \param[in]  cfg           - pointer to L3GD20H config struct to
 *                             configure the driver
 */
void l3gd20hStart(L3GD20HDriver *l3gd20h_drv_p, const L3GD20HConfig *cfg)
{
  osalDbgCheck((l3gd20h_drv_p != NULL) && (cfg != NULL));
  osalDbgAssert((l3gd20h_drv_p->state == L3GD20H_STOP) ||
                (l3gd20h_drv_p->state == L3GD20H_SLEEPING),
                "L3GD20H invalid state");

  if(l3gd20h_drv_p->state == L3GD20H_STOP) {
    l3gd20h_drv_p->cfg = cfg;
    size_t i;

    /* set bias and sensitivities of each axes */
    for(i = 0 ; i < L3GD20H_AXES ; i++) {
      l3gd20h_drv_p->bias[i] = 0.0f;
      l3gd20h_drv_p->sensitivity[i] = sensitivities[cfg->fullscale_val];
    }

    /* set fullscale value based on config */
    l3gd20h_drv_p->fullscale = cfg->fullscale_val;

    /* L3GD20H startup sequence */
    _startup_sequence(l3gd20h_drv_p);
  } else {
    /* in sleep state, wake device up */
    _wake_sequence(l3gd20h_drv_p);
  }

  l3gd20h_drv_p->state = L3GD20H_START;
}

/**
 * \brief Read contents of the WHO_AM_I register of the gyroscope
 * \param[in] l3gd20h_drv_p - pointer to L3GD20H driver
 * \param[in] byte_buf      - byte buffer to store register data into
 */
void l3gd20hWhoAmI(L3GD20HDriver *l3gd20h_drv_p, uint8_t *byte_buf)
{
  osalDbgCheck((l3gd20h_drv_p != NULL) && (byte_buf != NULL));
  osalDbgAssert(l3gd20h_drv_p->state == L3GD20H_START,
                "L3GD20H invalid state");
  _read_register(l3gd20h_drv_p, WHO_AM_I_REG_ADDR, L3GD20H_SINGLE_REG_READING, byte_buf, 1U);
}

/**
 * \brief Determine if new gyro readings are available
 *        for all axes by reading the status register
 * param[in] l3gd20h_drv_p - pointer to L3GD20H driver
 */
bool l3gd20hIsDataReady(L3GD20HDriver *l3gd20h_drv_p)
{
  osalDbgCheck((l3gd20h_drv_p != NULL));
  osalDbgAssert(l3gd20h_drv_p->state == L3GD20H_START,
                "L3GD20h invalid state");

  uint8_t status_reg_content = 0;
  _read_register(l3gd20h_drv_p, STATUS_REG_ADDR, L3GD20H_SINGLE_REG_READING, &status_reg_content, 1U);

  return ((status_reg_content & STATUS_REG_DATA_READY) > 0);
}

/**
 * \brief Read raw data from gyroscope and process data based on bias
 *        values calculated with @ref l3gd20hCalibrate() and sensitivity
 *        determined by chosen full-scale value
 * \note  It is assumed that gyroscope has been calibrated, or else
 *        data processing will produce wrong values
 */
void l3gd20hReadCooked(L3GD20HDriver *l3gd20h_drv_p, float axes[L3GD20H_AXES])
{
  int32_t raw_data[L3GD20H_AXES] = {0, 0, 0};
  size_t  i;

  _read_raw(l3gd20h_drv_p, raw_data);

  for(i = 0 ; i < L3GD20H_AXES ; i++) {
    axes[i] = (float)raw_data[i];
    axes[i] *= l3gd20h_drv_p->sensitivity[i];
    axes[i] -= l3gd20h_drv_p->bias[i];
  }
}

/**
 * \brief Calibrate gyroscope by calculating bias
 * \note see:
 * https://electronics.stackexchange.com/questions/56234/understanding-the-sensitivity-of-an-l3g4200d-gyroscope
 *
 * \param[in] l3gd20h_drv_p - pointer to L3GD20H driver
 */
void l3gd20hCalibrate(L3GD20HDriver *l3gd20h_drv_p)
{
  int32_t raw_data[L3GD20H_AXES] = {0, 0, 0};
  uint32_t i = 0;

  /* take multiple raw data readings */
  while(i < 50) { /* TODO: remove magic number */
    if(l3gd20hIsDataReady(l3gd20h_drv_p)) {
      int32_t buf[L3GD20H_AXES] = {0, 0, 0};
      _read_raw(l3gd20h_drv_p, buf);
      raw_data[L3GD20H_X_AXIS] += buf[L3GD20H_X_AXIS];
      raw_data[L3GD20H_Y_AXIS] += buf[L3GD20H_Y_AXIS];
      raw_data[L3GD20H_Z_AXIS] += buf[L3GD20H_Z_AXIS];
      i++;
    }
    chThdSleepMilliseconds(5);
  }

  /* get average of all raw data readings,
   * and multiply by sensitivity */
  for(i = 0 ; i < L3GD20H_AXES ; i++) {
    /* TODO: remove magic number */
    l3gd20h_drv_p->bias[i] = ((float)raw_data[i]) / 50.0f;
    l3gd20h_drv_p->bias[i] *= l3gd20h_drv_p->sensitivity[i];
  }
}

/**
 * \brief Stop the L3GD20H Peripheral driver, sending it into
 *                 low-power mode.
 * \param[in] l3gd20h_drv_p - pointer to L3GD20H driver
 */
void l3gd20hStop(L3GD20HDriver *l3gd20h_drv_p)
{
  osalDbgCheck(l3gd20h_drv_p != NULL);
  osalDbgAssert((l3gd20h_drv_p->state == L3GD20H_START),
                "L3GD20H invalid state");
  l3gd20h_drv_p->state = L3GD20H_SLEEPING;
  _sleep_sequence(l3gd20h_drv_p);
}
