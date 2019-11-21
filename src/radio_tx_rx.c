/**
 * \file radio_tx_rx.c
 * \author Mav Cuyugan
 *
 * This module is responsible for decoding the PPM input coming from the radio transceiver
 */

#include <string.h>

#include "radio_tx_rx.h"
#include "pinconf.h"

/* global handle for Radio Transceiver */
radio_tx_rx_handle_t RADIO_TXRX;

/**
 * every count in the icucnt_t variable is a time delta of
 * (1 / ICU_CLK_FREQ)
 */
#define ICU_CLK_FREQ (100U * 1000U)

/*
 * \notapi
 * \brief Convert milliseconds to ICU ticks
 */
#define MS_TO_ICU_TICKS(ms) (ms * (ICU_CLK_FREQ / 1000U))

/**
 * a PPM pulse width greater than this threshold is the last pulse in a frame
 */
static icucnt_t CHAN_WIDTH_THRESHOLD = MS_TO_ICU_TICKS(5U);

/**
 * \notapi
 * Convert the pulse width into a percent value knowing what the minimum and maximum widths are.
 *
 * To avoid floating point arithmetic, percente values are multiplied by 100,
 * such that 100% becomes 10000, 40% becomes 4000, and 60.75% becomes 6075
 */
static uint32_t pulse_width_to_percent(icucnt_t width)
{
  /* TODO: magic numbers!!! */

  if(width > 160U) {
    width = 160U;
  } else if(width < 60U) {
    width = 60U;
  }

  return ((width - 60U) * 10000U) / (160U - 60U);
}

/**
 * \notapi
 * Here we can determine the pulse width of a channel
 */
static void icuWidthCb(ICUDriver *icup)
{
  icucnt_t pulse_width = icuGetWidthX(icup);

  if(RADIO_TXRX.state == RADIO_TXRX_WAITING) {
    /* we're waiting for the first frame */

    if(pulse_width >= CHAN_WIDTH_THRESHOLD) {
      /* a pulse over the threshold means we've reached the end of the frame,
         so get ready to read the next incoming frame */
      RADIO_TXRX.active_channel = RADIO_TXRX_CHAN0;
      RADIO_TXRX.state = RADIO_TXRX_ACTIVE;
    }

  } else {
    /* we're reading the pulse widths of each channel */

    if(pulse_width >= CHAN_WIDTH_THRESHOLD) {

      /* we've reached the end of the frame */
      RADIO_TXRX.active_channel = RADIO_TXRX_CHAN0;

    } else {

      radio_tx_rx_channel_t chan = RADIO_TXRX.active_channel;

      if(chan >= RADIO_TXRX_CHANNELS) {

        /* something's wrong, we're reading too many channels... */
        RADIO_TXRX.active_channel = RADIO_TXRX_CHAN0;
        RADIO_TXRX.state = RADIO_TXRX_WAITING;

      } else {

        uint32_t percent = pulse_width_to_percent(pulse_width);

        RADIO_TXRX.channels[chan] = percent;
        RADIO_TXRX.active_channel++;

      }
    }
  }
}

/**
 * \notapi
 */
static void icuOverflowCb(ICUDriver *icup)
{
  (void)icup;

  RADIO_TXRX.state = RADIO_TXRX_WAITING;
  RADIO_TXRX.active_channel = RADIO_TXRX_CHAN0;
}

/**
 * Input Capture configuration
 */
static ICUConfig icucfg =
{
  ICU_INPUT_ACTIVE_HIGH,
  ICU_CLK_FREQ,
  icuWidthCb,
  NULL,
  icuOverflowCb,
  ICU_CHANNEL_1,
  0
};

/**
 * \brief Initialize the radio transceiver module
 * \param[in] handle - radio transceiver handle
 */
void radioTxRxInit(radio_tx_rx_handle_t* handle)
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(handle->state == RADIO_TXRX_UNINIT);

  (void)memset((void*)handle->channels, 0U, RADIO_TXRX_CHANNELS*sizeof(uint32_t));
  handle->active_channel = RADIO_TXRX_CHAN0;
  handle->state = RADIO_TXRX_STOP;
}

/**
 * \brief Start the Radio Transceiver module
 * \param[in] handle - radio transceiver handle
 */
void radioTxRxStart(radio_tx_rx_handle_t* handle)
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(handle->state == RADIO_TXRX_STOP);

  icuStart(&ICUD3, &icucfg);
  palSetPadMode(ICU_PORT, ICU_PADNUM, PAL_MODE_ALTERNATE(ICU_ALTMODE));

  icuStartCapture(&ICUD3);
  icuEnableNotifications(&ICUD3); /* enable callbacks */

  handle->state = RADIO_TXRX_WAITING;
}

/**
 * \brief Read the signal from each channel coming from the PPM input
 * \param[in]  handle - radio transceiver handle
 * \param[out] channels - signal values on each channel
 */
void radioTxRxReadInputs(radio_tx_rx_handle_t* handle, uint32_t channels[RADIO_TXRX_CHANNELS])
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(
    (handle->state == RADIO_TXRX_WAITING) ||
    (handle->state == RADIO_TXRX_ACTIVE));

  for(size_t i = 0U ; i < RADIO_TXRX_CHANNELS ; i++) {
    channels[i] = handle->channels[i];
  }

}