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

/* if a pulse has a period greater than this,
   then we can consider it as the last pulse in the frame */
static icucnt_t CHANNEL_PERIOD_THRESHOLD = 10U;

/**
 * \notapi
 * Here we can determine the pulse width of a channel
 */
static void icuWidthCb(ICUDriver *icup) {

  icucnt_t pulse_width = icuGetWidthX(icup);

  if(RADIO_TXRX.state == RADIO_TXRX_ACTIVE) {
    radio_tx_rx_channel_t chan = RADIO_TXRX.incoming_channel;

    osalMutexLock(&RADIO_TXRX.lock);

    RADIO_TXRX.channels[chan] = pulse_width;

    osalMutexUnlock(&RADIO_TXRX.lock);

  }

}

/**
 * \notapi
 * Here we can determine the width between activation edges,
 * to determine if we've reached the end of the PPM frame
 */
static void icuPeriodCb(ICUDriver *icup) {

  icucnt_t period_width = icuGetPeriodX(icup);

  if(RADIO_TXRX.state == RADIO_TXRX_WAITING) {
    /* waiting for first channel pulse */

    /* if period width is above threshold, then incoming pulse is from a new frame */
    if(period_width >= CHANNEL_PERIOD_THRESHOLD) {
      RADIO_TXRX.state = RADIO_TXRX_ACTIVE;
      RADIO_TXRX.incoming_channel = RADIO_TXRX_CHAN0;
    }

  } else {

    /* if period width is above threshold, then incoming pulse is from a new frame */
    if(period_width >= CHANNEL_PERIOD_THRESHOLD) { /* TODO: properly set threshold */
      RADIO_TXRX.incoming_channel = RADIO_TXRX_CHAN0;
    } else {
      if(RADIO_TXRX.incoming_channel + 1 >= RADIO_TXRX_CHANNELS) {

        /* We're somehow getting more channels than necessary on the signal???
           Set state back to waiting so we wait on a new frame */
        RADIO_TXRX.state = RADIO_TXRX_WAITING;

      } else {

        /* move on to the next channel */
        RADIO_TXRX.incoming_channel += 1;

      }
    }
  }
}

/**
 * Input Capture configuration
 */
static ICUConfig icucfg =
{
  ICU_INPUT_ACTIVE_HIGH,
  10000, /* 10kHz ICU clock frequency.   */
  icuWidthCb,
  icuPeriodCb,
  NULL, /* timer overflow callback, not needed */
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

  osalMutexObjectInit(&handle->lock);

  (void)memset((void*)handle->channels, 0U, RADIO_TXRX_CHANNELS*sizeof(icucnt_t));
  handle->incoming_channel = RADIO_TXRX_CHAN0;
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
void radioTxRxReadInputs(radio_tx_rx_handle_t* handle, icucnt_t channels[RADIO_TXRX_CHANNELS])
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(
    (handle->state == RADIO_TXRX_WAITING) ||
    (handle->state == RADIO_TXRX_ACTIVE));

  osalMutexLock(&handle->lock);

  for(size_t i = 0U ; i < RADIO_TXRX_CHANNELS ; i++) {
    channels[i] = handle->channels[i];
  }

  osalMutexUnlock(&handle->lock);
}