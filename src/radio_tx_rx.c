/**
 * \file radio_tx_rx.c
 * \author Mav Cuyugan
 *
 * This module is responsible for decoding the PPM input coming from the radio transceiver
 */

#include "radio_tx_rx.h"
#include "pinconf.h"

/* global handle for Radio Transceiver */
radio_tx_rx_handle_t RADIO_TXRX;

/**
 * \notapi
 * Here we can determine the pulse width of a channel
 */
static void icuWidthCb(ICUDriver *icup) {

  (void)icuGetWidthX(icup);

}

/**
 * \notapi
 * Here we can determine the width between activation edges,
 * to determine if we've reached the end of the PPM frame
 */
static void icuPeriodCb(ICUDriver *icup) {

  (void)icuGetPeriodX(icup);

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
 * Radio Transceiver input capture thread
 */
// THD_WORKING_AREA(radioTxRxThreadWorkingArea, 256);
// THD_FUNCTION(radioTxRxThread, arg)
// {
//   (void)arg;

//   while(true) {

//   }
// }

/**
 * \brief Initialize the radio transceiver module
 * \param[in] handle - radio transceiver handle
 */
void radioTxRxInit(radio_tx_rx_handle_t* handle)
{
  osalDbgCheck(handle != NULL);
  osalDbgCheck(handle->state == RADIO_TXRX_UNINIT);

  handle->state = RADIO_TXRX_STOP;
}

/**
 * \brief Start the Radio Transceiver module
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

  /* start the IMU Engine Thread */
  // chThdCreateStatic(
  //   radioTxRxThreadWorkingArea,
  //   sizeof(radioTxRxThreadWorkingArea),
  //   NORMALPRIO,
  //   radioTxRxThread,
  //   NULL);
}