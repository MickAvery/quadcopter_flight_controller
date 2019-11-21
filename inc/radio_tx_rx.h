/**
 * \file radio_tx_rx.h
 * \author Mav Cuyugan
 *
 * This module is responsible for decoding the PPM input coming from the radio transceiver
 */

#ifndef RADIO_TX_RX_H
#define RADIO_TX_RX_H

#include "hal.h"

/**
 * Radio transceiver states
 */
typedef enum
{
  RADIO_TXRX_UNINIT = 0, /*!< Transceiver is uninitialized */
  RADIO_TXRX_STOP,       /*!< Transmitter is waiting to be started */
  RADIO_TXRX_WAITING,    /*!< Transmitter has been started and is waiting on the first frame */
  RADIO_TXRX_ACTIVE      /*!< Transmitter is reading the signals from each channel in the PPM input */
} radio_tx_rx_state_t;

typedef enum
{
  RADIO_TXRX_CHAN0 = 0,
  RADIO_TXRX_CHAN1,
  RADIO_TXRX_CHAN2,
  RADIO_TXRX_CHAN3,
  RADIO_TXRX_CHAN4,
  RADIO_TXRX_CHAN5,
  RADIO_TXRX_CHAN6,
  RADIO_TXRX_CHAN7,
  RADIO_TXRX_CHANNELS
} radio_tx_rx_channel_t;

/**
 * The signals corresponding to each channel of the PPM input,
 * assuming that the FS-iA6B transceiver is used along with the FS-i6 radio.
 */
typedef enum
{
  RADIO_TXRX_ROLL = 0,
  RADIO_TXRX_PITCH,
  RADIO_TXRX_ALTITUDE,
  RADIO_TXRX_YAW,
  RADIO_TXRX_VRA,
  RADIO_TXRX_VRB,
  RADIO_TXRX_UNKNOWN_A,
  RADIO_TXRX_UNKNOWN_B
} radio_tx_rx_chan_def_t;

/**
 * Radio transceiver handle
 */
typedef struct
{
  radio_tx_rx_state_t state;              /*!< State of the radio transceiver */
  radio_tx_rx_channel_t active_channel;   /*!< The next pulse corresponds to this channel */
  uint32_t channels[RADIO_TXRX_CHANNELS]; /*!< The signal strength of each channel (in percent) */
} radio_tx_rx_handle_t;

/* global handle for Radio Transceiver */
extern radio_tx_rx_handle_t RADIO_TXRX;

/**
 * \brief Initialize the radio transceiver module
 * \param[in] handle - radio transceiver handle
 */
void radioTxRxInit(radio_tx_rx_handle_t* handle);

/**
 * \brief Start the Radio Transceiver module
 * \param[in] handle - radio transceiver handle
 */
void radioTxRxStart(radio_tx_rx_handle_t* handle);

/**
 * \brief Read the signal from each channel coming from the PPM input
 * \param[in]  handle - radio transceiver handle
 * \param[out] channels - signal values on each channel
 */
void radioTxRxReadInputs(radio_tx_rx_handle_t* handle, uint32_t channels[RADIO_TXRX_CHANNELS]);

#endif /* RADIO_TX_RX_H */