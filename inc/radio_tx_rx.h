/**
 * \file radio_tx_rx.h
 * \author Mav Cuyugan
 *
 * This module is responsible for decoding the PPM input coming from the radio transceiver
 */

#ifndef RADIO_TX_RX_H
#define RADIO_TX_RX_H

#include "hal.h"

typedef enum
{
  RADIO_TXRX_UNINIT = 0,
  RADIO_TXRX_STOP,
  RADIO_TXRX_WAITING,
  RADIO_TXRX_ACTIVE
} radio_tx_rx_state_t;

typedef enum
{
  RADIO_TXRX_CHAN0 = 0,
  RADIO_TXRX_CHAN1,
  RADIO_TXRX_CHAN2,
  RADIO_TXRX_CHAN3,
  RADIO_TXRX_CHAN4,
  RADIO_TXRX_CHAN5,
  RADIO_TXRX_CHANNELS
} radio_tx_rx_channel_t;

typedef struct
{
  radio_tx_rx_state_t state;
  radio_tx_rx_channel_t incoming_channel;
  volatile icucnt_t channels[RADIO_TXRX_CHANNELS];
  mutex_t lock;
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

#endif /* RADIO_TX_RX_H */