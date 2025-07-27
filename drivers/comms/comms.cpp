#include "comms.h"

Comms::Comms() : spi_slave(MOSI1, MISO1, SCK1, SSEL1) {
  if (sizeof(rx_data) != sizeof(tx_data)) {
    error("SPI buffer size mismatch, rx: %u, tx: %u", sizeof(rx_data), sizeof(tx_data));
  }

  tx_dma = new MODDMA_Config;
  rx_dma = new MODDMA_Config;

  tx_dma->channelNum(MODDMA::Channel_0)
      ->srcMemAddr(reinterpret_cast<uint32_t>(&tx_data))
      ->dstMemAddr(0)
      ->transferSize(sizeof(tx_data))
      ->transferType(MODDMA::m2p)
      ->srcConn(0)
      ->dstConn(MODDMA::SSP1_Tx)
      ->attach_tc(this, &Comms::tx_callback)
      ->attach_err(this, &Comms::err_callback);

  rx_dma->channelNum(MODDMA::Channel_2)
      ->srcMemAddr(0)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&rx_temp_buffer))
      ->transferSize(sizeof(rx_temp_buffer))
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP1_Rx)
      ->dstConn(0)
      ->attach_tc(this, &Comms::rx_callback)
      ->attach_err(this, &Comms::err_callback);
}

void Comms::start() {
  NVIC_SetPriority(DMA_IRQn, 1);

  tx_data.header = SPI_DATA_HEADER;
  tx_data.footer = SPI_DATA_FOOTER;

  dma.Prepare(rx_dma);
  dma.Prepare(tx_dma);

  // Enable SSP1 for DMA
  LPC_SSP1->DMACR = 0;
  LPC_SSP1->DMACR = (1 << 1) | (1 << 0);  // TX,RX DMA Enable
}

void Comms::rx_callback() {
  // upon receiving data
  // disable receive DMA and clear interrupt flags
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();

  // printf("received %d bytes: ", sizeof(rx_temp_buffer));
  // auto buf = reinterpret_cast<uint8_t*>(&rx_temp_buffer);
  // for (unsigned int i = 0; i < sizeof(rx_temp_buffer); i++) {
  //   printf("%02X ", buf[i]);
  // }
  // printf("\ntransmitting %d bytes: ", sizeof(tx_temp_buffer));
  // buf = reinterpret_cast<uint8_t*>(&tx_temp_buffer);
  // for (unsigned int i = 0; i < sizeof(tx_temp_buffer); i++) {
  //   printf("%02X ", buf[i]);
  // }
  // printf("\n");

  if (rx_temp_buffer.header == SPI_DATA_HEADER && rx_temp_buffer.footer == SPI_DATA_FOOTER) {
    // if receive is successful:
    // copy data from the temp receive buffer to the actual one
    // it's only 40 bytes or something, so DMA would only add overhead
    // memcpy is not possible on volatile structures, so do it the long way
    // we're also skipping the header and the footer
    for (size_t i = 1; i < sizeof(rx_temp_buffer) / 4 - 1; i++) {
      reinterpret_cast<volatile uint32_t*>(&rx_data)[i] = reinterpret_cast<volatile uint32_t*>(&rx_temp_buffer)[i];
    }
    // clear garbage counter
    reject_cnt = 0;
  } else {
    // if we've received garbage:
    // bump garbage counter
    reject_cnt++;
    if (reject_cnt > 5) {
      // too much garbage - signal to the main thread and return without attempting more transfers
      comms_error = true;
      return;
    }
  }
  // attempt another transfer
  dma.Prepare(rx_dma);
}

void Comms::tx_callback() {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();

  dma.Prepare(tx_dma);
}

void Comms::err_callback() {
  printf("DMA error on channel %d!\n", dma.irqProcessingChannel());
  this->comms_error = true;
  dma.Disable(dma.irqProcessingChannel());
  dma.clearErrIrq();
}

bool Comms::get_error() const { return this->comms_error; }
