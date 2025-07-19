#include "comms.h"

Comms::Comms() : spi_slave(MOSI1, MISO1, SCK1, SSEL1) {
  if (sizeof(rx_data) != sizeof(tx_data)) {
    error("SPI buffer size mismatch, rx: %lu, tx: %lu", sizeof(rx_data), sizeof(tx_data));
  }
  this->spi_slave.frequency(48000000);

  tx_dma = new MODDMA_Config;
  rx_dma = new MODDMA_Config;

  tx_dma->channelNum(MODDMA::Channel_0)
      ->srcMemAddr(reinterpret_cast<uint32_t>(ptr_tx_data))
      ->dstMemAddr(0)
      ->transferSize(sizeof(tx_data))
      ->transferType(MODDMA::m2p)
      ->srcConn(0)
      ->dstConn(MODDMA::SSP1_Tx)
      ->attach_err(&Comms::err_callback);

  rx_dma->channelNum(MODDMA::Channel_2)
      ->srcMemAddr(0)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&rx_temp_buffer))
      ->transferSize(sizeof(rx_temp_buffer))
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP1_Rx)
      ->dstConn(0)
      ->attach_tc(this, &Comms::rx_callback)
      ->attach_err(&Comms::err_callback);
}

void Comms::start() {
  NVIC_SetPriority(DMA_IRQn, 1);

  memcpy(&tx_temp_buffer, &tx_data, sizeof(tx_data));

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
  if (dma.irqType() == MODDMA::TcIrq) dma.clearTcIrq();
  if (dma.irqType() == MODDMA::ErrIrq) dma.clearErrIrq();

  // copy tx_data to a temp buffer and initiate the tx DMA
  memcpy(&tx_temp_buffer, &tx_data, sizeof(tx_data));
  dma.Prepare(tx_dma);

  comms_ready = false;
  comms_error = false;

  if (rx_temp_buffer.header == SPI_DATA_HEADER && rx_temp_buffer.footer == SPI_DATA_FOOTER) {
    // if receive is successful:
    // copy data from the temp receive buffer to the actual one
    memcpy(&rx_data, &rx_temp_buffer, sizeof(rx_temp_buffer));
    // signal to the main thread that there's data
    comms_ready = true;
    // clear garbage counter
    reject_cnt = 0;
  } else {
    // if we've received garbage:
    // bump garbage counter
    reject_cnt++;
    if (reject_cnt > 5) {
      // too much garbage - signal to the main thread
      comms_error = true;
    }
  }
  // attempt another transfer
  dma.Prepare(rx_dma);
}

void Comms::err_callback() { printf("err\r\n"); }

bool Comms::is_ready() const { return this->comms_ready; }

void Comms::not_ready() { this->comms_ready = false; }

bool Comms::get_error() const { return this->comms_error; }

void Comms::clear_error() { this->comms_error = false; }
