#include "Comms.h"

Comms::Comms()
    : spi_slave(MOSI1, MISO1, SCK1, SSEL1),
      rx_dma1(nullptr),
      rx_dma2(nullptr),
      tx_dma1(nullptr),
      tx_dma2(nullptr),
      rx_memcpy_dma1(nullptr),
      rx_memcpy_dma2(nullptr),
      temp_rx_buffer1(),
      temp_rx_buffer2(),
      reject_count(0),
      data_ready(false),
      spi_error(false),
      pru_reset(false) {
  this->ptr_rx_data = new rxData_t();
  this->ptr_tx_data = new txData_t();
}

void Comms::init() {
  // Create MODDMA configuration objects for the SPI transfer and memory copy
  rx_memcpy_dma1 = new MODDMA_Config;
  rx_memcpy_dma2 = new MODDMA_Config;
  tx_dma1 = new MODDMA_Config;
  tx_dma2 = new MODDMA_Config;
  rx_dma1 = new MODDMA_Config;
  rx_dma2 = new MODDMA_Config;

  // Setup DMA configurations
  tx_dma1->channelNum(MODDMA::Channel_0)
      ->srcMemAddr(reinterpret_cast<uint32_t>(ptr_tx_data))
      ->dstMemAddr(0)
      ->transferSize(SPI_BUFF_SIZE)
      ->transferType(MODDMA::m2p)
      ->srcConn(0)
      ->dstConn(MODDMA::SSP1_Tx)
      ->attach_tc(this, &Comms::tx1_callback)
      ->attach_err(this, &Comms::err_callback);

  tx_dma2->channelNum(MODDMA::Channel_1)
      ->srcMemAddr(reinterpret_cast<uint32_t>(ptr_tx_data))
      ->dstMemAddr(0)
      ->transferSize(SPI_BUFF_SIZE)
      ->transferType(MODDMA::m2p)
      ->srcConn(0)
      ->dstConn(MODDMA::SSP1_Tx)
      ->attach_tc(this, &Comms::tx2_callback)
      ->attach_err(this, &Comms::err_callback);

  rx_dma1->channelNum(MODDMA::Channel_2)
      ->srcMemAddr(0)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&temp_rx_buffer1))
      ->transferSize(SPI_BUFF_SIZE)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP1_Rx)
      ->dstConn(0)
      ->attach_tc(this, &Comms::rx1_callback)
      ->attach_err(this, &Comms::err_callback);

  rx_dma2->channelNum(MODDMA::Channel_3)
      ->srcMemAddr(0)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&temp_rx_buffer2))
      ->transferSize(SPI_BUFF_SIZE)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP1_Rx)
      ->dstConn(0)
      ->attach_tc(this, &Comms::rx2_callback)
      ->attach_err(this, &Comms::err_callback);

  rx_memcpy_dma1->channelNum(MODDMA::Channel_4)
      ->srcMemAddr(reinterpret_cast<uint32_t>(&temp_rx_buffer1))
      ->dstMemAddr(reinterpret_cast<uint32_t>(ptr_rx_data))
      ->transferSize(SPI_BUFF_SIZE)
      ->transferType(MODDMA::m2m);

  rx_memcpy_dma2->channelNum(MODDMA::Channel_5)
      ->srcMemAddr(reinterpret_cast<uint32_t>(&temp_rx_buffer2))
      ->dstMemAddr(reinterpret_cast<uint32_t>(ptr_rx_data))
      ->transferSize(SPI_BUFF_SIZE)
      ->transferType(MODDMA::m2m);
}

void Comms::start() {
  NVIC_SetPriority(DMA_IRQn, 1);

  this->ptr_tx_data->header = PRU_DATA;

  // Pass the configurations to the controller
  dma.Prepare(rx_dma1);
  dma.Prepare(tx_dma1);

  // Enable SSP1 for DMA
  LPC_SSP1->DMACR = 0;
  LPC_SSP1->DMACR = (1 << 1) | (1 << 0);  // TX,RX DMA Enable
}

void Comms::tx1_callback() {
  // SPI Tx
  MODDMA_Config *config = dma.getConfig();
  dma.Disable(static_cast<MODDMA::CHANNELS>(config->channelNum()));

  // Clear DMA IRQ flags.
  if (dma.irqType() == MODDMA::TcIrq) dma.clearTcIrq();
  if (dma.irqType() == MODDMA::ErrIrq) dma.clearErrIrq();

  dma.Prepare(tx_dma2);
}

void Comms::tx2_callback() {
  // SPI Tx
  MODDMA_Config *config = dma.getConfig();
  dma.Disable(static_cast<MODDMA::CHANNELS>(config->channelNum()));

  // Clear DMA IRQ flags.
  if (dma.irqType() == MODDMA::TcIrq) dma.clearTcIrq();
  if (dma.irqType() == MODDMA::ErrIrq) dma.clearErrIrq();

  dma.Prepare(tx_dma1);
}

void Comms::rx1_callback() {
  // SPI Rx
  MODDMA_Config *config = dma.getConfig();
  dma.Disable(static_cast<MODDMA::CHANNELS>(config->channelNum()));

  data_ready = false;
  spi_error = false;

  // Clear DMA IRQ flags.
  if (dma.irqType() == MODDMA::TcIrq) dma.clearTcIrq();
  if (dma.irqType() == MODDMA::ErrIrq) dma.clearErrIrq();

  // Check and move the received SPI data payload
  switch (temp_rx_buffer1.header) {
    case PRU_READ:
      data_ready = true;
      reject_count = 0;
      dma.Disable(rx_memcpy_dma2->channelNum());
      break;

    case PRU_WRITE:
      data_ready = true;
      reject_count = 0;
      dma.Prepare(rx_memcpy_dma1);
      break;

    default:
      reject_count++;
      if (reject_count > 5) {
        spi_error = true;
      }
      dma.Disable(rx_memcpy_dma2->channelNum());
  }

  // swap Rx buffers
  dma.Prepare(rx_dma2);
}

void Comms::rx2_callback() {
  // SPI Rx
  MODDMA_Config *config = dma.getConfig();
  dma.Disable(static_cast<MODDMA::CHANNELS>(config->channelNum()));

  data_ready = false;
  spi_error = false;

  // Clear DMA IRQ flags.
  if (dma.irqType() == MODDMA::TcIrq) dma.clearTcIrq();
  if (dma.irqType() == MODDMA::ErrIrq) dma.clearErrIrq();

  // Check and move the recieved SPI data payload
  switch (temp_rx_buffer2.header) {
    case PRU_READ:
      data_ready = true;
      reject_count = 0;
      dma.Disable(rx_memcpy_dma1->channelNum());
      break;

    case PRU_WRITE:
      data_ready = true;
      reject_count = 0;
      dma.Prepare(rx_memcpy_dma2);
      break;

    default:
      reject_count++;
      if (reject_count > 5) {
        spi_error = true;
      }
      dma.Disable(rx_memcpy_dma1->channelNum());
  }

  // swap Rx buffers
  dma.Prepare(rx_dma1);
}

void Comms::err_callback() { printf("err\r\n"); }

bool Comms::get_status() const { return this->data_ready; }

void Comms::set_status(const bool status) { this->data_ready = status; }

bool Comms::get_error() const { return this->spi_error; }

void Comms::set_error(const bool error) { this->spi_error = error; }