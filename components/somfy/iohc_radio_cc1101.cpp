#include "iohc_radio_cc1101.h"

#ifdef USE_SOMFY_IOHC
#ifdef USE_SOMFY_IOHC_CC1101

#include "iohc_protocol.h"
#include "somfy_hub_iohc.h"
#include "esphome/core/log.h"

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.iohc.radio.cc1101";

void Cc1101IohcRadio::setup(IohcRadioListener *listener) {
  this->listener_ = listener;
  this->cc1101_->register_listener(this);
}

void Cc1101IohcRadio::configure_1w() {
  this->cc1101_->set_frequency(iohc::FREQUENCY_1W);
  this->cc1101_->set_modulation_type(cc1101::Modulation::MODULATION_2_FSK);
  this->cc1101_->set_symbol_rate(iohc::SYMBOL_RATE);
  this->cc1101_->set_fsk_deviation(iohc::FSK_DEVIATION);
  this->cc1101_->set_filter_bandwidth(iohc::FILTER_BW);
  this->cc1101_->set_manchester(false);
  // The io-homecontrol sync (logical 0xFF 0x33) is UART-encoded on air; program
  // the hardware sync word to the first 16 encoded bits (0x7FD9). Hardware CRC
  // is off — the CRC-16 lives inside the logical frame and we verify it after
  // UART-decoding. Default to the fixed-length RX capture window.
  this->cc1101_->set_sync1(iohc_proto::PHY_HW_SYNC1);
  this->cc1101_->set_sync0(iohc_proto::PHY_HW_SYNC0);
  this->cc1101_->set_crc_enable(false);
  this->cc1101_->set_packet_length(iohc::RX_FIFO_WINDOW);
}

void Cc1101IohcRadio::configure_2w(uint8_t channel) {
  if (channel >= 3) channel = 0;
  this->cc1101_->set_frequency(iohc::FREQUENCY_2W[channel]);
}

bool Cc1101IohcRadio::transmit(const std::vector<uint8_t> &payload) {
  this->cc1101_->set_packet_length(static_cast<uint8_t>(payload.size()));
  auto err = this->cc1101_->transmit_packet(payload);
  this->cc1101_->set_packet_length(iohc::RX_FIFO_WINDOW);
  if (err != cc1101::CC1101Error::NONE) {
    ESP_LOGW(TAG, "TX error: %d", static_cast<int>(err));
    return false;
  }
  return true;
}

void Cc1101IohcRadio::begin_rx() { this->cc1101_->begin_rx(); }

void Cc1101IohcRadio::on_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi, uint8_t lqi) {
  this->listener_->on_iohc_packet(packet, rssi);
}

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC_CC1101
#endif  // USE_SOMFY_IOHC
