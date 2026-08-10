#include "iohc_radio_sx126x.h"

#ifdef USE_SOMFY_IOHC
#ifdef USE_SOMFY_IOHC_SX126X

#include "somfy_hub_iohc.h"
#include "esphome/core/log.h"

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.iohc.radio.sx126x";

void Sx126xIohcRadio::setup(IohcRadioListener *listener) {
  this->listener_ = listener;
  this->sx126x_->register_listener(this);
}

void Sx126xIohcRadio::configure_1w() {
  // Frequency, modulation, symbol rate, deviation, filter bandwidth, sync word
  // and the fixed 60-byte packet length are all set once at compile time via
  // the `sx126x:` YAML block (see somfy_iohc_1w_sx1262.yaml) — sx126x's
  // configure() is too heavy (image calibration, full modem re-init) to call
  // on every TX the way the cc1101 backend re-asserts settings.
}

void Sx126xIohcRadio::configure_2w(uint8_t channel) {
  ESP_LOGE(TAG, "2W mode is not supported with the sx126x radio backend");
}

bool Sx126xIohcRadio::transmit(const std::vector<uint8_t> &payload) {
  // sx126x is configured with a fixed payload_length (iohc::RX_FIFO_WINDOW)
  // shared by TX and RX, so transmit_packet() requires an exact-size match.
  // Real io-homecontrol frames are always shorter than that window and carry
  // no on-air length prefix, so the tail has to be filled with something.
  //
  // Pad with 0xFF, not 0x00. The physical layer is UART 8N1 (see
  // iohc_proto::uart_encode) whose idle level is a continuous 1, which is what
  // uart_encode itself uses to pad its final partial byte — and what real
  // remotes put on air (their captured tails run 0xFF/0xAA/0x55). A run of
  // 0x00 is 8 consecutive start-bit-shaped transitions per byte, which a
  // receiver doing genuine UART framing can latch onto as a stream of framing
  // errors right after the frame it was supposed to accept. Our own RX path
  // stops at the ctrl0 length field and so never noticed the difference.
  std::vector<uint8_t> padded = payload;
  padded.resize(iohc::RX_FIFO_WINDOW, 0xFF);

  auto err = this->sx126x_->transmit_packet(padded);
  if (err != sx126x::SX126xError::NONE) {
    ESP_LOGW(TAG, "TX error: %d", static_cast<int>(err));
    return false;
  }
  return true;
}

void Sx126xIohcRadio::begin_rx() { this->sx126x_->set_mode_rx(); }

void Sx126xIohcRadio::on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) {
  this->listener_->on_iohc_packet(packet, rssi);
}

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC_SX126X
#endif  // USE_SOMFY_IOHC
