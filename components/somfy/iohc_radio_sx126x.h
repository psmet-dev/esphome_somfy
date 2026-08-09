#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC
#ifdef USE_SOMFY_IOHC_SX126X

#include "esphome/components/sx126x/sx126x.h"
#include "iohc_radio.h"

namespace esphome {
namespace somfy {

/// IohcRadio backend wrapping ESPHome's native sx126x component (e.g. the
/// onboard SX1262 on boards like the Heltec WiFi LoRa 32 V3).
///
/// 1W only: sx126x's set_frequency() only updates a member variable — the
/// actual RF retune happens inside configure(), which also redoes packet-type
/// / PA / modem / packet-params setup (and image calibration on cold start).
/// That is far too slow to call every 2.7ms (iohc::CHANNEL_DWELL_US) for 2W's
/// 3-channel hop, and there is no lighter-weight public retune call. 1W never
/// hops, so it is unaffected. configure_2w() is a defensive no-op — the
/// somfy `cover` config validation rejects `mode: 2w` against an sx126x-backed
/// hub, so it should never actually be called.
class Sx126xIohcRadio : public IohcRadio, public sx126x::SX126xListener {
 public:
  void set_sx126x(sx126x::SX126x *sx126x) { this->sx126x_ = sx126x; }

  void setup(IohcRadioListener *listener) override;
  void configure_1w() override;
  void configure_2w(uint8_t channel) override;
  bool transmit(const std::vector<uint8_t> &payload) override;
  void begin_rx() override;

  // SX126xListener interface
  void on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) override;

 protected:
  sx126x::SX126x *sx126x_{nullptr};
  IohcRadioListener *listener_{nullptr};
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC_SX126X
#endif  // USE_SOMFY_IOHC
