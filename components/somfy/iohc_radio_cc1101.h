#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC
#ifdef USE_SOMFY_IOHC_CC1101

#include "esphome/components/cc1101/cc1101.h"
#include "iohc_radio.h"

namespace esphome {
namespace somfy {

/// IohcRadio backend wrapping ESPHome's native cc1101 component.
class Cc1101IohcRadio : public IohcRadio, public cc1101::CC1101Listener {
 public:
  void set_cc1101(cc1101::CC1101Component *cc1101) { this->cc1101_ = cc1101; }

  void setup(IohcRadioListener *listener) override;
  void configure_1w() override;
  void configure_2w(uint8_t channel) override;
  bool transmit(const std::vector<uint8_t> &payload) override;
  void begin_rx() override;

  // CC1101Listener interface
  void on_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi, uint8_t lqi) override;

 protected:
  cc1101::CC1101Component *cc1101_{nullptr};
  IohcRadioListener *listener_{nullptr};
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC_CC1101
#endif  // USE_SOMFY_IOHC
