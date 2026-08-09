#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC

#include <cstdint>
#include <vector>

namespace esphome {
namespace somfy {

/// Callback interface implemented by SomfyIohcHub to receive decoded RX packets
/// from whichever radio backend is in use.
class IohcRadioListener {
 public:
  virtual void on_iohc_packet(const std::vector<uint8_t> &packet, float rssi) = 0;
};

/// Radio backend interface: everything the iohc hub needs from the underlying
/// sub-GHz transceiver, independent of which chip drives it.
class IohcRadio {
 public:
  virtual ~IohcRadio() = default;

  // Register for RX callbacks and enter the initial state. Called once from
  // SomfyIohcHub::setup().
  virtual void setup(IohcRadioListener *listener) = 0;

  // Apply 1W radio parameters (fixed frequency, 2-FSK, io-homecontrol sync word).
  virtual void configure_1w() = 0;

  // Apply 2W radio parameters for the given hop channel (0-2).
  virtual void configure_2w(uint8_t channel) = 0;

  // Transmit a single on-air packet (already UART-encoded). Returns true on success.
  virtual bool transmit(const std::vector<uint8_t> &payload) = 0;

  // (Re-)enter RX mode, e.g. after a TX or a frequency change.
  virtual void begin_rx() = 0;
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
