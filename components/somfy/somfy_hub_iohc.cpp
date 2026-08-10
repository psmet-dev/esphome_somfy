#include "somfy_hub_iohc.h"

#ifdef USE_SOMFY_IOHC

#include "iohc_protocol.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cinttypes>
#include <cstring>
// AES on ESP32/ESP-IDF needs the IDF mbedTLS config selected before the header.
#define MBEDTLS_CONFIG_FILE "mbedtls/esp_config.h"
#include <mbedtls/aes.h>

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.iohc.hub";

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

uint16_t crc16_kermit(const uint8_t *data, size_t len) {
  return iohc_proto::crc16(data, len);
}

void aes128_ecb_encrypt(const uint8_t key[16], const uint8_t plaintext[16], uint8_t ciphertext[16]) {
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  mbedtls_aes_setkey_enc(&ctx, key, 128);
  mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, plaintext, ciphertext);
  mbedtls_aes_free(&ctx);
}

void compute_2w_response(const uint8_t key[16], const uint8_t *frame_data, size_t frame_len,
                         const uint8_t challenge[6], uint8_t response[6]) {
  // frame_data is the authenticated payload cmd || data. Build the 2W IV and
  // take the leading 6 bytes of AES(key, IV) as the response.
  uint8_t iv[16];
  iohc_proto::build_iv_2w(frame_data, frame_len, challenge, iv);
  iohc_proto::compute_mac(aes128_ecb_encrypt, key, iv, response);
}

// ---------------------------------------------------------------------------
// Setup / Loop / Dump
// ---------------------------------------------------------------------------

void SomfyIohcHub::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Somfy iohc hub...");
  this->configure_radio_1w();
  this->radio_->setup(this);
  // Enter RX immediately so we can hear physical io-homecontrol remotes (and 2W
  // feedback) from boot — the radio does not auto-listen, and otherwise RX
  // would only start after the first HA-initiated TX.
  this->radio_->begin_rx();
}

void SomfyIohcHub::loop() {
  // 2W frequency hopping
  if (this->listening_2w_) {
    const uint32_t now_us = micros();
    if ((now_us - this->last_hop_us_) >= iohc::CHANNEL_DWELL_US) {
      this->current_2w_channel_ = (this->current_2w_channel_ + 1) % 3;
      this->configure_radio_2w(this->current_2w_channel_);
      this->radio_->begin_rx();  // re-enter RX after frequency change
      this->last_hop_us_ = now_us;
    }
  }

  // 2W session state machine
  this->session_loop_();
}

void SomfyIohcHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Somfy iohc Hub:");
  ESP_LOGCONFIG(TAG, "  Radio: %s", this->radio_ != nullptr ? "configured" : "MISSING");
  ESP_LOGCONFIG(TAG, "  RX callbacks: %u", this->rx_callbacks_.size());
}

// ---------------------------------------------------------------------------
// TX (1W)
// ---------------------------------------------------------------------------

void SomfyIohcHub::transmit_packet(const std::vector<uint8_t> &frame, uint8_t repeat_count,
                                   bool mark_first_of_burst) {
  this->configure_radio_1w();

  // Wrap the logical frame in the io-homecontrol UART-8N1 physical encoding and
  // hand the radio a fixed-length packet (no variable-length prefix byte goes
  // on air). The hardware sync word (0x7FD9) supplies the leading 16 sync bits;
  // the codec emits the remaining 4 sync bits + the UART-framed frame.
  auto &payload = this->tx_payload_;
  iohc_proto::uart_encode(frame.data(), frame.size(), payload);

  ESP_LOGD(TAG, "TX 1W: %u logical / %u on-air bytes, %d repeats", static_cast<unsigned>(frame.size()),
           static_cast<unsigned>(payload.size()), repeat_count);
  ESP_LOGV(TAG, "TX 1W logical: %s", format_hex_pretty(frame).c_str());
  ESP_LOGV(TAG, "TX 1W on-air: %s", format_hex_pretty(payload).c_str());

  for (int i = 0; i < repeat_count; i++) {
    if (i == 1 && mark_first_of_burst) {
      // A real remote sets CtrlByte1 bit 0x20 on the first transmission of a
      // burst and clears it on the repeats -- captured live on seq 0x04B3 and
      // 0x04EE, which each appear twice differing in exactly this byte. The MAC
      // doesn't cover ctrl1 (it authenticates cmd || data only), so only the
      // CRC has to be recomputed. Re-encode once here and reuse it for the
      // remaining repeats.
      auto repeat_frame = frame;
      repeat_frame[1] = static_cast<uint8_t>(repeat_frame[1] & ~0x20);
      const uint16_t crc = crc16_kermit(repeat_frame.data(), repeat_frame.size() - 2);
      repeat_frame[repeat_frame.size() - 2] = static_cast<uint8_t>(crc & 0xFF);
      repeat_frame[repeat_frame.size() - 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
      iohc_proto::uart_encode(repeat_frame.data(), repeat_frame.size(), payload);
    }
    if (!this->radio_->transmit(payload)) {
      ESP_LOGW(TAG, "TX error on repeat %d", i);
      break;
    }
  }

  this->radio_->begin_rx();
}

void SomfyIohcHub::begin_rx() {
  this->radio_->begin_rx();
}

// ---------------------------------------------------------------------------
// Radio configuration
// ---------------------------------------------------------------------------

void SomfyIohcHub::configure_radio_1w() {
  this->radio_->configure_1w();
  this->listening_2w_ = false;
}

void SomfyIohcHub::configure_radio_2w(uint8_t channel) {
  if (channel >= 3) channel = 0;
  this->radio_->configure_2w(channel);
}

void SomfyIohcHub::start_2w_listen() {
  this->current_2w_channel_ = 0;
  this->configure_radio_2w(0);
  this->radio_->begin_rx();
  this->listening_2w_ = true;
  this->last_hop_us_ = micros();
}

void SomfyIohcHub::stop_2w_listen() {
  this->listening_2w_ = false;
  this->configure_radio_1w();
  // Resume 1W RX so passive state-sync keeps working after the 2W session.
  this->radio_->begin_rx();
}

// ---------------------------------------------------------------------------
// 2W TX: send command with challenge/response auth
// ---------------------------------------------------------------------------

void SomfyIohcHub::send_2w_command(uint32_t src_node, uint32_t dest_node, uint8_t cmd,
                                    const uint8_t *data, size_t data_len,
                                    const uint8_t key[16], Session2WCallback callback) {
  if (this->session_.state != Session2WState::IDLE &&
      this->session_.state != Session2WState::COMPLETE &&
      this->session_.state != Session2WState::FAILED) {
    ESP_LOGW(TAG, "2W session busy, cannot start new command");
    if (callback) callback(false, nullptr);
    return;
  }

  // Initialize session
  this->session_.state = Session2WState::CMD_SENT;
  this->session_.src_node = src_node;
  this->session_.dest_node = dest_node;
  this->session_.cmd = cmd;
  this->session_.cmd_data.assign(data, data + data_len);
  memcpy(this->session_.key, key, 16);
  this->session_.started_ms = millis();
  this->session_.retries = 0;
  this->session_.callback = std::move(callback);

  // Build and store the authenticated payload (cmd || data) for the challenge
  // response MAC. Node addresses are not part of the MAC input.
  this->session_.frame_payload.clear();
  this->session_.frame_payload.reserve(1 + data_len);
  this->session_.frame_payload.push_back(cmd);
  this->session_.frame_payload.insert(this->session_.frame_payload.end(), data, data + data_len);

  // Switch to 2W mode and send the command frame
  this->start_2w_listen();
  this->send_2w_frame_(src_node, dest_node, cmd, data, data_len);

  ESP_LOGD(TAG, "2W session started: src=0x%06" PRIX32 " dst=0x%06" PRIX32 " cmd=0x%02X", src_node, dest_node, cmd);
}

// ---------------------------------------------------------------------------
// 2W session loop (timeout management)
// ---------------------------------------------------------------------------

void SomfyIohcHub::session_loop_() {
  if (this->session_.state == Session2WState::IDLE ||
      this->session_.state == Session2WState::COMPLETE ||
      this->session_.state == Session2WState::FAILED)
    return;

  uint32_t elapsed = millis() - this->session_.started_ms;
  if (elapsed > iohc::SESSION_TIMEOUT_MS) {
    if (this->session_.retries < iohc::SESSION_MAX_RETRIES) {
      this->session_.retries++;
      this->session_.started_ms = millis();
      this->session_.state = Session2WState::CMD_SENT;
      ESP_LOGW(TAG, "2W session timeout, retry %d", this->session_.retries);
      this->send_2w_frame_(this->session_.src_node, this->session_.dest_node,
                           this->session_.cmd, this->session_.cmd_data.data(),
                           this->session_.cmd_data.size());
    } else {
      ESP_LOGW(TAG, "2W session failed after %d retries", this->session_.retries);
      this->session_.state = Session2WState::FAILED;
      this->stop_2w_listen();
      if (this->session_.callback) this->session_.callback(false, nullptr);
    }
  }
}

// ---------------------------------------------------------------------------
// 2W frame building and sending
// ---------------------------------------------------------------------------

std::vector<uint8_t> SomfyIohcHub::build_2w_frame_(uint32_t src, uint32_t dest, uint8_t cmd,
                                                     const uint8_t *data, size_t data_len) {
  std::vector<uint8_t> frame;
  frame.reserve(9 + data_len + 2);  // header + data + CRC

  // CtrlByte0 placeholder — order=00, isOneWay=0; the 5-bit size field is filled
  // in once the body (ctrl1..data) is assembled.
  frame.push_back(0x00);

  // CtrlByte1: 2W frames carry no Start/End framing bits (0x00).
  frame.push_back(iohc::CTRL1_2W);

  // Destination (3 bytes big-endian)
  frame.push_back(static_cast<uint8_t>((dest >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((dest >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(dest & 0xFF));

  // Source (3 bytes big-endian)
  frame.push_back(static_cast<uint8_t>((src >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((src >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(src & 0xFF));

  // Command
  frame.push_back(cmd);

  // Data
  for (size_t i = 0; i < data_len; i++) {
    frame.push_back(data[i]);
  }

  // CtrlByte0 size field = body length (everything after ctrl0, excluding the
  // trailing CRC) masked to 5 bits.
  const size_t size_field = frame.size() - 1;
  frame[0] = static_cast<uint8_t>(size_field & 0x1F);

  // CRC-16-KERMIT
  uint16_t crc = crc16_kermit(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  return frame;
}

void SomfyIohcHub::send_2w_frame_(uint32_t src, uint32_t dest, uint8_t cmd,
                                   const uint8_t *data, size_t data_len) {
  auto frame = this->build_2w_frame_(src, dest, cmd, data, data_len);
  // 2W frames are sent once on the current channel (868.95 MHz = ch1)
  this->configure_radio_2w(1);
  // Apply the same UART-8N1 physical encoding + fixed-length packet as 1W.
  auto &payload = this->tx_payload_;
  iohc_proto::uart_encode(frame.data(), frame.size(), payload);
  if (!this->radio_->transmit(payload)) {
    ESP_LOGW(TAG, "2W TX error");
  }
  this->radio_->begin_rx();
  ESP_LOGD(TAG, "TX 2W: cmd=0x%02X %u logical / %u on-air bytes", cmd, static_cast<unsigned>(frame.size()),
           static_cast<unsigned>(payload.size()));
}

// ---------------------------------------------------------------------------
// RX callback
// ---------------------------------------------------------------------------

void SomfyIohcHub::on_iohc_packet(const std::vector<uint8_t> &raw, float rssi) {
  // The radio captures a fixed-size window of raw on-air bytes after the
  // hardware sync match (0x7FD9). Strip the io-homecontrol UART 8N1 framing to
  // recover the logical frame bytes (this is what the documented captures show).
  auto &packet = this->rx_frame_;
  iohc_proto::uart_decode(raw.data(), raw.size(), packet);

  // ctrl0 low 5 bits = frame length excluding ctrl0 and the trailing 2-byte
  // CRC. Use it to drop any noise the fixed-length capture decoded past the
  // real frame, so the CRC residue check sees exactly the frame.
  if (packet.size() < 3) return;
  const size_t frame_len = 1 + (packet[0] & 0x1F) + 2;
  if (packet.size() < frame_len) return;  // truncated / undecodable capture
  packet.resize(frame_len);

  if (packet.size() < 11) return;  // minimum valid frame

  // Verify CRC
  uint16_t received_crc = static_cast<uint16_t>(packet[packet.size() - 2]) |
                          (static_cast<uint16_t>(packet[packet.size() - 1]) << 8);
  uint16_t calc_crc = crc16_kermit(packet.data(), packet.size() - 2);
  if (received_crc != calc_crc) {
    ESP_LOGV(TAG, "RX: CRC mismatch (got 0x%04X, calc 0x%04X)", received_crc, calc_crc);
    return;
  }

  // Parse header
  IohcDecodedPacket pkt;
  pkt.dest_node = (static_cast<uint32_t>(packet[2]) << 16) |
                  (static_cast<uint32_t>(packet[3]) << 8) |
                  static_cast<uint32_t>(packet[4]);
  pkt.src_node = (static_cast<uint32_t>(packet[5]) << 16) |
                 (static_cast<uint32_t>(packet[6]) << 8) |
                 static_cast<uint32_t>(packet[7]);
  pkt.cmd = packet[8];
  pkt.data = (packet.size() > 11) ? &packet[9] : nullptr;
  pkt.data_len = (packet.size() > 11) ? packet.size() - 11 : 0;  // 9 header + 2 CRC
  pkt.rssi = rssi;

  // Log the decoded logical frame, not just the header. The raw `Data:` dump an
  // sx126x on_packet lambda prints is the UART-encoded 60-byte window and has to
  // be run back through uart_decode before it means anything; this line is the
  // frame itself (ctrl0..CRC) and can be fed straight back to send_raw_frame().
  ESP_LOGD(TAG, "RX: src=0x%06" PRIX32 " dst=0x%06" PRIX32 " cmd=0x%02X rssi=%.1f len=%u frame=%s",
           pkt.src_node, pkt.dest_node, pkt.cmd, rssi, static_cast<unsigned>(packet.size()),
           format_hex(packet).c_str());

  // Handle 2W session packets
  this->handle_2w_packet_(pkt);

  // Dispatch to all registered callbacks
  for (auto &cb : this->rx_callbacks_) {
    cb(pkt);
  }
}

// ---------------------------------------------------------------------------
// 2W session packet handler (challenge/response state machine)
// ---------------------------------------------------------------------------

void SomfyIohcHub::handle_2w_packet_(const IohcDecodedPacket &pkt) {
  // Only process if we have an active session and packet is from our target
  if (this->session_.state == Session2WState::IDLE ||
      this->session_.state == Session2WState::COMPLETE ||
      this->session_.state == Session2WState::FAILED)
    return;

  if (pkt.src_node != this->session_.dest_node)
    return;
  if (pkt.dest_node != this->session_.src_node && pkt.dest_node != iohc::BROADCAST_ADDR)
    return;

  switch (this->session_.state) {
    case Session2WState::CMD_SENT:
      // Expecting 0x3C (Challenge Request) from actuator
      if (pkt.cmd == iohc::CMD_CHALLENGE_REQUEST && pkt.data_len >= 6) {
        memcpy(this->session_.challenge, pkt.data, 6);
        ESP_LOGD(TAG, "2W: Challenge received from 0x%06" PRIX32, pkt.src_node);

        // Compute response from the original frame payload (dest+src+cmd+data)
        uint8_t response[6];
        compute_2w_response(this->session_.key, this->session_.frame_payload.data(),
                           this->session_.frame_payload.size(), this->session_.challenge, response);

        // Send 0x3D (Challenge Response)
        this->session_.state = Session2WState::WAIT_STATUS;
        this->send_2w_frame_(this->session_.src_node, this->session_.dest_node,
                            iohc::CMD_CHALLENGE_RESPONSE, response, 6);
        ESP_LOGD(TAG, "2W: Challenge response sent");
      }
      break;

    case Session2WState::WAIT_STATUS:
      // Expecting status response (0xFE) or private ACK (0x21)
      if (pkt.cmd == iohc::CMD_STATUS || pkt.cmd == iohc::CMD_PRIVATE_ACK) {
        ESP_LOGD(TAG, "2W: Session complete, got cmd=0x%02X", pkt.cmd);
        this->session_.state = Session2WState::COMPLETE;
        this->stop_2w_listen();
        if (this->session_.callback) this->session_.callback(true, &pkt);
      }
      break;

    default:
      break;
  }
}

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
