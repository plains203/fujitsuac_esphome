#include "fujitsu_ac.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <cstring>

namespace esphome {
namespace fujitsu_ac {

static const char *const TAG = "fujitsu_ac";

// Poll cadence and timeouts (matches upstream dongle: one frame every 400 ms)
static const uint32_t FRAME_INTERVAL_MS = 400;
static const uint32_t RESPONSE_TIMEOUT_MS = 600;
static const uint32_t RX_GAP_RESET_MS = 100;
static const uint8_t TIMEOUTS_BEFORE_REINIT = 8;

// Fixed handshake frames + expected replies (from TFSXW1Controller)
static const uint8_t INIT1_PAYLOAD[4] = {0x00, 0x00, 0x00, 0x00};
static const uint8_t INIT2_PAYLOAD[4] = {0x00, 0x04, 0x00, 0x01};
static const uint8_t INIT1_EXPECT[8] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFD};
static const uint8_t INIT2_EXPECT[8] = {0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFC};
// Frames the indoor unit emits right after (re)booting; ignore and keep waiting.
static const uint8_t RESTART_FRAMES[2][8] = {
    {0xFE, 0x00, 0x00, 0x00, 0x01, 0x02, 0xFE, 0xFE},
    {0xFC, 0x00, 0x00, 0x00, 0x01, 0x02, 0xFF, 0x00},
};

// Poll groups — kept byte-identical to the upstream dongle's frames so we
// exercise exactly the traffic pattern the AC firmware has been tested with.
static const uint16_t IREG1[] = {0x0001, 0x0101};
static const uint16_t IREG2[] = {0x0110, 0x0111, 0x0112, 0x0113, 0x0114, 0x0115, 0x0117,
                                 0x011A, 0x011D, 0x0120, REG_V_AIRFLOW_COUNT, REG_V_SWING_SUPPORTED,
                                 REG_H_AIRFLOW_COUNT, REG_H_SWING_SUPPORTED};
static const uint16_t IREG3[] = {REG_ECONOMY_SUPPORTED, REG_MIN_HEAT_SUPPORTED, REG_HUMAN_SENSOR_SUPPORTED,
                                 REG_ESF_SUPPORTED, 0x0154, 0x0155, 0x0156, REG_POWERFUL_SUPPORTED,
                                 REG_OULN_SUPPORTED, REG_COIL_DRY_SUPPORTED};
static const uint16_t FRAME_A[] = {REG_POWER, REG_MODE, REG_SETPOINT, REG_FAN_SPEED,
                                   REG_V_AIRFLOW_SET, REG_V_SWING, REG_V_AIRFLOW,
                                   REG_H_AIRFLOW_SET, REG_H_SWING, REG_H_AIRFLOW,
                                   0x1031, REG_ACTUAL_TEMP, 0x1034};
static const uint16_t FRAME_B[] = {REG_ECONOMY, REG_MIN_HEAT, REG_HUMAN_SENSOR, 0x1103, 0x1104,
                                   0x1105, 0x1106, 0x1107, REG_ENERGY_SAVING_FAN, 0x1109,
                                   REG_POWERFUL, REG_OUTDOOR_LOW_NOISE, REG_COIL_DRY, 0x1200,
                                   0x1201, 0x1202, 0x1203, 0x1204, 0x1141};
static const uint16_t FRAME_C[] = {0x1400, 0x1401, 0x1402, 0x1403, 0x1404, 0x1405,
                                   0x1406, 0x140E, 0x2000, REG_OUTDOOR_TEMP, 0x2021, 0xF001};

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

// ---------------------------------------------------------------------------
// Component lifecycle
// ---------------------------------------------------------------------------
void FujitsuAC::setup() {
  this->last_tx_ms_ = millis();
  this->response_received_ = true;
  this->last_sent_ = LastSent::NONE;
}

void FujitsuAC::loop() {
  this->handle_rx_();
  this->tick_();
}

void FujitsuAC::dump_config() {
  ESP_LOGCONFIG(TAG, "Fujitsu AC (TFSXW1 protocol):");
  ESP_LOGCONFIG(TAG, "  Connected: %s", YESNO(this->phase_ == Phase::RUN));
  ESP_LOGCONFIG(TAG, "  Minimum-heat writes enabled: %s", YESNO(this->allow_min_heat_write_));
  if (this->init_regs_done_) {
    ESP_LOGCONFIG(TAG, "  Vertical louvre positions: %u (swing %s)", this->get_reg_(REG_V_AIRFLOW_COUNT),
                  YESNO(this->feature_supported_(REG_V_SWING_SUPPORTED)));
    ESP_LOGCONFIG(TAG, "  Horizontal louvre positions: %u (swing %s)", this->get_reg_(REG_H_AIRFLOW_COUNT),
                  YESNO(this->feature_supported_(REG_H_SWING_SUPPORTED)));
  }
  this->check_uart_settings(9600);
}

// ---------------------------------------------------------------------------
// TX state machine
// ---------------------------------------------------------------------------
void FujitsuAC::tick_() {
  const uint32_t now = millis();

  // Response timeout handling
  if (!this->response_received_ && (now - this->last_tx_ms_) >= RESPONSE_TIMEOUT_MS) {
    this->response_received_ = true;

    if (this->last_sent_ == LastSent::INIT1 || this->last_sent_ == LastSent::INIT2) {
      // Handshake not established yet — quietly keep retrying from INIT1.
      this->last_sent_ = LastSent::NONE;
    } else {
      this->timeout_count_++;
      if (!this->comm_lost_logged_) {
        ESP_LOGW(TAG, "No response from AC within %" PRIu32 " ms (frame %u)", RESPONSE_TIMEOUT_MS,
                 (unsigned) this->last_sent_);
        this->comm_lost_logged_ = true;
      }
      if (this->timeout_count_ >= TIMEOUTS_BEFORE_REINIT) {
        ESP_LOGW(TAG, "Communication lost, restarting handshake");
        this->phase_ = Phase::HANDSHAKE;
        this->init_regs_done_ = false;
        this->last_sent_ = LastSent::NONE;
        this->timeout_count_ = 0;
      }
      // Otherwise fall through: state machine continues from last_sent_,
      // re-polling the cycle. Pending writes are preserved (only cleared on ack).
    }
  }

  if (!this->response_received_ || (now - this->last_tx_ms_) < FRAME_INTERVAL_MS)
    return;

  switch (this->last_sent_) {
    case LastSent::NONE:
      this->last_sent_ = LastSent::INIT1;
      ESP_LOGD(TAG, "Handshake: sending Init1");
      this->send_frame_(0x00, INIT1_PAYLOAD, sizeof(INIT1_PAYLOAD));
      break;

    case LastSent::INIT1:
      this->last_sent_ = LastSent::INIT2;
      ESP_LOGD(TAG, "Handshake: sending Init2");
      this->send_frame_(0x01, INIT2_PAYLOAD, sizeof(INIT2_PAYLOAD));
      break;

    case LastSent::INIT2:
      this->send_read_(LastSent::IREG1, IREG1, ARRAY_LEN(IREG1));
      break;
    case LastSent::IREG1:
      this->send_read_(LastSent::IREG2, IREG2, ARRAY_LEN(IREG2));
      break;
    case LastSent::IREG2:
      this->send_read_(LastSent::IREG3, IREG3, ARRAY_LEN(IREG3));
      break;

    case LastSent::IREG3:
      this->init_regs_done_ = true;
      this->phase_ = Phase::RUN;
      ESP_LOGI(TAG, "Handshake complete, polling started");
      this->send_read_(LastSent::FRAME_A, FRAME_A, ARRAY_LEN(FRAME_A));
      break;

    case LastSent::FRAME_C:
      this->send_read_(LastSent::FRAME_A, FRAME_A, ARRAY_LEN(FRAME_A));
      break;

    case LastSent::FRAME_A:
      if (!this->pending_writes_.empty()) {
        this->send_write_frame_();
      } else {
        this->send_read_(LastSent::FRAME_B, FRAME_B, ARRAY_LEN(FRAME_B));
      }
      break;

    case LastSent::WRITE:
      if (!this->verify_addrs_.empty()) {
        // Read back what we just wrote so entity state updates immediately.
        this->send_read_(LastSent::VERIFY, this->verify_addrs_.data(), this->verify_addrs_.size());
        this->verify_addrs_.clear();
      } else {
        this->send_read_(LastSent::FRAME_B, FRAME_B, ARRAY_LEN(FRAME_B));
      }
      break;

    case LastSent::VERIFY:
      this->send_read_(LastSent::FRAME_B, FRAME_B, ARRAY_LEN(FRAME_B));
      break;

    case LastSent::FRAME_B:
      this->send_read_(LastSent::FRAME_C, FRAME_C, ARRAY_LEN(FRAME_C));
      break;
  }
}

void FujitsuAC::send_frame_(uint8_t type, const uint8_t *payload, size_t payload_len) {
  uint8_t buf[96];
  if (payload_len + 7 > sizeof(buf)) {
    ESP_LOGE(TAG, "TX frame too large (%u bytes)", (unsigned) (payload_len + 7));
    return;
  }

  buf[0] = type;
  buf[1] = buf[2] = buf[3] = 0x00;
  buf[4] = (uint8_t) payload_len;
  memcpy(buf + 5, payload, payload_len);

  uint16_t checksum = 0xFFFF;
  for (size_t i = 0; i < 5 + payload_len; i++)
    checksum -= buf[i];
  buf[5 + payload_len] = checksum >> 8;
  buf[6 + payload_len] = checksum & 0xFF;

  this->write_array(buf, payload_len + 7);
  this->last_tx_ms_ = millis();
  this->response_received_ = false;
}

void FujitsuAC::send_read_(LastSent tag, const uint16_t *addrs, size_t count) {
  uint8_t payload[64];
  if (count * 2 > sizeof(payload)) {
    ESP_LOGE(TAG, "Read frame too large");
    return;
  }
  for (size_t i = 0; i < count; i++) {
    payload[i * 2] = addrs[i] >> 8;
    payload[i * 2 + 1] = addrs[i] & 0xFF;
  }
  this->last_sent_ = tag;
  this->send_frame_(0x03, payload, count * 2);
}

void FujitsuAC::send_write_frame_() {
  uint8_t payload[64];
  size_t count = this->pending_writes_.size();
  if (count > 15)
    count = 15;  // keep frames well inside the 128 B rx window of the AC

  this->verify_addrs_.clear();
  for (size_t i = 0; i < count; i++) {
    const auto &w = this->pending_writes_[i];
    payload[i * 4] = w.first >> 8;
    payload[i * 4 + 1] = w.first & 0xFF;
    payload[i * 4 + 2] = w.second >> 8;
    payload[i * 4 + 3] = w.second & 0xFF;
    this->verify_addrs_.push_back(w.first);
    ESP_LOGD(TAG, "Writing reg 0x%04X = 0x%04X", w.first, w.second);
  }

  this->last_sent_ = LastSent::WRITE;
  this->send_frame_(0x02, payload, count * 4);
  // Writes stay queued until the AC acks the frame (see handle_frame_),
  // so a lost frame is retried on the next cycle.
}

// ---------------------------------------------------------------------------
// RX framing
// ---------------------------------------------------------------------------
void FujitsuAC::handle_rx_() {
  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b))
      break;

    const uint32_t now = millis();

    // Discard a stale partial frame after a quiet gap
    if (this->rx_idx_ > 0 && (now - this->last_rx_byte_ms_) > RX_GAP_RESET_MS) {
      ESP_LOGV(TAG, "RX gap, discarding %u stale bytes", (unsigned) this->rx_idx_);
      this->rx_idx_ = 0;
    }
    this->last_rx_byte_ms_ = now;

    if (this->rx_idx_ >= sizeof(this->rx_buf_)) {
      ESP_LOGW(TAG, "RX buffer overflow, resetting parser");
      this->rx_idx_ = 0;
    }
    this->rx_buf_[this->rx_idx_++] = b;

    if (this->rx_idx_ >= 5) {
      const size_t expected = (size_t) this->rx_buf_[4] + 7;
      if (expected > sizeof(this->rx_buf_)) {
        // Impossible length byte -> line noise; resync.
        ESP_LOGW(TAG, "Bogus frame length %u, resetting parser", (unsigned) expected);
        this->rx_idx_ = 0;
        continue;
      }
      if (this->rx_idx_ == expected) {
        const bool valid = checksum_ok_(this->rx_buf_, expected);
        this->handle_frame_(this->rx_buf_, expected, valid);
        this->rx_idx_ = 0;
      }
    }
  }
}

bool FujitsuAC::checksum_ok_(const uint8_t *buf, size_t len) {
  const uint16_t frame_checksum = ((uint16_t) buf[len - 2] << 8) | buf[len - 1];
  uint16_t checksum = 0xFFFF;
  for (size_t i = 0; i < len - 2; i++)
    checksum -= buf[i];
  return frame_checksum == checksum;
}

void FujitsuAC::handle_frame_(const uint8_t *buf, size_t len, bool valid) {
  if (!valid) {
    ESP_LOGW(TAG, "Frame with invalid checksum (%u bytes), dropped", (unsigned) len);
    return;
  }

  this->timeout_count_ = 0;
  if (this->comm_lost_logged_) {
    this->comm_lost_logged_ = false;
    ESP_LOGI(TAG, "Communication with AC restored");
  }

  // Handshake replies
  if (this->last_sent_ == LastSent::INIT1) {
    for (const auto &restart : RESTART_FRAMES) {
      if (len == 8 && memcmp(buf, restart, 8) == 0) {
        ESP_LOGD(TAG, "AC restart frame received, waiting for real Init1 reply");
        return;  // keep waiting; Init1 will be retried on timeout
      }
    }
    if (len == sizeof(INIT1_EXPECT) && memcmp(buf, INIT1_EXPECT, sizeof(INIT1_EXPECT)) == 0) {
      this->response_received_ = true;
    } else {
      ESP_LOGW(TAG, "Unexpected Init1 reply, retrying handshake");
      this->last_sent_ = LastSent::NONE;
      this->response_received_ = true;
    }
    return;
  }

  if (this->last_sent_ == LastSent::INIT2) {
    if (len == sizeof(INIT2_EXPECT) && memcmp(buf, INIT2_EXPECT, sizeof(INIT2_EXPECT)) == 0) {
      this->response_received_ = true;
    } else {
      ESP_LOGW(TAG, "Unexpected Init2 reply, retrying handshake");
      this->last_sent_ = LastSent::NONE;
      this->response_received_ = true;
    }
    return;
  }

  // Read response
  if (buf[0] == 0x03) {
    this->response_received_ = true;
    if (buf[5] != 0x01) {
      ESP_LOGW(TAG, "Read response with error status 0x%02X", buf[5]);
      return;
    }
    this->update_registers_(buf, len);
    this->sync_entities_();
    return;
  }

  // Write ack
  if (buf[0] == 0x02) {
    this->response_received_ = true;
    if (buf[5] != 0x01) {
      ESP_LOGW(TAG, "Write rejected by AC (status 0x%02X)", buf[5]);
      this->pending_writes_.clear();
      this->verify_addrs_.clear();
      return;
    }
    // Remove the writes we sent (first N entries); anything queued after
    // the frame went out stays for the next write slot.
    const size_t sent = this->verify_addrs_.size();
    if (sent >= this->pending_writes_.size()) {
      this->pending_writes_.clear();
    } else {
      this->pending_writes_.erase(this->pending_writes_.begin(), this->pending_writes_.begin() + sent);
    }
    return;
  }

  ESP_LOGD(TAG, "Unhandled frame type 0x%02X (%u bytes)", buf[0], (unsigned) len);
}

void FujitsuAC::update_registers_(const uint8_t *buf, size_t len) {
  // payload = 1 status byte + N * (addr_hi addr_lo val_hi val_lo)
  const size_t count = buf[4] / 4;
  for (size_t i = 0; i < count; i++) {
    const size_t idx = 6 + i * 4;
    if (idx + 3 >= len - 2)
      break;  // never index into the checksum
    const uint16_t addr = ((uint16_t) buf[idx] << 8) | buf[idx + 1];
    const uint16_t value = ((uint16_t) buf[idx + 2] << 8) | buf[idx + 3];

    auto it = this->regs_.find(addr);
    if (it == this->regs_.end() || it->second != value) {
      ESP_LOGD(TAG, "Reg 0x%04X: 0x%04X -> 0x%04X", addr, it == this->regs_.end() ? 0 : it->second, value);
      this->regs_[addr] = value;
    }
  }

  // Periodic raw dump of key regs even when unchanged, to distinguish
  // "AC reports a constant" from "we stopped hearing from the AC".
  const uint32_t now = millis();
  if (now - this->last_raw_dump_ms_ > 60000) {
    this->last_raw_dump_ms_ = now;
    ESP_LOGD(TAG, "Raw: actual=0x%04X outdoor=0x%04X power=%u mode=%u", this->get_reg_(REG_ACTUAL_TEMP),
             this->get_reg_(REG_OUTDOOR_TEMP), this->get_reg_(REG_POWER), this->get_reg_(REG_MODE));
  }
}

// ---------------------------------------------------------------------------
// Register helpers
// ---------------------------------------------------------------------------
bool FujitsuAC::feature_supported_(uint16_t addr) const {
  if (addr == REG_V_AIRFLOW_COUNT || addr == REG_H_AIRFLOW_COUNT)
    return this->get_reg_(addr) > 0;
  return this->get_reg_(addr) == 0x0001;
}

uint16_t FujitsuAC::feature_reg_(Feature f) {
  switch (f) {
    case Feature::POWERFUL: return REG_POWERFUL;
    case Feature::ECONOMY: return REG_ECONOMY;
    case Feature::ENERGY_SAVING_FAN: return REG_ENERGY_SAVING_FAN;
    case Feature::OUTDOOR_LOW_NOISE: return REG_OUTDOOR_LOW_NOISE;
    case Feature::COIL_DRY: return REG_COIL_DRY;
    case Feature::HUMAN_SENSOR: return REG_HUMAN_SENSOR;
    case Feature::MINIMUM_HEAT: return REG_MIN_HEAT;
  }
  return 0;
}

uint16_t FujitsuAC::feature_support_reg_(Feature f) {
  switch (f) {
    case Feature::POWERFUL: return REG_POWERFUL_SUPPORTED;
    case Feature::ECONOMY: return REG_ECONOMY_SUPPORTED;
    case Feature::ENERGY_SAVING_FAN: return REG_ESF_SUPPORTED;
    case Feature::OUTDOOR_LOW_NOISE: return REG_OULN_SUPPORTED;
    case Feature::COIL_DRY: return REG_COIL_DRY_SUPPORTED;
    case Feature::HUMAN_SENSOR: return REG_HUMAN_SENSOR_SUPPORTED;
    case Feature::MINIMUM_HEAT: return REG_MIN_HEAT_SUPPORTED;
  }
  return 0;
}

const char *FujitsuAC::feature_name_(Feature f) {
  switch (f) {
    case Feature::POWERFUL: return "powerful";
    case Feature::ECONOMY: return "economy";
    case Feature::ENERGY_SAVING_FAN: return "energy_saving_fan";
    case Feature::OUTDOOR_LOW_NOISE: return "outdoor_unit_low_noise";
    case Feature::COIL_DRY: return "coil_dry";
    case Feature::HUMAN_SENSOR: return "human_sensor";
    case Feature::MINIMUM_HEAT: return "minimum_heat";
  }
  return "?";
}

void FujitsuAC::queue_write_(uint16_t addr, uint16_t value) {
  for (auto &w : this->pending_writes_) {
    if (w.first == addr) {
      w.second = value;
      return;
    }
  }
  this->pending_writes_.emplace_back(addr, value);
}

uint16_t FujitsuAC::encode_setpoint_(float temp, uint16_t mode_reg) const {
  int raw = (int) lroundf(temp * 10.0f);
  raw = (raw + 2) / 5 * 5;  // snap to 0.5 C steps (upstream behaviour)
  const int min_raw = (mode_reg == MODE_HEAT) ? 160 : 180;
  if (raw < min_raw) {
    ESP_LOGW(TAG, "Setpoint %.1f below minimum, clamping", temp);
    raw = min_raw;
  } else if (raw > 300) {
    ESP_LOGW(TAG, "Setpoint %.1f above maximum, clamping", temp);
    raw = 300;
  }
  return (uint16_t) raw;
}

optional<float> FujitsuAC::decode_probe_temp_(uint16_t raw) {
  // Probe temps: (raw - 5025) / 100 degC. 0x0000 = not read yet,
  // 0xFFFF = sensor unavailable.
  if (raw == 0x0000 || raw == 0xFFFF)
    return {};
  const float t = ((int32_t) raw - 5025) / 100.0f;
  if (t < -50.0f || t > 80.0f)
    return {};
  return t;
}

// ---------------------------------------------------------------------------
// Climate
// ---------------------------------------------------------------------------
climate::ClimateTraits FujitsuAC::climate_traits() {
  climate::ClimateTraits traits;
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.set_visual_min_temperature(16.0f);
  traits.set_visual_max_temperature(30.0f);
  traits.set_visual_target_temperature_step(0.5f);
  traits.set_visual_current_temperature_step(0.1f);

  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT_COOL,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_HEAT,
  });

  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_QUIET,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });

  // Until discovery has run, offer everything; afterwards trim to hardware.
  const bool v = !this->init_regs_done_ || this->feature_supported_(REG_V_SWING_SUPPORTED);
  const bool h = !this->init_regs_done_ || this->feature_supported_(REG_H_SWING_SUPPORTED);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
  if (v)
    traits.add_supported_swing_mode(climate::CLIMATE_SWING_VERTICAL);
  if (h)
    traits.add_supported_swing_mode(climate::CLIMATE_SWING_HORIZONTAL);
  if (v && h)
    traits.add_supported_swing_mode(climate::CLIMATE_SWING_BOTH);

  return traits;
}

void FujitsuAC::control_climate(const climate::ClimateCall &call) {
  if (this->phase_ != Phase::RUN) {
    ESP_LOGW(TAG, "Ignoring climate command, AC not connected yet");
    return;
  }

  const bool guarded = this->coil_dry_on_() || this->min_heat_on_();
  uint16_t effective_mode_reg = this->get_reg_(REG_MODE);

  if (call.get_mode().has_value()) {
    const auto mode = *call.get_mode();
    if (mode == climate::CLIMATE_MODE_OFF) {
      this->queue_write_(REG_POWER, 0x0000);
      this->climate_->mode = climate::CLIMATE_MODE_OFF;
    } else if (guarded) {
      ESP_LOGW(TAG, "Mode change blocked: coil dry / minimum heat is active");
    } else {
      uint16_t mode_reg;
      switch (mode) {
        case climate::CLIMATE_MODE_HEAT_COOL: mode_reg = MODE_AUTO; break;
        case climate::CLIMATE_MODE_COOL: mode_reg = MODE_COOL; break;
        case climate::CLIMATE_MODE_DRY: mode_reg = MODE_DRY; break;
        case climate::CLIMATE_MODE_FAN_ONLY: mode_reg = MODE_FAN; break;
        case climate::CLIMATE_MODE_HEAT: mode_reg = MODE_HEAT; break;
        default:
          ESP_LOGW(TAG, "Unsupported climate mode %d", (int) mode);
          mode_reg = 0xFFFF;
          break;
      }
      if (mode_reg != 0xFFFF) {
        if (!this->powered_on_())
          this->queue_write_(REG_POWER, 0x0001);
        this->queue_write_(REG_MODE, mode_reg);
        effective_mode_reg = mode_reg;
        this->climate_->mode = mode;
      }
    }
  }

  if (call.get_target_temperature().has_value()) {
    if (guarded) {
      ESP_LOGW(TAG, "Setpoint change blocked: coil dry / minimum heat is active");
    } else if (effective_mode_reg == MODE_FAN) {
      ESP_LOGW(TAG, "Setpoint ignored in fan-only mode");
    } else {
      const float t = *call.get_target_temperature();
      const uint16_t raw = this->encode_setpoint_(t, effective_mode_reg);
      this->queue_write_(REG_SETPOINT, raw);
      this->climate_->target_temperature = raw / 10.0f;
    }
  }

  if (call.get_fan_mode().has_value()) {
    if (guarded) {
      ESP_LOGW(TAG, "Fan change blocked: coil dry / minimum heat is active");
    } else {
      uint16_t fan_reg = 0xFFFF;
      switch (*call.get_fan_mode()) {
        case climate::CLIMATE_FAN_AUTO: fan_reg = FAN_R_AUTO; break;
        case climate::CLIMATE_FAN_QUIET: fan_reg = FAN_R_QUIET; break;
        case climate::CLIMATE_FAN_LOW: fan_reg = FAN_R_LOW; break;
        case climate::CLIMATE_FAN_MEDIUM: fan_reg = FAN_R_MEDIUM; break;
        case climate::CLIMATE_FAN_HIGH: fan_reg = FAN_R_HIGH; break;
        default: ESP_LOGW(TAG, "Unsupported fan mode"); break;
      }
      if (fan_reg != 0xFFFF) {
        this->queue_write_(REG_FAN_SPEED, fan_reg);
        this->climate_->fan_mode = *call.get_fan_mode();
      }
    }
  }

  if (call.get_swing_mode().has_value()) {
    if (this->coil_dry_on_()) {
      ESP_LOGW(TAG, "Swing change blocked: coil dry is active");
    } else {
      const auto swing = *call.get_swing_mode();
      const bool want_v = swing == climate::CLIMATE_SWING_VERTICAL || swing == climate::CLIMATE_SWING_BOTH;
      const bool want_h = swing == climate::CLIMATE_SWING_HORIZONTAL || swing == climate::CLIMATE_SWING_BOTH;
      if (this->feature_supported_(REG_V_SWING_SUPPORTED))
        this->queue_write_(REG_V_SWING, want_v ? 0x0001 : 0x0000);
      if (this->feature_supported_(REG_H_SWING_SUPPORTED))
        this->queue_write_(REG_H_SWING, want_h ? 0x0001 : 0x0000);
      this->climate_->swing_mode = swing;
    }
  }

  // Optimistic publish; verified state arrives via the read-back frame.
  this->climate_->publish_state();
}

// ---------------------------------------------------------------------------
// Extra entities
// ---------------------------------------------------------------------------
bool FujitsuAC::set_feature_state(Feature feature, bool state) {
  if (this->phase_ != Phase::RUN) {
    ESP_LOGW(TAG, "Ignoring %s command, AC not connected yet", feature_name_(feature));
    return false;
  }

  if (feature == Feature::MINIMUM_HEAT && !this->allow_min_heat_write_) {
    ESP_LOGW(TAG,
             "Minimum-heat writes are disabled (untested upstream). "
             "Set enable_minimum_heat_control: true to allow.");
    return false;
  }

  // Guards mirrored from TFSXW1Controller
  const bool needs_mode_guard = feature == Feature::POWERFUL || feature == Feature::ECONOMY;
  if (needs_mode_guard && (this->coil_dry_on_() || this->min_heat_on_())) {
    ESP_LOGW(TAG, "%s change blocked: coil dry / minimum heat is active", feature_name_(feature));
    return false;
  }
  if (feature == Feature::ENERGY_SAVING_FAN && this->min_heat_on_()) {
    ESP_LOGW(TAG, "%s change blocked: minimum heat is active", feature_name_(feature));
    return false;
  }

  const uint16_t support_reg = feature_support_reg_(feature);
  if (this->init_regs_done_ && support_reg != 0 && !this->feature_supported_(support_reg)) {
    ESP_LOGW(TAG, "%s is not supported by this indoor unit", feature_name_(feature));
    return false;
  }

  this->queue_write_(feature_reg_(feature), state ? 0x0001 : 0x0000);
  return true;
}

void FujitsuAC::set_airflow(bool horizontal, const std::string &value) {
  if (this->phase_ != Phase::RUN) {
    ESP_LOGW(TAG, "Ignoring airflow command, AC not connected yet");
    return;
  }
  if (this->coil_dry_on_()) {
    ESP_LOGW(TAG, "Airflow change blocked: coil dry is active");
    return;
  }

  const uint16_t count_reg = horizontal ? REG_H_AIRFLOW_COUNT : REG_V_AIRFLOW_COUNT;
  const uint16_t swing_reg = horizontal ? REG_H_SWING : REG_V_SWING;
  const uint16_t setter_reg = horizontal ? REG_H_AIRFLOW_SET : REG_V_AIRFLOW_SET;

  if (!this->feature_supported_(count_reg)) {
    ESP_LOGW(TAG, "%s airflow is not supported by this indoor unit", horizontal ? "Horizontal" : "Vertical");
    return;
  }

  if (value == "Swing") {
    this->queue_write_(swing_reg, 0x0001);
    return;
  }

  // "Position N"
  int pos = 0;
  if (sscanf(value.c_str(), "Position %d", &pos) == 1 && pos >= 1 && pos <= 6) {
    const uint16_t max_pos = this->get_reg_(count_reg, 6);
    if (max_pos > 0 && pos > max_pos) {
      ESP_LOGW(TAG, "Position %d exceeds louvre count (%u) reported by unit", pos, max_pos);
      return;
    }
    this->queue_write_(swing_reg, 0x0000);
    this->queue_write_(setter_reg, (uint16_t) pos);
  } else {
    ESP_LOGW(TAG, "Unknown airflow option '%s'", value.c_str());
  }
}

// ---------------------------------------------------------------------------
// Entity state sync (device -> HA)
// ---------------------------------------------------------------------------
void FujitsuAC::sync_entities_() {
  this->sync_climate_();

  if (this->indoor_sensor_ != nullptr && this->has_reg_(REG_ACTUAL_TEMP)) {
    auto t = decode_probe_temp_(this->get_reg_(REG_ACTUAL_TEMP));
    if (t.has_value() &&
        (!this->indoor_sensor_->has_state() || fabsf(this->indoor_sensor_->state - *t) > 0.049f)) {
      this->indoor_sensor_->publish_state(*t);
    }
  }

  if (this->outdoor_sensor_ != nullptr && this->has_reg_(REG_OUTDOOR_TEMP)) {
    auto t = decode_probe_temp_(this->get_reg_(REG_OUTDOOR_TEMP));
    if (t.has_value() &&
        (!this->outdoor_sensor_->has_state() || fabsf(this->outdoor_sensor_->state - *t) > 0.049f)) {
      this->outdoor_sensor_->publish_state(*t);
    }
  }

  for (auto *sw : this->switches_) {
    const uint16_t reg = feature_reg_(sw->get_feature());
    if (!this->has_reg_(reg))
      continue;
    const bool state = this->get_reg_(reg) == 0x0001;
    if (sw->state != state)
      sw->publish_state(state);
  }

  const auto sync_select = [this](FujitsuSelect *sel, uint16_t status_reg) {
    if (sel == nullptr || !this->has_reg_(status_reg))
      return;
    const uint16_t raw = this->get_reg_(status_reg);
    std::string value;
    if (raw == AIRFLOW_SWING) {
      value = "Swing";
    } else if (raw >= 1 && raw <= 6) {
      value = "Position " + to_string(raw);
    } else {
      return;
    }
    if (sel->current_option() != value)
      sel->publish_state(value);
  };
  sync_select(this->vertical_select_, REG_V_AIRFLOW);
  sync_select(this->horizontal_select_, REG_H_AIRFLOW);
}

void FujitsuAC::sync_climate_() {
  if (this->climate_ == nullptr || !this->has_reg_(REG_POWER))
    return;

  bool changed = false;

  climate::ClimateMode mode = climate::CLIMATE_MODE_OFF;
  if (this->powered_on_()) {
    switch (this->get_reg_(REG_MODE)) {
      case MODE_AUTO: mode = climate::CLIMATE_MODE_HEAT_COOL; break;
      case MODE_COOL: mode = climate::CLIMATE_MODE_COOL; break;
      case MODE_DRY: mode = climate::CLIMATE_MODE_DRY; break;
      case MODE_FAN: mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      case MODE_HEAT: mode = climate::CLIMATE_MODE_HEAT; break;
      default: mode = climate::CLIMATE_MODE_HEAT_COOL; break;
    }
  }
  if (this->climate_->mode != mode) {
    this->climate_->mode = mode;
    changed = true;
  }

  const uint16_t sp = this->get_reg_(REG_SETPOINT);
  if (sp != 0x0000 && sp != 0xFFFF) {  // fan mode reports 0xFFFF
    const float target = sp / 10.0f;
    if (std::isnan(this->climate_->target_temperature) ||
        fabsf(this->climate_->target_temperature - target) > 0.049f) {
      this->climate_->target_temperature = target;
      changed = true;
    }
  }

  if (this->has_reg_(REG_ACTUAL_TEMP)) {
    auto t = decode_probe_temp_(this->get_reg_(REG_ACTUAL_TEMP));
    if (t.has_value() && (std::isnan(this->climate_->current_temperature) ||
                          fabsf(this->climate_->current_temperature - *t) > 0.049f)) {
      this->climate_->current_temperature = *t;
      changed = true;
    }
  }

  if (this->has_reg_(REG_FAN_SPEED)) {
    optional<climate::ClimateFanMode> fan;
    switch (this->get_reg_(REG_FAN_SPEED)) {
      case FAN_R_AUTO: fan = climate::CLIMATE_FAN_AUTO; break;
      case FAN_R_QUIET: fan = climate::CLIMATE_FAN_QUIET; break;
      case FAN_R_LOW: fan = climate::CLIMATE_FAN_LOW; break;
      case FAN_R_MEDIUM: fan = climate::CLIMATE_FAN_MEDIUM; break;
      case FAN_R_HIGH: fan = climate::CLIMATE_FAN_HIGH; break;
      default: break;
    }
    if (fan.has_value() && this->climate_->fan_mode != *fan) {
      this->climate_->fan_mode = *fan;
      changed = true;
    }
  }

  if (this->has_reg_(REG_V_SWING) || this->has_reg_(REG_H_SWING)) {
    const bool v = this->get_reg_(REG_V_SWING) == 0x0001;
    const bool h = this->get_reg_(REG_H_SWING) == 0x0001;
    climate::ClimateSwingMode swing = climate::CLIMATE_SWING_OFF;
    if (v && h)
      swing = climate::CLIMATE_SWING_BOTH;
    else if (v)
      swing = climate::CLIMATE_SWING_VERTICAL;
    else if (h)
      swing = climate::CLIMATE_SWING_HORIZONTAL;
    if (this->climate_->swing_mode != swing) {
      this->climate_->swing_mode = swing;
      changed = true;
    }
  }

  if (changed)
    this->climate_->publish_state();
}

// ---------------------------------------------------------------------------
// Entity glue
// ---------------------------------------------------------------------------
void FujitsuClimate::control(const climate::ClimateCall &call) { this->parent_->control_climate(call); }
climate::ClimateTraits FujitsuClimate::traits() { return this->parent_->climate_traits(); }
void FujitsuClimate::dump_config() { LOG_CLIMATE("", "Fujitsu AC Climate", this); }

void FujitsuSwitch::write_state(bool state) {
  if (this->parent_->set_feature_state(this->feature_, state)) {
    // Optimistic; corrected by the verify read if the AC disagrees.
    this->publish_state(state);
  } else {
    // Refused -> republish real state so the UI toggle snaps back.
    this->publish_state(this->state);
  }
}

void FujitsuSelect::control(const std::string &value) {
  this->parent_->set_airflow(this->horizontal_, value);
  this->publish_state(value);
}

}  // namespace fujitsu_ac
}  // namespace esphome
