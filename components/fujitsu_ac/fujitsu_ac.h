// ESPHome external component for Fujitsu air conditioners speaking the
// UTY-TFSXW1 (FGLair dongle) UART protocol.
//
// Protocol logic ported from https://github.com/Benas09/FujitsuAC
// (TFSXW1Controller / TFSXW1Bridge), reimplemented on top of ESPHome's
// uart::UARTDevice with a hardened frame parser.
//
// UART: 9600 8N1, BOTH LINES INVERTED. Configure the uart bus with
// inverted: true on rx_pin and tx_pin (see example YAML).

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace esphome {
namespace fujitsu_ac {

// ---------------------------------------------------------------------------
// Register map (TFSXW1)
// ---------------------------------------------------------------------------
enum Reg : uint16_t {
  // Capability / discovery registers
  REG_V_AIRFLOW_COUNT = 0x0130,
  REG_V_SWING_SUPPORTED = 0x0131,
  REG_H_AIRFLOW_COUNT = 0x0142,
  REG_H_SWING_SUPPORTED = 0x0143,
  REG_ECONOMY_SUPPORTED = 0x0150,
  REG_MIN_HEAT_SUPPORTED = 0x0151,
  REG_HUMAN_SENSOR_SUPPORTED = 0x0152,
  REG_ESF_SUPPORTED = 0x0153,
  REG_POWERFUL_SUPPORTED = 0x0170,
  REG_OULN_SUPPORTED = 0x0171,
  REG_COIL_DRY_SUPPORTED = 0x0193,

  // State / control registers
  REG_POWER = 0x1000,
  REG_MODE = 0x1001,
  REG_SETPOINT = 0x1002,       // temp * 10, 0.5 C steps; 0xFFFF in fan mode
  REG_FAN_SPEED = 0x1003,
  REG_V_AIRFLOW_SET = 0x1010,  // write-side vertical louvre position
  REG_V_SWING = 0x1011,
  REG_V_AIRFLOW = 0x10A0,      // read-side vertical louvre position (0x20 = swing)
  REG_H_AIRFLOW_SET = 0x1022,
  REG_H_SWING = 0x1023,
  REG_H_AIRFLOW = 0x10A9,
  REG_ACTUAL_TEMP = 0x1033,    // (raw - 5025) / 100 degC
  REG_ECONOMY = 0x1100,
  REG_MIN_HEAT = 0x1101,
  REG_HUMAN_SENSOR = 0x1102,
  REG_ENERGY_SAVING_FAN = 0x1108,
  REG_POWERFUL = 0x1120,
  REG_OUTDOOR_LOW_NOISE = 0x1121,
  REG_COIL_DRY = 0x1144,
  REG_OUTDOOR_TEMP = 0x2020,   // (raw - 5025) / 100 degC
};

// Mode register values
static const uint16_t MODE_AUTO = 0x0000;
static const uint16_t MODE_COOL = 0x0001;
static const uint16_t MODE_DRY = 0x0002;
static const uint16_t MODE_FAN = 0x0003;
static const uint16_t MODE_HEAT = 0x0004;

// Fan speed register values
static const uint16_t FAN_R_AUTO = 0x0000;
static const uint16_t FAN_R_QUIET = 0x0002;
static const uint16_t FAN_R_LOW = 0x0005;
static const uint16_t FAN_R_MEDIUM = 0x0008;
static const uint16_t FAN_R_HIGH = 0x000B;

static const uint16_t AIRFLOW_SWING = 0x0020;

enum class Feature : uint8_t {
  POWERFUL = 0,
  ECONOMY,
  ENERGY_SAVING_FAN,
  OUTDOOR_LOW_NOISE,
  COIL_DRY,
  HUMAN_SENSOR,
  MINIMUM_HEAT,
};

class FujitsuAC;

// ---------------------------------------------------------------------------
// Entities
// ---------------------------------------------------------------------------
class FujitsuClimate : public climate::Climate, public Component {
 public:
  void set_parent(FujitsuAC *parent) { this->parent_ = parent; }
  void dump_config() override;

 protected:
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  FujitsuAC *parent_{nullptr};
};

class FujitsuSwitch : public switch_::Switch, public Parented<FujitsuAC> {
 public:
  void set_feature(Feature feature) { this->feature_ = feature; }
  Feature get_feature() const { return this->feature_; }

 protected:
  void write_state(bool state) override;
  Feature feature_{Feature::POWERFUL};
};

class FujitsuSelect : public select::Select, public Parented<FujitsuAC> {
 public:
  void set_horizontal(bool horizontal) { this->horizontal_ = horizontal; }
  bool is_horizontal() const { return this->horizontal_; }

 protected:
  void control(const std::string &value) override;
  bool horizontal_{false};
};

// ---------------------------------------------------------------------------
// Hub
// ---------------------------------------------------------------------------
class FujitsuAC : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Wiring from codegen
  void set_climate(FujitsuClimate *climate) {
    this->climate_ = climate;
    climate->set_parent(this);
  }
  void set_outdoor_sensor(sensor::Sensor *s) { this->outdoor_sensor_ = s; }
  void set_indoor_sensor(sensor::Sensor *s) { this->indoor_sensor_ = s; }
  void set_vertical_select(FujitsuSelect *s) { this->vertical_select_ = s; }
  void set_horizontal_select(FujitsuSelect *s) { this->horizontal_select_ = s; }
  void register_switch(FujitsuSwitch *sw) { this->switches_.push_back(sw); }
  void set_allow_minimum_heat_write(bool allow) { this->allow_min_heat_write_ = allow; }

  // Entity -> hub commands
  void control_climate(const climate::ClimateCall &call);
  climate::ClimateTraits climate_traits();
  bool set_feature_state(Feature feature, bool state);
  void set_airflow(bool horizontal, const std::string &value);

  bool is_connected() const { return this->phase_ == Phase::RUN; }

 protected:
  enum class Phase : uint8_t { HANDSHAKE, RUN };
  enum class LastSent : uint8_t {
    NONE,
    INIT1,
    INIT2,
    IREG1,
    IREG2,
    IREG3,
    FRAME_A,
    WRITE,
    VERIFY,
    FRAME_B,
    FRAME_C,
  };

  // --- protocol / IO ---
  void tick_();
  void send_frame_(uint8_t type, const uint8_t *payload, size_t payload_len);
  void send_read_(LastSent tag, const uint16_t *addrs, size_t count);
  void send_write_frame_();
  void handle_rx_();
  void handle_frame_(const uint8_t *buf, size_t len, bool valid);
  void update_registers_(const uint8_t *buf, size_t len);
  static bool checksum_ok_(const uint8_t *buf, size_t len);

  // --- register helpers ---
  bool has_reg_(uint16_t addr) const { return this->regs_.count(addr) != 0; }
  uint16_t get_reg_(uint16_t addr, uint16_t fallback = 0) const {
    auto it = this->regs_.find(addr);
    return it == this->regs_.end() ? fallback : it->second;
  }
  bool powered_on_() const { return this->get_reg_(REG_POWER) == 0x0001; }
  bool coil_dry_on_() const { return this->get_reg_(REG_COIL_DRY) == 0x0001; }
  bool min_heat_on_() const { return this->get_reg_(REG_MIN_HEAT) == 0x0001; }
  bool feature_supported_(uint16_t addr) const;
  static uint16_t feature_reg_(Feature f);
  static uint16_t feature_support_reg_(Feature f);
  static const char *feature_name_(Feature f);

  void queue_write_(uint16_t addr, uint16_t value);
  uint16_t encode_setpoint_(float temp, uint16_t mode_reg) const;
  static optional<float> decode_probe_temp_(uint16_t raw);

  // --- entity sync ---
  void sync_entities_();
  void sync_climate_();

  // --- entities ---
  FujitsuClimate *climate_{nullptr};
  sensor::Sensor *outdoor_sensor_{nullptr};
  sensor::Sensor *indoor_sensor_{nullptr};
  FujitsuSelect *vertical_select_{nullptr};
  FujitsuSelect *horizontal_select_{nullptr};
  std::vector<FujitsuSwitch *> switches_;
  bool allow_min_heat_write_{false};

  // --- state ---
  std::map<uint16_t, uint16_t> regs_;
  std::vector<std::pair<uint16_t, uint16_t>> pending_writes_;
  std::vector<uint16_t> verify_addrs_;

  Phase phase_{Phase::HANDSHAKE};
  LastSent last_sent_{LastSent::NONE};
  bool response_received_{true};
  bool init_regs_done_{false};
  uint32_t last_tx_ms_{0};
  uint8_t timeout_count_{0};
  bool comm_lost_logged_{false};

  // rx assembly
  uint8_t rx_buf_[128];
  size_t rx_idx_{0};
  uint32_t last_rx_byte_ms_{0};
};

}  // namespace fujitsu_ac
}  // namespace esphome
