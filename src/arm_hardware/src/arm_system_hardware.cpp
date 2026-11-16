#include "arm_hardware/arm_system_hardware.hpp"

#include "hardware_interface/types/lifecycle_state_names.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_hardware
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr size_t kMegaFrame = 34;
// 32 bytes: sync(2) ms(4) enabled fault fault_id pos(6) load(6) target(6)
// volt(3) volts_min checksum
constexpr size_t kWristFrame = 32;
constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;

uint8_t xor_checksum(const std::string & text)
{
  uint8_t v = 0;
  for (const char c : text) {
    v ^= static_cast<uint8_t>(c);
  }
  return v;
}

int16_t read_i16(const uint8_t * p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }
uint16_t read_u16(const uint8_t * p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

const char * mega_fault(uint8_t code)
{
  switch (code) {
    case 1: return "tracking error (jam, collision, or lost power)";
    case 2: return "pot wiper out of range (wire open)";
    case 3: return "pot excitation lost";
    case 4: return "dead-man: no command reached the Mega for 400 ms";
    case 5: return "refused to enable";
    default: return "unknown";
  }
}

const char * wrist_fault(uint8_t code)
{
  switch (code) {
    case 1: return "load limit (collision or stall)";
    case 2: return "a servo stopped answering the bus";
    case 3: return "dead-man: no command reached the ESP32 for 400 ms";
    case 4: return "refused to enable";
    case 5: return "supply below 9.0 V - the pack sags under load; check leads and connectors";
    default: return "unknown";
  }
}

std::string param_or(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

/// Find complete, checksum-valid frames in `rx` and keep the newest. Anything
/// failing its checksum is discarded: the Mega's USB bridge corrupts a small
/// fraction of traffic under load, and a corrupted frame must never be acted on.
/// `text` collects the bytes that are not part of a binary frame - the boards
/// print diagnostics on the same link, and those lines are the only place the
/// firmware says why it did something ("!! refusing: id 1 not answering",
/// where the status frame carries only the generic fault code).
bool scan_frames(
  std::vector<uint8_t> & rx, size_t frame_size, size_t checksum_index,
  const std::function<void(const uint8_t *)> & accept, std::string * text = nullptr)
{
  bool found = false;
  size_t search = 0;
  while (rx.size() - search >= frame_size) {
    if (rx[search] != kSync0 || rx[search + 1] != kSync1) {
      if (text != nullptr) {
        const char c = static_cast<char>(rx[search]);
        // Printable ASCII and newlines only: anything else is a fragment of a
        // binary frame that lost sync, not something the board meant to say.
        if (c == '\n' || (c >= 0x20 && c <= 0x7E)) {
          text->push_back(c);
        }
      }
      ++search;
      continue;
    }
    const uint8_t * f = rx.data() + search;
    uint8_t sum = 0;
    for (size_t i = 2; i < checksum_index; ++i) {
      sum ^= f[i];
    }
    if (sum != f[checksum_index]) {
      search += 2;
      continue;
    }
    accept(f);
    found = true;
    search += frame_size;
  }
  rx.erase(rx.begin(), rx.begin() + static_cast<long>(search));
  if (rx.size() > 4096) {
    rx.clear();
  }
  return found;
}

// Log the board's own words. Only complete lines are emitted, so one split
// across two reads is held until the rest arrives, and only lines the firmware
// marks as problems ("!!") - "##" status chatter would bury them.
void drain_board_text(
  std::string & text, const char * who, const rclcpp::Logger & logger)
{
  size_t nl;
  while ((nl = text.find('\n')) != std::string::npos) {
    std::string line = text.substr(0, nl);
    text.erase(0, nl + 1);
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
      line.pop_back();
    }
    if (line.rfind("!!", 0) == 0) {
      RCLCPP_WARN(logger, "%s says: %s", who, line.c_str());
    }
  }
  // Never let a board that prints without newlines grow this without bound.
  if (text.size() > 512) {
    text.clear();
  }
}
}  // namespace

hardware_interface::CallbackReturn ArmSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & info = get_hardware_info();

  mega_port_ = param_or(info.hardware_parameters, "mega_port", mega_port_);
  wrist_port_ = param_or(info.hardware_parameters, "esp32_port", wrist_port_);
  mega_baud_ = std::stoi(param_or(info.hardware_parameters, "mega_baud", "250000"));
  wrist_baud_ = std::stoi(param_or(info.hardware_parameters, "esp32_baud", "115200"));
  status_hz_ = std::stoi(param_or(info.hardware_parameters, "status_hz", "100"));
  handshake_timeout_ms_ =
    std::stoi(param_or(info.hardware_parameters, "handshake_timeout_ms", "3000"));

  // ARM_LOG_SUPPLY=1 logs every servo's supply and load twice a second. The
  // threshold warning alone cannot say whether a servo's voltage tracks its own
  // current or the current drawn by everything else; a time series can.
  if (const char * env = std::getenv("ARM_LOG_SUPPLY")) {
    if (env[0] != '\0' && env[0] != '0') {
      supply_log_period_ = 0.5;
    }
  }

  joints_.clear();
  for (const auto & joint : info.joints) {
    JointMap map;
    map.name = joint.name;
    const std::string board = param_or(joint.parameters, "board", "loopback");

    if (board == "mega") {
      map.board = Board::Mega;
      map.channel = std::stoi(param_or(joint.parameters, "channel", "0"));
      map.zero_deg = std::stod(param_or(joint.parameters, "zero_deg", "0"));
    } else if (board == "esp32") {
      map.board = Board::Esp32;
      // The firmware always reports ids 1,2,3 in that order, so the servo id
      // doubles as the slot index.
      map.channel = std::stoi(param_or(joint.parameters, "servo_id", "0"));
      map.zero_counts = std::stod(param_or(joint.parameters, "zero_counts", "2048"));
      map.counts_per_rad = std::stod(param_or(joint.parameters, "counts_per_rad", "651.899"));
    } else if (board == "loopback") {
      map.board = Board::Loopback;
    } else {
      RCLCPP_ERROR(
        get_logger(), "joint '%s': unknown board '%s' (expected mega, esp32 or loopback)",
        joint.name.c_str(), board.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (map.board != Board::Loopback && (map.channel < 1 || map.channel > 3)) {
      RCLCPP_ERROR(
        get_logger(), "joint '%s': board=%s needs channel/servo_id 1..3, got %d",
        joint.name.c_str(), board.c_str(), map.channel);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // What this joint reports before it has been measured or commanded. It
    // matters most for the gripper: its travel is entirely negative, so the 0.0
    // an uninitialised interface holds is outside its limits and the planner
    // rejects the whole state.
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION && !si.initial_value.empty()) {
        map.initial_position = std::stod(si.initial_value);
      }
    }

    // Position limits straight from the URDF, which gets them from
    // joint_parameters.yaml. Read rather than hard-coded so a limit is still
    // written in exactly one place.
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION) {
        continue;
      }
      if (!ci.min.empty()) {
        map.limit_lower = std::stod(ci.min);
      }
      if (!ci.max.empty()) {
        map.limit_upper = std::stod(ci.max);
      }
    }
    if (map.limit_lower > map.limit_upper) {
      RCLCPP_ERROR(
        get_logger(), "joint '%s': position min %.6f is above max %.6f", joint.name.c_str(),
        map.limit_lower, map.limit_upper);
      return hardware_interface::CallbackReturn::ERROR;
    }

    map.direction = std::stod(param_or(joint.parameters, "direction", "1"));
    if (std::abs(std::abs(map.direction) - 1.0) > 1e-9) {
      RCLCPP_ERROR(
        get_logger(), "joint '%s': direction must be 1 or -1, got %f", joint.name.c_str(),
        map.direction);
      return hardware_interface::CallbackReturn::ERROR;
    }
    joints_.push_back(map);
  }

  // Each channel of a board in use must be claimed exactly once. A duplicate
  // means two URDF joints driving one servo; a gap leaves a servo uncommanded
  // while its firmware still holds it.
  for (const Board board : {Board::Mega, Board::Esp32}) {
    if (!uses(board)) {
      continue;
    }
    for (int channel = 1; channel <= 3; ++channel) {
      const auto n = std::count_if(
        joints_.begin(), joints_.end(),
        [board, channel](const JointMap & j) { return j.board == board && j.channel == channel; });
      if (n != 1) {
        RCLCPP_ERROR(
          get_logger(), "%s channel %d is claimed by %ld joints, expected exactly 1",
          board == Board::Mega ? "Mega" : "ESP32", channel, n);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  for (const auto & j : joints_) {
    if (j.board == Board::Mega) {
      RCLCPP_INFO(
        get_logger(), "%-10s mega     channel %d  direction %+.0f  zero %+7.2f deg",
        j.name.c_str(), j.channel, j.direction, j.zero_deg);
    } else if (j.board == Board::Esp32) {
      RCLCPP_INFO(
        get_logger(), "%-10s esp32    servo id %d  direction %+.0f  zero %.0f counts",
        j.name.c_str(), j.channel, j.direction, j.zero_counts);
    } else {
      RCLCPP_INFO(get_logger(), "%-10s loopback (no hardware)", j.name.c_str());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool ArmSystemHardware::uses(Board board) const
{
  return std::any_of(
    joints_.begin(), joints_.end(), [board](const JointMap & j) { return j.board == board; });
}

hardware_interface::CallbackReturn ArmSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  std::string error;

  // Start every joint where the URDF declared. Until the boards have been read
  // this is the only honest answer, and it has to be a legal one: the component
  // sits inactive at startup, so these are what the stack sees first.
  for (const auto & j : joints_) {
    set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, j.initial_position);
    set_state(j.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  }

  if (uses(Board::Mega)) {
    if (!mega_.open(mega_port_, mega_baud_, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (uses(Board::Esp32)) {
    if (!wrist_.open(wrist_port_, wrist_baud_, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      mega_.close();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Opening a port can reset the board. Both firmwares boot into their safe
  // state - Mega detached, wrist torque off - so waiting it out is enough.
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  rx_.clear();
  text_.clear();
  rx_wrist_.clear();
  text_wrist_.clear();
  mega_.flush_input();
  wrist_.flush_input();

  if (uses(Board::Mega)) {
    send_line(mega_, "B " + std::to_string(status_hz_));
    MegaFrame frame;
    if (!wait_for_mega(std::chrono::milliseconds(handshake_timeout_ms_), frame)) {
      RCLCPP_ERROR(
        get_logger(), "no status frame from the Mega on %s within %d ms. Is arm3 flashed?",
        mega_port_.c_str(), handshake_timeout_ms_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      get_logger(), "Mega on %s: %.2f %.2f %.2f deg", mega_port_.c_str(), frame.measured_deg[0],
      frame.measured_deg[1], frame.measured_deg[2]);
  }

  if (uses(Board::Esp32)) {
    send_line(wrist_, "B " + std::to_string(status_hz_));
    WristFrame frame;
    if (!wait_for_wrist(std::chrono::milliseconds(handshake_timeout_ms_), frame)) {
      RCLCPP_ERROR(
        get_logger(), "no status frame from the ESP32 on %s within %d ms. Is st3215_arm flashed?",
        wrist_port_.c_str(), handshake_timeout_ms_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      get_logger(), "ESP32 on %s: %u %u %u counts", wrist_port_.c_str(), frame.position[0],
      frame.position[1], frame.position[2]);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (mega_.is_open()) {
    send_line(mega_, "D");
    send_line(mega_, "B 0");
    mega_.close();
  }
  if (wrist_.is_open()) {
    send_line(wrist_, "D");
    send_line(wrist_, "B 0");
    wrist_.close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  fault_reported_ = false;
  MegaFrame mega{};
  WristFrame wrist{};

  // Relax both boards before enabling either. Activation enables one at a time
  // and the Mega's handshake can take handshake_timeout_ms, during which an
  // ESP32 still enabled from a previous session receives nothing and trips its
  // own 400 ms dead-man.
  send_line(mega_, "D");
  send_line(wrist_, "D");

  // Feed both dead-men whenever activation blocks. Each board relaxes if no
  // byte reaches it for 400 ms, and activation waits for a handshake per board
  // plus up to three attempts each, so feeding only the board being enabled
  // starves the other. Every sleep in here is spent feeding; a heartbeat to a
  // board mid-handshake is harmless.
  auto feed_both = [this]() {
      if (uses(Board::Mega)) { send_line(mega_, "h"); }
      if (uses(Board::Esp32)) { send_line(wrist_, "h"); }
    };
  auto feed_for = [&feed_both](std::chrono::milliseconds total) {
      const auto until = std::chrono::steady_clock::now() + total;
      while (std::chrono::steady_clock::now() < until) {
        feed_both();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    };

  // Each board refuses to enable unless its own joints are healthy, and says
  // which one is not.
  for (int pass = 0; pass < 2; ++pass) {
    const bool mega_pass = pass == 0;
    if (mega_pass && !uses(Board::Mega)) {
      continue;
    }
    if (!mega_pass && !uses(Board::Esp32)) {
      continue;
    }
    SerialPort & port = mega_pass ? mega_ : wrist_;

    // Each attempt flushes this board's queued frames immediately before
    // clearing its fault, so every frame examined postdates the clear. Both
    // boards stream from on_configure onward, and a board left faulted is the
    // normal case - a stack that is killed never sends a deactivate, so the
    // ESP32 trips its dead-man. The flush must be per board and inside this
    // loop: the Mega pass runs first and can spend handshake_timeout_ms in its
    // handshake, during which the ESP32 queues hundreds more frames.
    //
    // The retries exist because the wrist servos are daisy-chained for power and
    // the one furthest along intermittently stops answering the bus at zero
    // load. That is a hardware fault this cannot fix, but it can stop a
    // momentary dropout taking the whole stack down. Each attempt clears the
    // latched fault and re-enables from a known state, so it is a genuine
    // second try. A persistent fault still fails, three attempts later.
    const char * const who = mega_pass ? "Mega" : "ESP32";
    constexpr int kEnableAttempts = 3;
    bool enabled = false;
    std::string last_reason;

    for (int attempt = 1; attempt <= kEnableAttempts && !enabled; ++attempt) {
      if (mega_pass) {
        rx_.clear();
        text_.clear();
      } else {
        rx_wrist_.clear();
        text_wrist_.clear();
      }
      port.flush_input();

      send_line(port, "c");
      send_line(port, "E");

      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(handshake_timeout_ms_);
      bool faulted = false;
      while (std::chrono::steady_clock::now() < deadline && !enabled && !faulted) {
        if (mega_pass) {
          if (wait_for_mega(std::chrono::milliseconds(200), mega)) {
            enabled = mega.enabled;
            if (mega.fault != 0) {
              last_reason = mega_fault(mega.fault);
              faulted = true;
            }
          }
        } else {
          if (wait_for_wrist(std::chrono::milliseconds(200), wrist)) {
            enabled = wrist.enabled;
            if (wrist.fault != 0) {
              last_reason = wrist_fault(wrist.fault);
              faulted = true;
            }
          }
        }
        feed_both();                              // both, not just this board
      }
      if (!enabled && !faulted) {
        last_reason = "did not report enabled within " +
                      std::to_string(handshake_timeout_ms_) + " ms";
      }
      if (!enabled && attempt < kEnableAttempts) {
        RCLCPP_WARN(
          get_logger(), "%s refused to enable (%s) - attempt %d of %d, retrying",
          who, last_reason.c_str(), attempt, kEnableAttempts);
        // Let a relaxed board settle, and a rebooted servo finish, before
        // asking again - while keeping both boards fed.
        feed_for(std::chrono::milliseconds(400));
        send_line(port, "D");
        feed_for(std::chrono::milliseconds(100));
      }
    }

    if (!enabled) {
      RCLCPP_ERROR(
        get_logger(), "%s refused to enable after %d attempts: %s", who, kEnableAttempts,
        last_reason.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Seed the commands from where the arm actually is. Without this the first
  // write() sends whatever the command interfaces hold - zero - and the arm
  // lunges to its home pose at full speed.
  for (const auto & j : joints_) {
    double q = 0.0;
    if (j.board == Board::Mega) {
      q = j.direction * (mega.measured_deg[j.channel - 1] - j.zero_deg) * kDegToRad;
      last_position_[j.channel - 1] = q;
    } else if (j.board == Board::Esp32) {
      q = j.direction * (static_cast<double>(wrist.position[j.channel - 1]) - j.zero_counts) /
          j.counts_per_rad;
      last_wrist_position_[j.channel - 1] = q;
    } else {
      q = get_state<double>(j.name + "/" + hardware_interface::HW_IF_POSITION);
    }
    // Clamped here too, and the command especially: seeding it from a measured
    // value a hair outside the limits hands the controller an out-of-range
    // setpoint on its first cycle.
    q = reported_position(j, q);
    set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, q);
    set_state(j.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    set_command(j.name + "/" + hardware_interface::HW_IF_POSITION, q);
  }

  last_frame_ = mega;
  last_wrist_ = wrist;
  have_frame_ = uses(Board::Mega);
  have_wrist_ = uses(Board::Esp32);
  last_frame_time_ = last_wrist_time_ = get_clock()->now();
  if (uses(Board::Esp32)) {
    RCLCPP_INFO(
      get_logger(), "arm enabled and holding; servo supply %.1f / %.1f / %.1f V "
      "(lowest %.1f V)",
      wrist.volt[0], wrist.volt[1], wrist.volt[2], wrist.volts_min);
  } else {
    RCLCPP_INFO(get_logger(), "arm enabled and holding");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  send_line(mega_, "D");
  send_line(wrist_, "D");
  RCLCPP_INFO(get_logger(), "arm relaxed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool ArmSystemHardware::send_line(SerialPort & port, const std::string & body)
{
  if (!port.is_open()) {
    return false;
  }
  char suffix[8];
  std::snprintf(suffix, sizeof(suffix), "*%02X\n", xor_checksum(body));
  return port.write_all(body + suffix);
}

bool ArmSystemHardware::parse_mega(MegaFrame & newest)
{
  if (!mega_.read_available(rx_)) {
    return false;
  }
  const bool got = scan_frames(rx_, kMegaFrame, kMegaFrame - 1, [&newest](const uint8_t * f) {
    newest.ms = static_cast<uint32_t>(f[2]) | (static_cast<uint32_t>(f[3]) << 8) |
                (static_cast<uint32_t>(f[4]) << 16) | (static_cast<uint32_t>(f[5]) << 24);
    newest.enabled = f[6] != 0;
    newest.fault = f[7];
    newest.fault_joint = f[8];
    for (int i = 0; i < 3; ++i) {
      newest.measured_deg[i] = read_i16(f + 9 + 2 * i) / 100.0;
      newest.setpoint_deg[i] = read_i16(f + 15 + 2 * i) / 100.0;
      newest.target_deg[i] = read_i16(f + 21 + 2 * i) / 100.0;
      newest.command_us[i] = read_u16(f + 27 + 2 * i);
    }
  }, &text_);
  drain_board_text(text_, "Mega", get_logger());
  return got;
}

bool ArmSystemHardware::parse_wrist(WristFrame & newest)
{
  if (!wrist_.read_available(rx_wrist_)) {
    return false;
  }
  const bool got =
    scan_frames(rx_wrist_, kWristFrame, kWristFrame - 1, [&newest](const uint8_t * f) {
    newest.ms = static_cast<uint32_t>(f[2]) | (static_cast<uint32_t>(f[3]) << 8) |
                (static_cast<uint32_t>(f[4]) << 16) | (static_cast<uint32_t>(f[5]) << 24);
    newest.enabled = f[6] != 0;
    newest.fault = f[7];
    newest.fault_id = f[8];
    for (int i = 0; i < 3; ++i) {
      newest.position[i] = read_u16(f + 9 + 2 * i);
      newest.load[i] = read_i16(f + 15 + 2 * i);
      newest.target[i] = read_u16(f + 21 + 2 * i);
    }
    double lowest = 99.0;
    for (int i = 0; i < 3; ++i) {
      newest.volt[i] = f[27 + i] / 10.0;
      if (newest.volt[i] > 0.05 && newest.volt[i] < lowest) {
        lowest = newest.volt[i];
      }
    }
    newest.volts = (lowest > 98.0) ? 0.0 : lowest;
    newest.volts_min = f[30] / 10.0;
  }, &text_wrist_);
  drain_board_text(text_wrist_, "ESP32", get_logger());
  return got;
}

bool ArmSystemHardware::wait_for_mega(std::chrono::milliseconds timeout, MegaFrame & frame)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (parse_mega(frame)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

bool ArmSystemHardware::wait_for_wrist(std::chrono::milliseconds timeout, WristFrame & frame)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (parse_wrist(frame)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

// A measured position, nudged inside this joint's limits when only just
// outside them. A driver that edits its own sensor readings needs a reason:
//
// MoveIt's CheckStartStateBounds adapter refuses to plan if any start-state
// joint is outside its URDF limits, by any amount, with no tolerance to
// configure. (`fix_start_state` is not that knob - it only normalises
// continuous, planar and floating joints, and a bounded revolute joint still
// aborts the pipeline. Verified against moveit2 2.12.4.) Meanwhile the
// JointTrajectoryController declares a goal reached anywhere inside its goal
// tolerance of 1.5 - 2.6 deg. So a goal AT a limit can settle past it, report
// SUCCEEDED, and leave the arm unable to plan its way out of a position that
// ordinary successful operation put it in - recoverable only by hand.
//
// Of the numbers involved this is the only one that can honestly change. The
// goal tolerance cannot go below the backlash without failing goals on
// backlash alone, and tightening the limits just moves the goal to the new
// limit. Inside kReportClamp the clamp is smaller than this arm's measurement
// noise, so the clamped value is no less true than the raw one; outside it the
// raw value is reported and the planner still stops.
double ArmSystemHardware::reported_position(const JointMap & j, double q)
{
  const double lo = j.limit_lower;
  const double hi = j.limit_upper;
  if (q >= lo && q <= hi) {
    return q;
  }
  const double over = (q < lo) ? (lo - q) : (q - hi);
  if (over <= kReportClamp) {
    // Not logged. It is expected, it is within the noise floor, and it happens
    // at 100 Hz for as long as the arm sits there.
    return std::clamp(q, lo, hi);
  }
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "joint '%s' is %.2f deg outside its limits (%.4f rad, limits %.4f..%.4f) - too far to treat "
    "as measurement noise, so the TRUE value is being reported and MoveIt will refuse to plan. "
    "Move this joint back inside its range by hand, with torque off.",
    j.name.c_str(), over * 180.0 / M_PI, q, lo, hi);
  return q;
}

hardware_interface::return_type ArmSystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = std::max(1e-3, period.seconds());
  bool faulted = false;

  MegaFrame mega;
  if (uses(Board::Mega) && parse_mega(mega)) {
    for (const auto & j : joints_) {
      if (j.board != Board::Mega) {
        continue;
      }
      const int idx = j.channel - 1;
      const double q =
        reported_position(j, j.direction * (mega.measured_deg[idx] - j.zero_deg) * kDegToRad);
      set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, q);
      set_state(
        j.name + "/" + hardware_interface::HW_IF_VELOCITY, (q - last_position_[idx]) / dt);
      last_position_[idx] = q;
    }
    last_frame_ = mega;
    last_frame_time_ = time;
    if (mega.fault != 0) {
      if (!fault_reported_) {
        RCLCPP_ERROR(
          get_logger(), "Mega fault on joint %u: %s - every servo relaxed", mega.fault_joint,
          mega_fault(mega.fault));
      }
      faulted = true;
    }
  }

  WristFrame wrist;
  if (uses(Board::Esp32) && parse_wrist(wrist)) {
    for (const auto & j : joints_) {
      if (j.board != Board::Esp32) {
        continue;
      }
      const int idx = j.channel - 1;
      const double q = reported_position(
        j, j.direction * (static_cast<double>(wrist.position[idx]) - j.zero_counts) /
             j.counts_per_rad);
      set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, q);
      set_state(
        j.name + "/" + hardware_interface::HW_IF_VELOCITY, (q - last_wrist_position_[idx]) / dt);
      last_wrist_position_[idx] = q;
    }
    last_wrist_ = wrist;
    last_wrist_time_ = time;

    if (supply_log_period_ > 0.0 &&
        (time - last_supply_log_).seconds() > supply_log_period_) {
      last_supply_log_ = time;
      RCLCPP_INFO(
        get_logger(), "SUPPLY %.1f %.1f %.1f V | LOAD %d %d %d | min %.1f V",
        wrist.volt[0], wrist.volt[1], wrist.volt[2],
        wrist.load[0], wrist.load[1], wrist.load[2], wrist.volts_min);

      // The wrist's positions, in counts and degrees, so a START_STATE_INVALID
      // can be attributed to a joint. Both units because the failure modes read
      // differently: a joint outside its limit shows in the degrees, one sitting
      // on the 0/4095 seam only in the counts.
      {
        char buf[192];
        int n = 0;
        for (int idx = 0; idx < 3 && n < static_cast<int>(sizeof(buf)) - 1; ++idx) {
          // Indexed by `channel - 1`, the same mapping read() uses above, so
          // these are the angles ros2_control publishes - not a second,
          // differently-derived number.
          const JointMap * j = nullptr;
          for (const auto & cand : joints_) {
            if (cand.board == Board::Esp32 && cand.channel - 1 == idx) {
              j = &cand;
              break;
            }
          }
          const double deg = (j == nullptr)
            ? 0.0
            : j->direction * (static_cast<double>(wrist.position[idx]) - j->zero_counts) /
                j->counts_per_rad * 180.0 / M_PI;
          n += snprintf(
            buf + n, sizeof(buf) - static_cast<size_t>(n), "%s%u ct/%+.1f deg",
            idx ? "  " : "", wrist.position[idx], deg);
        }
        RCLCPP_INFO(get_logger(), "WRIST %s", buf);
      }

      // The Mega side too, because the DS51150s report no voltage and an abort
      // cannot otherwise say whether a lagging joint is a control problem or a
      // power one.
      //
      // ERR is the column that matters: `set - meas`, the firmware's tracking
      // error against its own approach setpoint, which is what FAULT_ERR_DEG
      // trips on. It is not the host-trajectory error, which legitimately
      // contains up to 8 deg of designed approach lag. A large ERR means the
      // servo is not reaching where it is told, which on a position-commanded
      // servo means torque or volts.
      //
      // `us` carries no effort information: us = 1500 + deg * 7.4834 reproduces
      // every sample to under 2 us, so it is the position command re-expressed.
      // The Mega has no current sensing, so there is no effort signal here.
      RCLCPP_INFO(
        get_logger(),
        // `tgt` is the host's commanded value as the firmware received it; `set`
        // is the firmware's setpoint after its approach filter. Both are needed
        // to tell a bad host command from a misbehaving firmware loop.
        "MEGA  meas %+7.2f %+7.2f %+7.2f | set %+7.2f %+7.2f %+7.2f | "
        "tgt %+7.2f %+7.2f %+7.2f | ERR %+6.2f %+6.2f %+6.2f deg | us %4u %4u %4u",
        last_frame_.measured_deg[0], last_frame_.measured_deg[1],
        last_frame_.measured_deg[2], last_frame_.setpoint_deg[0],
        last_frame_.setpoint_deg[1], last_frame_.setpoint_deg[2],
        last_frame_.target_deg[0], last_frame_.target_deg[1],
        last_frame_.target_deg[2],
        last_frame_.setpoint_deg[0] - last_frame_.measured_deg[0],
        last_frame_.setpoint_deg[1] - last_frame_.measured_deg[1],
        last_frame_.setpoint_deg[2] - last_frame_.measured_deg[2],
        last_frame_.command_us[0], last_frame_.command_us[1],
        last_frame_.command_us[2]);
    }

    // A sagging supply is the leading explanation for a servo dropping off the
    // bus mid-move: brown an ST3215 out and it reboots, which looks identical
    // to "stopped answering". 10.5 V is 3.5 V/cell on a 3S pack. Reported once
    // per 0.1 V of further sag, not at the 100 Hz this runs at.
    if (wrist.volts > 0.1 && wrist.volts < 10.5 && wrist.volts < volts_warned_ - 0.1) {
      volts_warned_ = wrist.volts;
      RCLCPP_WARN(
        get_logger(),
        "servo supply sagging: id1 %.1f V / load %d   id2 %.1f V / load %d   "
        "id3 %.1f V / load %d   (lowest seen %.1f V). Load is 0-1000 of stall. "
        "A servo whose volts drop while its load stays comparable to its "
        "neighbours' is being starved by a resistive supply; one whose LOAD is "
        "also much higher is drawing the sag itself - a bind, not a bad crimp.",
        wrist.volt[0], wrist.load[0], wrist.volt[1], wrist.load[1],
        wrist.volt[2], wrist.load[2], wrist.volts_min);
    }

    if (wrist.fault != 0) {
      if (!fault_reported_) {
        RCLCPP_ERROR(
          get_logger(), "ESP32 fault on servo %u: %s - wrist torque off", wrist.fault_id,
          wrist_fault(wrist.fault));
        // For an undervoltage fault, say where the volts went, from the
        // gradient across the three servos - one servo holding 12.6 V while the
        // others collapse means the supply is fine and the loss is in the chain.
        if (wrist.fault == 5) {
          const double hi = std::max({wrist.volt[0], wrist.volt[1], wrist.volt[2]});
          const double lo = std::min({wrist.volt[0], wrist.volt[1], wrist.volt[2]});
          RCLCPP_ERROR(
            get_logger(), "  id1 %.1f V  id2 %.1f V  id3 %.1f V - spread %.1f V. %s",
            wrist.volt[0], wrist.volt[1], wrist.volt[2], hi - lo,
            (hi - lo) < 1.0
              ? "All three fell together, so the loss is UPSTREAM of the chain: "
                "the pack, its main lead, or a shared connector."
              : "They did NOT fall together, so the supply is sound and the loss "
                "is INSIDE the chain. The resistance is in the link shared by the "
                "servos that dropped - check the segment and connectors feeding "
                "them from the one that held its voltage.");
        }
      }
      faulted = true;
    }
  }

  // Loopback joints: state follows command, so everything above behaves as it
  // will once their hardware exists.
  for (const auto & j : joints_) {
    if (j.board == Board::Loopback) {
      const double c = get_command<double>(j.name + "/" + hardware_interface::HW_IF_POSITION);
      // Command interfaces start as NaN and a controller may not have written
      // one yet. Following it blindly puts a NaN in the TF tree and RViz drops
      // the link.
      if (std::isfinite(c)) {
        set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, c);
      }
      set_state(j.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    }
  }

  if (faulted) {
    fault_reported_ = true;
    // Only an active component is stopped by a board fault. ros2_control reads
    // inactive hardware too, and both boards latch their fault until the 'c'
    // that only on_activate sends - so a fault left from a previous session is
    // reported on the first read after configure and would deactivate the
    // component before activation could clear it. While inactive nothing is
    // energised, so there is nothing to stop; if the fault is still real,
    // on_activate refuses to enable and quotes the board's own words.
    if (get_lifecycle_state().label() == hardware_interface::lifecycle_state_names::ACTIVE) {
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_ONCE(
      get_logger(),
      "  the fault above was latched from before this stack started (component is "
      "'%s', nothing is energised). Activation clears it and will refuse to "
      "enable if it is still real.",
      get_lifecycle_state().label().c_str());
  }

  if (have_frame_ && (time - last_frame_time_).seconds() > stale_timeout_s_) {
    RCLCPP_ERROR(get_logger(), "no status from the Mega for %.0f ms - link lost",
      (time - last_frame_time_).seconds() * 1000.0);
    return hardware_interface::return_type::ERROR;
  }
  if (have_wrist_ && (time - last_wrist_time_).seconds() > stale_timeout_s_) {
    RCLCPP_ERROR(get_logger(), "no status from the ESP32 for %.0f ms - link lost",
      (time - last_wrist_time_).seconds() * 1000.0);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  double deg[3] = {0.0, 0.0, 0.0};
  long counts[3] = {2048, 2048, 2048};

  for (const auto & j : joints_) {
    if (j.board == Board::Loopback) {
      continue;
    }
    const double q = get_command<double>(j.name + "/" + hardware_interface::HW_IF_POSITION);
    if (!std::isfinite(q)) {
      return hardware_interface::return_type::OK;   // controller not writing yet
    }
    if (j.board == Board::Mega) {
      deg[j.channel - 1] = j.zero_deg + j.direction * q * kRadToDeg;
    } else {
      counts[j.channel - 1] =
        std::lround(j.zero_counts + j.direction * q * j.counts_per_rad);
    }
  }

  char body[64];
  if (uses(Board::Mega)) {
    std::snprintf(body, sizeof(body), "G %.2f %.2f %.2f", deg[0], deg[1], deg[2]);
    if (!send_line(mega_, body)) {
      RCLCPP_ERROR(get_logger(), "could not write to the Mega - link lost");
      return hardware_interface::return_type::ERROR;
    }
  }
  if (uses(Board::Esp32)) {
    std::snprintf(body, sizeof(body), "G %ld %ld %ld", counts[0], counts[1], counts[2]);
    if (!send_line(wrist_, body)) {
      RCLCPP_ERROR(get_logger(), "could not write to the ESP32 - link lost");
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmSystemHardware, hardware_interface::SystemInterface)
