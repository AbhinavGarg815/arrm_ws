// ros2_control hardware interface for the arm.
//
// The firmware works in each servo's own frame (degrees or counts, zero at
// mid-travel); ROS works in URDF joint frames (radians, zero at the upright
// home pose). The mapping lives here, driven by actuators.yaml via per-joint
// <param> entries in the xacro:
//
//     q_urdf   = direction * (deg_servo - zero_deg) * pi/180
//     deg_servo = zero_deg + direction * q_urdf * 180/pi
//
// Never fix a reversed joint by flipping an axis in the URDF - that
// invalidates the geometry, the collision matrix and every named pose.
//
// Two boards: the Mega (arm3) speaks degrees for the three DS51150 joints, the
// ESP32 (st3215_arm) speaks counts for the three ST3215 wrist joints. Both are
// safe at boot, checksum their commands, stream a binary status frame and relax
// on a dead-man. The gripper has no hardware and runs in loopback.

#ifndef ARM_HARDWARE__ARM_SYSTEM_HARDWARE_HPP_
#define ARM_HARDWARE__ARM_SYSTEM_HARDWARE_HPP_

#include <chrono>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "arm_hardware/serial_port.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arm_hardware
{

/// One status frame from the Mega's arm3 firmware: 34 bytes, little-endian.
struct MegaFrame
{
  uint32_t ms{0};
  bool enabled{false};
  uint8_t fault{0};
  uint8_t fault_joint{0};
  double measured_deg[3]{};
  double setpoint_deg[3]{};
  double target_deg[3]{};
  uint16_t command_us[3]{};
};

/// One status frame from the ESP32's st3215_arm firmware: 28 bytes.
struct WristFrame
{
  uint32_t ms{0};
  bool enabled{false};
  uint8_t fault{0};
  uint8_t fault_id{0};
  uint16_t position[3]{};
  int16_t load[3]{};
  uint16_t target[3]{};
  double volt[3]{};        ///< each servo's own supply reading, now
  double volts{0.0};       ///< lowest of the three, for the threshold checks
  double volts_min{0.0};   ///< lowest any of them has seen since the board booted
};

class ArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class Board { Mega, Esp32, Loopback };

  struct JointMap
  {
    std::string name;
    Board board{Board::Loopback};
    int channel{0};          ///< Mega: joint index 1..3. ESP32: index 0..2 by id.
    double direction{1.0};   ///< +1 or -1
    double zero_deg{0.0};    ///< Mega: servo degrees at q = 0
    double zero_counts{2048.0};    ///< ESP32: servo counts at q = 0
    double counts_per_rad{651.899};///< ESP32: 4096 / 2pi
    /// What the joint reports before anything has measured or commanded it.
    /// The mock plugin applies this itself; a real component must honour it.
    double initial_position{0.0};
    /// Position limits from the URDF command_interface min/max, i.e. from
    /// joint_parameters.yaml. Used by reported_position().
    double limit_lower{-std::numeric_limits<double>::infinity()};
    double limit_upper{+std::numeric_limits<double>::infinity()};
  };

  /// How far outside its limits a measured position may be pulled back in.
  /// 0.05 rad = 2.9 deg, which is >= every joint's goal tolerance (0.026 -
  /// 0.046 rad) and >= every joint's backlash (0.52 - 1.08 deg). Both bounds
  /// matter: the controller may call a goal reached anywhere inside its
  /// tolerance, so a goal at a limit can settle outside it and leave the arm
  /// unplannable; and a violation smaller than the backlash is noise, not a
  /// physical fact. Safe only because every limit sits >= 5 deg inside the
  /// nearest hard boundary, so a fully clamped reading is still 2.1 deg clear.
  /// The firmware's soft limits act on the TRUE angle and back this up.
  static constexpr double kReportClamp = 0.05;

  /// A measured position, pulled inside the limits if barely outside them.
  /// Beyond kReportClamp the true value is reported, so a real excursion (a
  /// seam wrap, a slipped coupling) still stops the planner.
  double reported_position(const JointMap & j, double q);

  /// Pull complete, checksum-valid frames out of a buffer and keep the newest.
  bool parse_mega(MegaFrame & newest);
  bool parse_wrist(WristFrame & newest);
  /// Send one line with the XOR checksum both firmwares expect.
  bool send_line(SerialPort & port, const std::string & body);
  /// Block (briefly) until a frame arrives, or the deadline passes.
  bool wait_for_mega(std::chrono::milliseconds timeout, MegaFrame & frame);
  bool wait_for_wrist(std::chrono::milliseconds timeout, WristFrame & frame);
  /// True if this hardware has any joint on that board.
  bool uses(Board board) const;

  std::vector<JointMap> joints_;
  SerialPort mega_;
  SerialPort wrist_;
  std::vector<uint8_t> rx_;
  std::vector<uint8_t> rx_wrist_;
  // Plain-text diagnostics the boards print between their binary frames, held
  // across reads because a line can arrive split across two. These carry the
  // only statement of why an enable was refused ("!! refusing: id 1 not
  // answering"). See drain_board_text().
  std::string text_;
  std::string text_wrist_;

  std::string mega_port_{"/dev/ttyACM0"};
  std::string wrist_port_{"/dev/ttyUSB0"};
  int mega_baud_{250000};
  int wrist_baud_{115200};
  int status_hz_{100};
  int handshake_timeout_ms_{3000};
  double stale_timeout_s_{0.25};

  MegaFrame last_frame_{};
  WristFrame last_wrist_{};
  bool have_frame_{false};
  bool have_wrist_{false};
  rclcpp::Time last_frame_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_wrist_time_{0, 0, RCL_STEADY_TIME};
  double last_position_[3]{};
  double last_wrist_position_[3]{};
  bool fault_reported_{false};
  double volts_warned_{99.0};  ///< so a sagging supply is reported once, not 100x/s

  /// Periodic per-servo supply/load logging, for chasing where a sag comes
  /// from. Off unless ARM_LOG_SUPPLY is set in the environment.
  double supply_log_period_{0.0};
  rclcpp::Time last_supply_log_{0, 0, RCL_STEADY_TIME};
};

}  // namespace arm_hardware

#endif  // ARM_HARDWARE__ARM_SYSTEM_HARDWARE_HPP_
