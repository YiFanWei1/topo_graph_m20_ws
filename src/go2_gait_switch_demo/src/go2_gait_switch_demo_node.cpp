#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/go2/sport/sport_client.hpp"

namespace
{

constexpr double kMaximumYawRate = 0.5;
constexpr int kMaximumCycles = 2;
constexpr int32_t kLegacySwitchGaitApiId = 1011;

enum class Stage
{
  kStartup,
  kGaitSettle,
  kRotate,
  kStopGap,
  kDone,
};

}  // namespace

// Go2 firmware before Motion Control 2.0 exposes gait selection through
// SwitchGait(), API 1011. The current SDK removed the public wrapper, so keep
// the compatibility call local to this demo. Unsupported firmware returns a
// non-zero SDK code and the sequence aborts before sending Move().
class Go2SportClientCompat : public unitree::robot::go2::SportClient
{
public:
  void InitWithLegacyGaitApi()
  {
    Init();
    RegistApi(kLegacySwitchGaitApiId, 0);
  }

  int32_t SwitchGait(const int gait_type)
  {
    return Call(
      kLegacySwitchGaitApiId,
      std::string("{\"data\":") + std::to_string(gait_type) + "}");
  }
};

class Go2GaitSwitchDemo : public rclcpp::Node
{
public:
  Go2GaitSwitchDemo()
  : Node("go2_gait_switch_demo")
  {
    network_interface_ = declare_parameter<std::string>("network_interface", "eth0");
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    yaw_rate_ = declare_parameter<double>("yaw_rate", 0.5);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    cycles_ = declare_parameter<int>("cycles", 2);
    startup_delay_sec_ = declare_parameter<double>("startup_delay_sec", 5.0);
    gait_settle_sec_ = declare_parameter<double>("gait_settle_sec", 2.0);
    rotate_duration_sec_ = declare_parameter<double>("rotate_duration_sec", 2.0);
    stop_gap_sec_ = declare_parameter<double>("stop_gap_sec", 2.0);
    sdk_timeout_sec_ = declare_parameter<double>("sdk_timeout_sec", 2.0);

    validateParameters();

    emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/emergency_stop",
      std::bind(
        &Go2GaitSwitchDemo::emergencyStopCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    if (enable_motion_) {
      RCLCPP_WARN(
        get_logger(),
        "REAL MOTION ARMED: vx=0, vy=0, wz=%.3f rad/s. Keep the remote emergency stop ready.",
        yaw_rate_);
      unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
      sport_client_ = std::make_unique<Go2SportClientCompat>();
      sport_client_->SetTimeout(static_cast<float>(sdk_timeout_sec_));
      sport_client_->InitWithLegacyGaitApi();
      if (!stopRobot("startup safety stop", true)) {
        throw std::runtime_error("Go2 rejected the startup StopMove request");
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "DRY RUN ONLY: enable_motion is false; no Unitree SDK connection or motion command will be made.");
    }

    stage_deadline_ = now() + rclcpp::Duration::from_seconds(startup_delay_sec_);
    const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&Go2GaitSwitchDemo::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Sequence starts in %.1f s: StaticWalk -> SwitchGait(3), %d cycles, %.1f s rotation per gait.",
      startup_delay_sec_, cycles_, rotate_duration_sec_);
    RCLCPP_INFO(
      get_logger(),
      "Emergency stop: ros2 service call %s std_srvs/srv/Trigger {}",
      emergency_stop_service_->get_service_name());
  }

  ~Go2GaitSwitchDemo() override
  {
    stopRobot("node shutdown", false);
  }

private:
  void validateParameters()
  {
    if (network_interface_.empty()) {
      throw std::invalid_argument("network_interface must not be empty");
    }
    if (!std::isfinite(yaw_rate_) || std::abs(yaw_rate_) < 1e-6) {
      throw std::invalid_argument("yaw_rate must be finite and non-zero");
    }
    if (std::abs(yaw_rate_) > kMaximumYawRate + 1e-9) {
      throw std::invalid_argument("|yaw_rate| must not exceed the hard safety limit 0.5 rad/s");
    }
    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ < 5.0 || control_rate_hz_ > 50.0) {
      throw std::invalid_argument("control_rate_hz must be in [5, 50]");
    }
    if (cycles_ < 1 || cycles_ > kMaximumCycles) {
      throw std::invalid_argument("cycles must be 1 or 2");
    }
    validateDuration("startup_delay_sec", startup_delay_sec_, 1.0, 30.0);
    validateDuration("gait_settle_sec", gait_settle_sec_, 0.2, 10.0);
    validateDuration("rotate_duration_sec", rotate_duration_sec_, 0.2, 5.0);
    validateDuration("stop_gap_sec", stop_gap_sec_, 0.2, 10.0);
    validateDuration("sdk_timeout_sec", sdk_timeout_sec_, 0.2, 5.0);
  }

  static void validateDuration(
    const std::string & name, const double value, const double minimum, const double maximum)
  {
    if (!std::isfinite(value) || value < minimum || value > maximum) {
      throw std::invalid_argument(
              name + " must be in [" + std::to_string(minimum) + ", " +
              std::to_string(maximum) + "] seconds");
    }
  }

  void timerCallback()
  {
    if (stage_ == Stage::kDone || now() < stage_deadline_) {
      if (stage_ == Stage::kRotate) {
        sendRotationCommand();
      }
      return;
    }

    switch (stage_) {
      case Stage::kStartup:
        beginGaitPhase();
        break;
      case Stage::kGaitSettle:
        stage_ = Stage::kRotate;
        stage_deadline_ = now() + rclcpp::Duration::from_seconds(rotate_duration_sec_);
        RCLCPP_INFO(
          get_logger(), "Rotating with %s: vx=0, vy=0, wz=%.3f rad/s for %.1f s.",
          currentGaitName().c_str(), yaw_rate_, rotate_duration_sec_);
        break;
      case Stage::kRotate:
        finishGaitPhase();
        break;
      case Stage::kStopGap:
        ++phase_index_;
        if (phase_index_ >= cycles_ * 2) {
          finishSequence("completed two-gait sequence");
        } else {
          beginGaitPhase();
        }
        break;
      case Stage::kDone:
        break;
    }
  }

  void beginGaitPhase()
  {
    if (!stopRobot("before gait switch", true)) {
      abortSequence("StopMove failed before gait switch");
      return;
    }

    const bool use_static_walk = (phase_index_ % 2) == 0;
    const int return_code = callSdk([this, use_static_walk]() {
        return use_static_walk ? sport_client_->StaticWalk() : sport_client_->SwitchGait(3);
      });
    if (return_code != 0) {
      RCLCPP_ERROR(
        get_logger(), "%s failed, SDK return code: %d", currentGaitName().c_str(), return_code);
      abortSequence("gait switch failed");
      return;
    }

    const int cycle_number = phase_index_ / 2 + 1;
    RCLCPP_INFO(
      get_logger(), "Cycle %d/%d: switched to %s; settling for %.1f s.",
      cycle_number, cycles_, currentGaitName().c_str(), gait_settle_sec_);
    stage_ = Stage::kGaitSettle;
    stage_deadline_ = now() + rclcpp::Duration::from_seconds(gait_settle_sec_);
  }

  void sendRotationCommand()
  {
    const int return_code = callSdk([this]() {
        return sport_client_->Move(0.0F, 0.0F, static_cast<float>(yaw_rate_));
      });
    if (return_code != 0) {
      RCLCPP_ERROR(get_logger(), "Move failed, SDK return code: %d", return_code);
      abortSequence("rotation command failed");
    }
  }

  void finishGaitPhase()
  {
    if (!stopRobot("end of gait phase", true)) {
      abortSequence("StopMove failed after rotation");
      return;
    }
    RCLCPP_INFO(get_logger(), "%s rotation finished; robot stopped.", currentGaitName().c_str());
    stage_ = Stage::kStopGap;
    stage_deadline_ = now() + rclcpp::Duration::from_seconds(stop_gap_sec_);
  }

  std::string currentGaitName() const
  {
    return (phase_index_ % 2) == 0 ? "StaticWalk" : "ClimbStair(SwitchGait=3)";
  }

  template<typename FunctionT>
  int callSdk(FunctionT && function)
  {
    if (!enable_motion_) {
      return 0;
    }
    if (!sport_client_) {
      return -1;
    }
    try {
      return function();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Unitree SDK exception: %s", exception.what());
      return -1;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown Unitree SDK exception");
      return -1;
    }
  }

  bool stopRobot(const std::string & reason, const bool report_failure)
  {
    if (!enable_motion_ || !sport_client_) {
      return true;
    }
    try {
      const int return_code = sport_client_->StopMove();
      if (return_code != 0 && report_failure) {
        RCLCPP_ERROR(
          get_logger(), "StopMove failed during '%s', SDK return code: %d",
          reason.c_str(), return_code);
      }
      return return_code == 0;
    } catch (...) {
      if (report_failure) {
        RCLCPP_ERROR(get_logger(), "StopMove threw during '%s'", reason.c_str());
      }
      return false;
    }
  }

  void abortSequence(const std::string & reason)
  {
    if (stage_ == Stage::kDone) {
      return;
    }
    RCLCPP_ERROR(get_logger(), "Sequence aborted: %s", reason.c_str());
    finishSequence("aborted");
  }

  void finishSequence(const std::string & reason)
  {
    stage_ = Stage::kDone;
    for (int attempt = 0; attempt < 3; ++attempt) {
      stopRobot("final redundant stop", false);
    }
    RCLCPP_WARN(get_logger(), "Control stopped: %s. No more Move commands will be sent.", reason.c_str());
    if (timer_) {
      timer_->cancel();
    }
    rclcpp::shutdown();
  }

  void emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    stopRobot("emergency stop service", false);
    response->success = true;
    response->message = "StopMove sent; sequence is terminating";
    abortSequence("emergency stop service called");
  }

  std::string network_interface_;
  bool enable_motion_{false};
  double yaw_rate_{0.5};
  double control_rate_hz_{20.0};
  int cycles_{2};
  double startup_delay_sec_{5.0};
  double gait_settle_sec_{2.0};
  double rotate_duration_sec_{2.0};
  double stop_gap_sec_{2.0};
  double sdk_timeout_sec_{2.0};

  int phase_index_{0};
  Stage stage_{Stage::kStartup};
  rclcpp::Time stage_deadline_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<Go2SportClientCompat> sport_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Go2GaitSwitchDemo>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_gait_switch_demo"), "%s", exception.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
