#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <route3d_pid_controller/msg/gait_transition.hpp>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace route3d_go2_adapter
{
namespace
{

using GaitTransition = route3d_pid_controller::msg::GaitTransition;
constexpr std::int32_t kLegacySwitchGaitApiId = 1011;

class Go2SportClientCompat : public unitree::robot::go2::SportClient
{
public:
  void initWithLegacyGaitApi()
  {
    Init();
    RegistApi(kLegacySwitchGaitApiId, 0);
  }

  std::int32_t switchGait(const int gait_type)
  {
    return Call(
      kLegacySwitchGaitApiId,
      std::string("{\"data\":") + std::to_string(gait_type) + "}");
  }
};

enum class AdapterState
{
  kIdle,
  kStopping,
  kPreSwitchDelay,
  kRetryWaiting,
  kSettling,
  kPublishingAcknowledgement,
  kError,
  kEmergencyStopped,
};

const char * stateName(const AdapterState state)
{
  switch (state) {
    case AdapterState::kIdle: return "IDLE";
    case AdapterState::kStopping: return "STOPPING";
    case AdapterState::kPreSwitchDelay: return "PRE_SWITCH_DELAY";
    case AdapterState::kRetryWaiting: return "RETRY_WAITING";
    case AdapterState::kSettling: return "SETTLING";
    case AdapterState::kPublishingAcknowledgement: return "PUBLISHING_ACKNOWLEDGEMENT";
    case AdapterState::kError: return "ERROR";
    case AdapterState::kEmergencyStopped: return "EMERGENCY_STOPPED";
  }
  return "UNKNOWN";
}

bool sameRequest(const GaitTransition & lhs, const GaitTransition & rhs)
{
  return lhs.route_sequence == rhs.route_sequence && lhs.task_index == rhs.task_index &&
         lhs.gait_command == rhs.gait_command;
}

bool supportedGaitCommand(const std::string & command)
{
  return command == "static_walk" || command == "switch_gait_3";
}

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

}  // namespace

class Go2RouteAdapterNode : public rclcpp::Node
{
public:
  Go2RouteAdapterNode()
  : Node("route3d_go2_adapter")
  {
    network_interface_ = declare_parameter<std::string>("network_interface", "eth0");
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    const auto legacy_command_topic = declare_parameter<std::string>(
      "topics.command", "/cmd_vel_pid");
    pid_command_topic_ = declare_parameter<std::string>(
      "topics.pid_command", legacy_command_topic);
    efficient_command_topic_ = declare_parameter<std::string>(
      "topics.efficient_command", "/cmd_vel_effi");
    controller_source_topic_ = declare_parameter<std::string>(
      "topics.controller_source", "/route3d_controller/active_source");
    selected_command_topic_ = declare_parameter<std::string>(
      "topics.selected_command", "/route3d_go2_adapter/selected_command");
    transition_topic_ = declare_parameter<std::string>(
      "topics.gait_transition", "/route3d_pid_controller/gait_transition");
    gait_acknowledged_topic_ = declare_parameter<std::string>(
      "topics.gait_acknowledged", "/route3d_pid_controller/gait_acknowledged");
    status_topic_ = declare_parameter<std::string>(
      "topics.status", "/route3d_go2_adapter/status");
    send_rate_hz_ = declare_parameter<double>("command.send_rate_hz", 30.0);
    command_timeout_s_ = declare_parameter<double>("command.timeout_s", 0.10);
    maximum_vx_mps_ = declare_parameter<double>("command.maximum_vx_mps", 0.80);
    maximum_vy_mps_ = declare_parameter<double>("command.maximum_vy_mps", 0.40);
    maximum_wz_radps_ = declare_parameter<double>("command.maximum_wz_radps", 1.20);
    gait_pre_switch_stop_delay_s_ = declare_parameter<double>(
      "gait.pre_switch_stop_delay_s", 1.0);
    gait_settle_time_s_ = declare_parameter<double>("gait.settle_time_s", 2.0);
    transition_timeout_s_ = declare_parameter<double>("gait.transition_timeout_s", 8.0);
    gait_command_max_retries_ = declare_parameter<std::int64_t>(
      "gait.command_max_retries", 3);
    gait_command_retry_initial_delay_s_ = declare_parameter<double>(
      "gait.command_retry_initial_delay_s", 1.0);
    gait_command_retry_backoff_multiplier_ = declare_parameter<double>(
      "gait.command_retry_backoff_multiplier", 2.0);
    require_state_feedback_ = declare_parameter<bool>("gait.require_state_feedback", true);
    sport_state_topic_ = declare_parameter<std::string>(
      "gait.state_topic", "rt/sportmodestate");
    state_timeout_s_ = declare_parameter<double>("gait.state_timeout_s", 1.0);
    minimum_stable_samples_ = static_cast<std::size_t>(declare_parameter<std::int64_t>(
      "gait.minimum_stable_samples", 5));
    sdk_timeout_s_ = declare_parameter<double>("sdk.timeout_s", 10.0);
    validateParameters();

    pid_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      pid_command_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        commandCallback("pid", message, pid_command_);
      });
    efficient_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      efficient_command_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        commandCallback("efficient_3d_local_planner", message, efficient_command_);
      });
    controller_source_subscription_ = create_subscription<std_msgs::msg::String>(
      controller_source_topic_, latchedQos(),
      std::bind(&Go2RouteAdapterNode::controllerSourceCallback, this, std::placeholders::_1));
    transition_subscription_ = create_subscription<GaitTransition>(
      transition_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&Go2RouteAdapterNode::transitionCallback, this, std::placeholders::_1));
    gait_acknowledged_publisher_ = create_publisher<GaitTransition>(
      gait_acknowledged_topic_, latchedQos());
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, latchedQos());
    selected_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      selected_command_topic_, rclcpp::QoS(10).reliable());
    emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/emergency_stop", std::bind(
        &Go2RouteAdapterNode::emergencyStopCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    retry_service_ = create_service<std_srvs::srv::Trigger>(
      "~/retry_gait_transition", std::bind(
        &Go2RouteAdapterNode::retryCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    if (enable_motion_) {
      unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
      sport_client_ = std::make_unique<Go2SportClientCompat>();
      sport_client_->SetTimeout(static_cast<float>(sdk_timeout_s_));
      sport_client_->initWithLegacyGaitApi();
      sport_state_subscription_ =
        std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
        sport_state_topic_);
      sport_state_subscription_->InitChannel(
        std::bind(&Go2RouteAdapterNode::sportStateCallback, this, std::placeholders::_1), 10);
      // The deployed Go2 accepts SportClient::Move() but may return -1 for
      // StopMove() while the sport service is still being discovered.  The
      // legacy adapter did not make startup conditional on StopMove(), so do
      // not tear down the only command consumer here.  The timer below keeps
      // sending a zero Move command until a controller is explicitly selected.
      if (!stopRobot("startup")) {
        RCLCPP_WARN(
          get_logger(),
          "Startup stop was not acknowledged; continuing with zero Move commands");
      }
      RCLCPP_WARN(
        get_logger(), "REAL GO2 OUTPUT ENABLED on %s; this node owns velocity and gait SDK calls",
        network_interface_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "DRY RUN: gait transitions are simulated and acknowledged; no Unitree SDK is opened");
    }

    const auto period = std::chrono::duration<double>(1.0 / send_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Go2RouteAdapterNode::timerCallback, this));
    setState(AdapterState::kIdle, "adapter ready");
  }

  ~Go2RouteAdapterNode() override
  {
    if (sport_state_subscription_) {
      sport_state_subscription_->CloseChannel();
      sport_state_subscription_.reset();
    }
    if (sport_client_) {
      (void)stopRobot("shutdown");
      sport_client_.reset();
      unitree::robot::ChannelFactory::Instance()->Release();
    }
  }

private:
  void validateParameters() const
  {
    const auto finite_positive = [](const double value) {
        return std::isfinite(value) && value > 0.0;
      };
    if (network_interface_.empty() || pid_command_topic_.empty() ||
      efficient_command_topic_.empty() || controller_source_topic_.empty() ||
      selected_command_topic_.empty() || transition_topic_.empty() ||
      gait_acknowledged_topic_.empty() || sport_state_topic_.empty())
    {
      throw std::invalid_argument("adapter topic and network names must not be empty");
    }
    if (!finite_positive(send_rate_hz_) || send_rate_hz_ > 100.0 ||
      !finite_positive(command_timeout_s_) || !finite_positive(maximum_vx_mps_) ||
      !finite_positive(maximum_vy_mps_) || !finite_positive(maximum_wz_radps_) ||
      !std::isfinite(gait_pre_switch_stop_delay_s_) || gait_pre_switch_stop_delay_s_ < 0.0 ||
      !finite_positive(gait_settle_time_s_) || !finite_positive(transition_timeout_s_) ||
      gait_command_max_retries_ < 0 ||
      !finite_positive(gait_command_retry_initial_delay_s_) ||
      !std::isfinite(gait_command_retry_backoff_multiplier_) ||
      gait_command_retry_backoff_multiplier_ < 1.0 ||
      !finite_positive(state_timeout_s_) || !finite_positive(sdk_timeout_s_) ||
      gait_pre_switch_stop_delay_s_ + gait_settle_time_s_ >= transition_timeout_s_ ||
      minimum_stable_samples_ == 0U)
    {
      throw std::invalid_argument("invalid Go2 adapter timing, rate, limit, or sample parameter");
    }
  }

  struct CommandState
  {
    bool available{false};
    float vx{0.0F};
    float vy{0.0F};
    float wz{0.0F};
    std::chrono::steady_clock::time_point received{};
  };

  void commandCallback(
    const std::string & source, const geometry_msgs::msg::Twist::ConstSharedPtr message,
    CommandState & command)
  {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->linear.y) ||
      !std::isfinite(message->angular.z))
    {
      failTransition("received non-finite velocity command from '" + source + "'");
      return;
    }
    double vx = std::clamp(message->linear.x, -maximum_vx_mps_, maximum_vx_mps_);
    double vy = std::clamp(message->linear.y, -maximum_vy_mps_, maximum_vy_mps_);
    const double planar_speed = std::hypot(vx, vy);
    if (planar_speed > maximum_vx_mps_) {
      const double scale = maximum_vx_mps_ / planar_speed;
      vx *= scale;
      vy *= scale;
    }
    command.vx = static_cast<float>(vx);
    command.vy = static_cast<float>(vy);
    command.wz = static_cast<float>(std::clamp(
      message->angular.z, -maximum_wz_radps_, maximum_wz_radps_));
    command.received = std::chrono::steady_clock::now();
    command.available = true;
  }

  void controllerSourceCallback(const std_msgs::msg::String::ConstSharedPtr message)
  {
    if (message->data != "none" && message->data != "pid" &&
      message->data != "efficient_3d_local_planner")
    {
      RCLCPP_ERROR(
        get_logger(), "Rejecting unknown controller source '%s'; output disabled",
        message->data.c_str());
      selected_controller_ = "none";
      clearCommands();
      (void)stopRobot("invalid controller source");
      // StopMove exits the selected gait on the deployed Go2.  A later
      // controller recovery must therefore be allowed to replay the same
      // route/task gait request instead of treating it as a duplicate.
      last_completed_transition_.reset();
      publishStatus();
      return;
    }
    if (message->data == selected_controller_) {
      return;
    }
    const std::string previous = selected_controller_;
    selected_controller_ = message->data;
    clearCommands();
    // While no controller is selected the adapter already holds the robot at
    // zero velocity.  In particular, none -> controller happens immediately
    // after a completed gait transition.  Calling StopMove here cancels the
    // StaticWalk/SwitchGait mode that was just selected.  A real stop remains
    // mandatory when disabling an active controller or switching directly
    // between two active command sources.
    if (selected_controller_ == "none" || previous != "none") {
      (void)stopRobot("controller source switch");
      if (selected_controller_ == "none") {
        // Releasing control for an obstacle stop or operator pause invalidates
        // StaticWalk/SwitchGait.  Permit the PID controller to establish the
        // requested gait again before it resumes autonomous velocity output.
        last_completed_transition_.reset();
      }
    }
    state_detail_ = "controller source switched from '" + previous + "' to '" +
      selected_controller_ + "'";
    publishStatus();
    RCLCPP_INFO(get_logger(), "%s", state_detail_.c_str());
  }

  void transitionCallback(const GaitTransition::ConstSharedPtr message)
  {
    if (!supportedGaitCommand(message->gait_command)) {
      failTransition("unsupported gait command '" + message->gait_command + "'");
      return;
    }
    if (last_completed_transition_.has_value() &&
      sameRequest(*message, *last_completed_transition_))
    {
      return;
    }
    if (pending_transition_.has_value() && sameRequest(*message, *pending_transition_)) {
      return;
    }
    if (state_ == AdapterState::kEmergencyStopped) {
      RCLCPP_ERROR(get_logger(), "Ignoring gait request while emergency stop is latched");
      return;
    }
    if (pending_transition_.has_value() &&
      message->route_sequence == pending_transition_->route_sequence)
    {
      failTransition("conflicting gait request for the same route sequence");
      return;
    }

    pending_transition_ = *message;
    gait_command_retry_count_ = 0;
    last_gait_sdk_code_ = 0;
    selected_controller_ = "none";
    clearCommands();
    transition_started_ = std::chrono::steady_clock::now();
    setState(
      AdapterState::kStopping,
      "accepted gait transition '" + message->gait_command + "'");
  }

  void sportStateCallback(const void * raw_message)
  {
    const auto message =
      *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(raw_message);
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    const auto mode = message.mode();
    const auto gait = message.gait_type();
    if (has_sport_feedback_ && mode == sport_mode_ && gait == gait_type_) {
      ++stable_feedback_samples_;
    } else {
      stable_feedback_samples_ = 1U;
    }
    sport_mode_ = mode;
    gait_type_ = gait;
    has_sport_feedback_ = true;
    ++feedback_sequence_;
    last_feedback_time_ = std::chrono::steady_clock::now();
  }

  void timerCallback()
  {
    switch (state_) {
      case AdapterState::kIdle:
        forwardVelocity();
        break;
      case AdapterState::kStopping:
        beginGaitTransition();
        break;
      case AdapterState::kPreSwitchDelay:
        waitBeforeGaitSwitch();
        break;
      case AdapterState::kRetryWaiting:
        waitForGaitRetry();
        break;
      case AdapterState::kSettling:
        monitorGaitTransition();
        break;
      case AdapterState::kPublishingAcknowledgement:
        publishGaitAcknowledgement();
        break;
      case AdapterState::kError:
      case AdapterState::kEmergencyStopped:
        enforceStopped();
        break;
    }
  }

  void forwardVelocity()
  {
    float vx = 0.0F;
    float vy = 0.0F;
    float wz = 0.0F;
    const CommandState * command = nullptr;
    if (selected_controller_ == "pid") {
      command = &pid_command_;
    } else if (selected_controller_ == "efficient_3d_local_planner") {
      command = &efficient_command_;
    }
    if (command != nullptr && command->available &&
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - command->received).count() <= command_timeout_s_)
    {
      vx = command->vx;
      vy = command->vy;
      wz = command->wz;
    }
    geometry_msgs::msg::Twist selected;
    selected.linear.x = vx;
    selected.linear.y = vy;
    selected.angular.z = wz;
    selected_command_publisher_->publish(selected);
    if (!enable_motion_ || !sport_client_) {
      return;
    }
    const auto result = sport_client_->Move(vx, vy, wz);
    if (result != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "SportClient::Move returned %d", result);
    }
  }

  void beginGaitTransition()
  {
    if (!pending_transition_.has_value()) {
      failTransition("internal error: gait request is missing");
      return;
    }
    if (!stopRobot("before gait switch")) {
      failTransition("StopMove failed before gait switch");
      return;
    }

    transition_started_ = std::chrono::steady_clock::now();
    pre_switch_deadline_ = transition_started_ +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(gait_pre_switch_stop_delay_s_));
    setState(
      AdapterState::kPreSwitchDelay,
      "StopMove accepted; waiting " + std::to_string(gait_pre_switch_stop_delay_s_) +
      " s before gait command");
  }

  void waitBeforeGaitSwitch()
  {
    if (checkTransitionTimeout("timed out while waiting to switch gait after StopMove")) {
      return;
    }
    if (std::chrono::steady_clock::now() < pre_switch_deadline_) {
      return;
    }
    executeGaitCommand();
  }

  void executeGaitCommand()
  {
    if (!pending_transition_.has_value()) {
      failTransition("internal error: gait request is missing after StopMove delay");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback_sequence_at_switch_ = feedback_sequence_;
      stable_feedback_samples_ = 0U;
    }
    const int result = callGaitCommand(pending_transition_->gait_command);
    if (result != 0) {
      handleGaitCommandFailure(result);
      return;
    }
    settle_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(gait_settle_time_s_));
    setState(
      AdapterState::kSettling,
      "gait command accepted; waiting for stable robot state");
  }

  int callGaitCommand(const std::string & command)
  {
    if (!enable_motion_) {
      return 0;
    }
    if (!sport_client_) {
      return -1;
    }
    try {
      if (command == "static_walk") {
        return sport_client_->StaticWalk();
      }
      if (command == "switch_gait_3") {
        return sport_client_->switchGait(3);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Unitree gait SDK exception: %s", error.what());
      return -1;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown Unitree gait SDK exception");
      return -1;
    }
    return -1;
  }

  void handleGaitCommandFailure(const int sdk_code)
  {
    last_gait_sdk_code_ = sdk_code;
    if (!pending_transition_.has_value()) {
      failTransition(
        "gait command failed with SDK code " + std::to_string(sdk_code) +
        " but no pending transition is available");
      return;
    }
    if (gait_command_retry_count_ >= gait_command_max_retries_) {
      failTransition(
        "gait command '" + pending_transition_->gait_command +
        "' failed with SDK code " + std::to_string(sdk_code) + " after " +
        std::to_string(gait_command_retry_count_) + " automatic retries");
      return;
    }

    ++gait_command_retry_count_;
    const double delay_s = gait_command_retry_initial_delay_s_ * std::pow(
      gait_command_retry_backoff_multiplier_,
      static_cast<double>(gait_command_retry_count_ - 1));
    retry_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(delay_s));
    selected_controller_ = "none";
    clearCommands();
    setState(
      AdapterState::kRetryWaiting,
      "gait command '" + pending_transition_->gait_command +
      "' failed with SDK code " + std::to_string(sdk_code) + "; retry " +
      std::to_string(gait_command_retry_count_) + "/" +
      std::to_string(gait_command_max_retries_) + " in " +
      std::to_string(delay_s) + " s");
  }

  void waitForGaitRetry()
  {
    if (std::chrono::steady_clock::now() < retry_deadline_) {
      return;
    }
    if (!pending_transition_.has_value()) {
      failTransition("internal error: gait retry request is missing");
      return;
    }
    setState(
      AdapterState::kStopping,
      "executing automatic gait retry " + std::to_string(gait_command_retry_count_) + "/" +
      std::to_string(gait_command_max_retries_) + " for '" +
      pending_transition_->gait_command + "'");
  }

  void monitorGaitTransition()
  {
    if (checkTransitionTimeout(gaitTransitionTimeoutReason())) {
      return;
    }
    if (std::chrono::steady_clock::now() < settle_deadline_) {
      return;
    }
    if (enable_motion_ && require_state_feedback_ && !sportFeedbackStable()) {
      return;
    }
    setState(
      AdapterState::kPublishingAcknowledgement,
      "gait stable; publishing PID acknowledgement");
  }

  std::string gaitTransitionTimeoutReason()
  {
    if (!pending_transition_.has_value()) {
      return "gait transition timed out without a pending command";
    }
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!has_sport_feedback_) {
      return "gait transition timed out: no sport-mode feedback received";
    }
    return "gait transition timed out: command '" + pending_transition_->gait_command +
           "' did not receive stable fresh sport feedback; observed mode=" +
           std::to_string(static_cast<int>(sport_mode_)) + ", gait_type=" +
           std::to_string(static_cast<int>(gait_type_));
  }

  bool sportFeedbackStable()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!pending_transition_.has_value()) {
      return false;
    }
    // This Go2 firmware accepts the legacy SwitchGait(3) API and visibly
    // enters stair mode, but SportModeState.gait_type remains 0.  Therefore
    // gait_type cannot be used as an acknowledgement for legacy gait calls.
    // Match the proven demo: require a successful SDK return, the settle
    // interval, and fresh stable robot-state feedback before acknowledging.
    if (!has_sport_feedback_ || feedback_sequence_ <= feedback_sequence_at_switch_ ||
      stable_feedback_samples_ < minimum_stable_samples_)
    {
      return false;
    }
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_feedback_time_).count() <= state_timeout_s_;
  }

  void publishGaitAcknowledgement()
  {
    if (!pending_transition_.has_value()) {
      failTransition("internal error: gait acknowledgement is missing");
      return;
    }
    const auto expected = *pending_transition_;
    gait_acknowledged_publisher_->publish(expected);
    last_completed_transition_ = expected;
    pending_transition_.reset();
    gait_command_retry_count_ = 0;
    last_gait_sdk_code_ = 0;
    setState(AdapterState::kIdle, "gait transition complete; PID acknowledgement published");
  }

  bool checkTransitionTimeout(const std::string & reason)
  {
    if (std::chrono::duration<double>(
      std::chrono::steady_clock::now() - transition_started_).count() <= transition_timeout_s_)
    {
      return false;
    }
    failTransition(reason);
    return true;
  }

  bool stopRobot(const std::string & reason)
  {
    if (!enable_motion_ || !sport_client_) {
      return true;
    }
    try {
      const auto result = sport_client_->StopMove();
      if (result == 0) {
        return true;
      }
      RCLCPP_WARN(
        get_logger(), "StopMove failed during '%s' with SDK code %d; trying Move(0,0,0)",
        reason.c_str(), result);
      const auto fallback_result = sport_client_->Move(0.0F, 0.0F, 0.0F);
      if (fallback_result != 0) {
        RCLCPP_ERROR(
          get_logger(), "Move(0,0,0) fallback failed during '%s' with SDK code %d",
          reason.c_str(), fallback_result);
      }
      return fallback_result == 0;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "StopMove exception during '%s': %s", reason.c_str(), error.what());
      return false;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown StopMove exception during '%s'", reason.c_str());
      return false;
    }
  }


  void enforceStopped()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    if (now_steady - last_redundant_stop_time_ < std::chrono::seconds(1)) {
      return;
    }
    last_redundant_stop_time_ = now_steady;
    (void)stopRobot("latched stop state");
  }

  void failTransition(const std::string & detail)
  {
    selected_controller_ = "none";
    clearCommands();
    (void)stopRobot("transition failure");
    setState(AdapterState::kError, detail);
  }

  void setState(const AdapterState state, const std::string & detail)
  {
    state_ = state;
    state_detail_ = detail;
    publishStatus();
    if (state == AdapterState::kError || state == AdapterState::kEmergencyStopped) {
      RCLCPP_ERROR(get_logger(), "Go2 adapter state=%s: %s", stateName(state), detail.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Go2 adapter state=%s: %s", stateName(state), detail.c_str());
    }
  }

  void publishStatus()
  {
    nlohmann::json status = {
      {"state", stateName(state_)},
      {"detail", state_detail_},
      {"enable_motion", enable_motion_},
      {"active_controller", selected_controller_},
      {"has_pid_command", pid_command_.available},
      {"has_efficient_command", efficient_command_.available},
      {"has_sport_state", has_sport_feedback_},
      {"gait_retry_count", gait_command_retry_count_},
      {"gait_retry_max", gait_command_max_retries_},
      {"last_gait_sdk_code", last_gait_sdk_code_}};
    if (pending_transition_.has_value()) {
      status["route_sequence"] = pending_transition_->route_sequence;
      status["task_index"] = pending_transition_->task_index;
      status["gait_command"] = pending_transition_->gait_command;
    }
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (has_sport_feedback_) {
        status["sport_mode"] = sport_mode_;
        status["gait_type"] = gait_type_;
        status["stable_feedback_samples"] = stable_feedback_samples_;
      }
    }
    std_msgs::msg::String message;
    message.data = status.dump();
    status_publisher_->publish(message);
  }

  void emergencyStopCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    pending_transition_.reset();
    gait_command_retry_count_ = 0;
    last_gait_sdk_code_ = 0;
    selected_controller_ = "none";
    clearCommands();
    const bool stopped = stopRobot("emergency stop service");
    setState(AdapterState::kEmergencyStopped, "emergency stop latched; restart node to clear");
    response->success = stopped;
    response->message = state_detail_;
  }

  void retryCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (state_ != AdapterState::kError || !pending_transition_.has_value()) {
      response->success = false;
      response->message = "no failed gait transition is available for retry";
      return;
    }
    gait_command_retry_count_ = 0;
    last_gait_sdk_code_ = 0;
    transition_started_ = std::chrono::steady_clock::now();
    setState(AdapterState::kStopping, "operator requested gait transition retry");
    response->success = true;
    response->message = state_detail_;
  }

  std::string network_interface_;
  bool enable_motion_{false};
  void clearCommands()
  {
    pid_command_ = {};
    efficient_command_ = {};
  }

  std::string pid_command_topic_;
  std::string efficient_command_topic_;
  std::string controller_source_topic_;
  std::string selected_command_topic_;
  std::string transition_topic_;
  std::string gait_acknowledged_topic_;
  std::string status_topic_;
  double send_rate_hz_{30.0};
  double command_timeout_s_{0.10};
  double maximum_vx_mps_{0.80};
  double maximum_vy_mps_{0.40};
  double maximum_wz_radps_{1.20};
  double gait_pre_switch_stop_delay_s_{1.0};
  double gait_settle_time_s_{2.0};
  double transition_timeout_s_{8.0};
  std::int64_t gait_command_max_retries_{3};
  double gait_command_retry_initial_delay_s_{1.0};
  double gait_command_retry_backoff_multiplier_{2.0};
  bool require_state_feedback_{true};
  std::string sport_state_topic_;
  double state_timeout_s_{1.0};
  std::size_t minimum_stable_samples_{5U};
  double sdk_timeout_s_{2.0};

  AdapterState state_{AdapterState::kIdle};
  std::string state_detail_;
  std::optional<GaitTransition> pending_transition_;
  std::optional<GaitTransition> last_completed_transition_;
  std::string selected_controller_{"none"};
  CommandState pid_command_;
  CommandState efficient_command_;
  std::chrono::steady_clock::time_point transition_started_{};
  std::chrono::steady_clock::time_point pre_switch_deadline_{};
  std::chrono::steady_clock::time_point settle_deadline_{};
  std::chrono::steady_clock::time_point retry_deadline_{};
  std::chrono::steady_clock::time_point last_redundant_stop_time_{};
  std::int64_t gait_command_retry_count_{0};
  int last_gait_sdk_code_{0};

  std::mutex feedback_mutex_;
  bool has_sport_feedback_{false};
  std::uint8_t sport_mode_{0U};
  std::uint8_t gait_type_{0U};
  std::size_t stable_feedback_samples_{0U};
  std::uint64_t feedback_sequence_{0U};
  std::uint64_t feedback_sequence_at_switch_{0U};
  std::chrono::steady_clock::time_point last_feedback_time_{};

  std::unique_ptr<Go2SportClientCompat> sport_client_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
    sport_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pid_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr efficient_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_source_subscription_;
  rclcpp::Subscription<GaitTransition>::SharedPtr transition_subscription_;
  rclcpp::Publisher<GaitTransition>::SharedPtr gait_acknowledged_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr selected_command_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retry_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace route3d_go2_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<route3d_go2_adapter::Go2RouteAdapterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route3d_go2_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
