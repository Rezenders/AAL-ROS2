// Copyright 2026 KAS-lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "aal_bt/ensure_configuration.hpp"

#include <memory>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace aal_bt
{

EnsureConfiguration::EnsureConfiguration(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::DecoratorNode(name, config)
{
  node_ = rclcpp::Node::make_shared("ensure_cfg_" + name);
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

BT::PortsList EnsureConfiguration::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      ACTION_NAME_PORT,
      "/reconfigure_system",
      "ReconfigureSystem action server name"),
    BT::InputPort<std::vector<std::string>>(
      ACTIVATE_PORT,
      "Nodes that must be active before the child runs"),
    BT::InputPort<std::vector<std::string>>(
      DEACTIVATE_PORT,
      std::vector<std::string>{},
      "Nodes that must be inactive before the child runs"),
    BT::InputPort<std::vector<std::string>>(
      ON_EXIT_PORT,
      std::vector<std::string>{},
      "Teardown override. Empty=deactivate nodes_to_activate. [\"none\"]=skip."),
  };
}

void EnsureConfiguration::initActionClient(const std::string & action_name)
{
  action_client_ = rclcpp_action::create_client<ReconfigureAction>(
    node_, action_name, callback_group_);
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "EnsureConfiguration: action server '%s' not available after 5s",
      action_name.c_str());
  }
}

void EnsureConfiguration::resetGoalState()
{
  goal_accepted_ = false;
  result_received_ = false;
  result_success_ = false;
  goal_handle_.reset();
}

void EnsureConfiguration::cancelCurrentGoal()
{
  if (goal_handle_ && action_client_) {
    const auto status = goal_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    {
      action_client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
  }
}

void EnsureConfiguration::sendGoal(
  const std::vector<std::string> & to_activate,
  const std::vector<std::string> & to_deactivate)
{
  resetGoalState();

  ReconfigureAction::Goal goal_msg;
  goal_msg.desired_features.active_features = to_activate;
  goal_msg.desired_features.inactive_features = to_deactivate;

  auto options = rclcpp_action::Client<ReconfigureAction>::SendGoalOptions{};

  options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & handle) {
      if (!handle) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "EnsureConfiguration: goal rejected by action server");
        result_received_ = true;
        result_success_ = false;
      } else {
        goal_handle_ = handle;
        goal_accepted_ = true;
      }
    };

  options.result_callback =
    [this](const GoalHandle::WrappedResult & wr) {
      result_received_ = true;
      result_success_ = (wr.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wr.result->success);
      if (!result_success_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "EnsureConfiguration: reconfiguration failed (code=%d)",
          static_cast<int>(wr.code));
      }
    };

  action_client_->async_send_goal(goal_msg, options);
}

std::vector<std::string> EnsureConfiguration::computeTeardownList()
{
  std::vector<std::string> on_exit;
  getInput(ON_EXIT_PORT, on_exit);

  if (!on_exit.empty()) {
    if (on_exit.size() == 1 && on_exit[0] == TEARDOWN_NONE) {
      return {};
    }
    return on_exit;
  }
  return activated_nodes_;
}

BT::NodeStatus EnsureConfiguration::tick()
{
  // ── Idle: first tick ────────────────────────────────────────────────────
  if (status() == BT::NodeStatus::IDLE) {
    std::string action_name;
    getInput(ACTION_NAME_PORT, action_name);

    std::vector<std::string> to_activate;
    getInput(ACTIVATE_PORT, to_activate);

    std::vector<std::string> to_deactivate;
    getInput(DEACTIVATE_PORT, to_deactivate);

    activated_nodes_ = to_activate;

    initActionClient(action_name);
    sendGoal(to_activate, to_deactivate);
    phase_ = Phase::SettingUp;
    setStatus(BT::NodeStatus::RUNNING);
    return BT::NodeStatus::RUNNING;
  }

  // ── SettingUp: waiting for reconfigure action ────────────────────────────
  if (phase_ == Phase::SettingUp) {
    executor_.spin_some();
    if (!result_received_) {
      return BT::NodeStatus::RUNNING;
    }
    if (!result_success_) {
      phase_ = Phase::Idle;
      return BT::NodeStatus::FAILURE;
    }
    // fall through to RunningChild
    phase_ = Phase::RunningChild;
  }

  // ── RunningChild ─────────────────────────────────────────────────────────
  if (phase_ == Phase::RunningChild) {
    BT::NodeStatus child_status = child_node_->executeTick();

    if (child_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }

    // Child finished (SUCCESS or FAILURE)
    child_result_ = child_status;
    resetChild();

    auto teardown = computeTeardownList();
    if (teardown.empty()) {
      phase_ = Phase::Idle;
      return child_result_;
    }

    sendGoal({}, teardown);
    phase_ = Phase::TearingDown;
    return BT::NodeStatus::RUNNING;
  }

  // ── TearingDown ──────────────────────────────────────────────────────────
  if (phase_ == Phase::TearingDown) {
    executor_.spin_some();
    if (!result_received_) {
      return BT::NodeStatus::RUNNING;
    }
    phase_ = Phase::Idle;
    return child_result_;
  }

  // Should never reach here
  return BT::NodeStatus::FAILURE;
}

void EnsureConfiguration::halt()
{
  cancelCurrentGoal();

  auto teardown = computeTeardownList();
  if (!teardown.empty() && action_client_) {
    ReconfigureAction::Goal goal_msg;
    goal_msg.desired_features.inactive_features = teardown;
    // fire-and-forget
    action_client_->async_send_goal(goal_msg);
  }

  resetChild();
  phase_ = Phase::Idle;
  activated_nodes_.clear();
}

}  // namespace aal_bt
