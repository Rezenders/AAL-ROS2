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
#ifndef AAL_BT__ENSURE_CONFIGURATION_HPP_
#define AAL_BT__ENSURE_CONFIGURATION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "aal_msgs/action/desired_system_configuration.hpp"

namespace aal_bt
{

/**
 * Decorator that ensures a desired lifecycle configuration is active before
 * running its child, then tears it down on completion or preemption.
 *
 * Ports:
 *   action_name                 (string)         — ReconfigureSystem action server name
 *   nodes_to_activate           (vector<string>) — nodes that must be active before child
 *   nodes_to_deactivate         (vector<string>) — nodes that must be inactive before child
 *   nodes_to_deactivate_on_exit (vector<string>)
 *       empty    → deactivate whatever was in nodes_to_activate (default)
 *       ["none"] → skip teardown entirely
 *       other    → deactivate exactly these nodes
 */
class EnsureConfiguration : public BT::DecoratorNode
{
public:
  using ReconfigureAction = aal_msgs::action::DesiredSystemConfiguration;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ReconfigureAction>;

  static constexpr const char * ACTION_NAME_PORT = "action_name";
  static constexpr const char * ACTIVATE_PORT = "nodes_to_activate";
  static constexpr const char * DEACTIVATE_PORT = "nodes_to_deactivate";
  static constexpr const char * ON_EXIT_PORT = "nodes_to_deactivate_on_exit";
  static constexpr const char * TEARDOWN_NONE = "none";

  EnsureConfiguration(const std::string & name, const BT::NodeConfig & config);
  ~EnsureConfiguration() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  enum class Phase { Idle, SettingUp, RunningChild, TearingDown };

  void initActionClient(const std::string & action_name);
  void sendGoal(
    const std::vector<std::string> & to_activate,
    const std::vector<std::string> & to_deactivate);
  std::vector<std::string> computeTeardownList();
  void cancelCurrentGoal();
  void resetGoalState();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp_action::Client<ReconfigureAction>::SharedPtr action_client_;

  Phase phase_{Phase::Idle};
  BT::NodeStatus child_result_{BT::NodeStatus::IDLE};
  bool goal_accepted_{false};
  bool result_received_{false};
  bool result_success_{false};
  GoalHandle::SharedPtr goal_handle_;

  std::vector<std::string> activated_nodes_;
};

}  // namespace aal_bt

#endif  // AAL_BT__ENSURE_CONFIGURATION_HPP_
