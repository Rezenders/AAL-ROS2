//  Copyright 2026 KAS-lab
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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "aal_bt/ensure_configuration.hpp"
#include "aal_msgs/action/set_desired_features.hpp"

// ─── Aliases ─────────────────────────────────────────────────────────────────

using DSC = aal_msgs::action::SetDesiredFeatures;
using GoalHandleDSC = rclcpp_action::ServerGoalHandle<DSC>;

// ─── Recorded goal data ───────────────────────────────────────────────────────

struct ReceivedGoal
{
  std::vector<std::string> active_features;
  std::vector<std::string> inactive_features;
};

// ─── Mock action server ───────────────────────────────────────────────────────

class MockReconfigureServer
{
public:
  MockReconfigureServer(
    rclcpp::Node::SharedPtr node,
    bool accept = true,
    bool succeed = true)
  : accept_(accept), succeed_(succeed)
  {
    server_ = rclcpp_action::create_server<DSC>(
      node,
      "/reconfigure_system",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const DSC::Goal>) {
        return accept_ ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
        rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<GoalHandleDSC>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandleDSC> handle) {execute(handle);});
  }

  int goals_received() const {return goals_received_.load();}

  std::vector<ReceivedGoal> received_goals() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_goals_;
  }

private:
  void execute(std::shared_ptr<GoalHandleDSC> handle)
  {
    const auto & cfg = handle->get_goal()->desired_features;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      received_goals_.push_back({cfg.active_features, cfg.inactive_features});
    }  // releases mutex_
    goals_received_++;
    auto result = std::make_shared<DSC::Result>();
    result->success = succeed_;
    result->message = succeed_ ? "ok" : "failed";
    handle->succeed(result);
  }

  bool accept_;
  bool succeed_;
  std::atomic<int> goals_received_{0};
  mutable std::mutex mutex_;
  std::vector<ReceivedGoal> received_goals_;
  rclcpp_action::Server<DSC>::SharedPtr server_;
};

// ─── Test fixture ─────────────────────────────────────────────────────────────

class EnsureConfigurationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_node_ = rclcpp::Node::make_shared("mock_server_node");
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
    factory_->registerNodeType<aal_bt::EnsureConfiguration>("EnsureConfiguration");
  }

  void TearDown() override
  {
    if (server_executor_) {
      server_executor_->cancel();
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    mock_server_.reset();
    server_executor_.reset();
  }

  void startServer(bool accept = true, bool succeed = true)
  {
    mock_server_ = std::make_unique<MockReconfigureServer>(server_node_, accept, succeed);
    server_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    server_executor_->add_node(server_node_);
    server_thread_ = std::thread([this]() {server_executor_->spin();});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::string makeXml(
    const std::string & child_type,
    const std::string & nodes_to_activate,
    const std::string & nodes_to_deactivate = "",
    const std::string & on_exit = "")
  {
    std::string xml =
      R"(<root BTCPP_format="4"><BehaviorTree ID="T">)"
      R"(<EnsureConfiguration action_name="/reconfigure_system")"
      R"( nodes_to_activate=")" + nodes_to_activate + R"(")";
    if (!nodes_to_deactivate.empty()) {
      xml += R"( nodes_to_deactivate=")" + nodes_to_deactivate + R"(")";
    }
    if (!on_exit.empty()) {
      xml += R"( nodes_to_deactivate_on_exit=")" + on_exit + R"(")";
    }
    xml += "><" + child_type + "/>";
    xml += "</EnsureConfiguration></BehaviorTree></root>";
    return xml;
  }

  BT::NodeStatus tickUntilDone(BT::Tree & tree, int max_ticks = 300)
  {
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < max_ticks && status == BT::NodeStatus::RUNNING; ++i) {
      status = tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return status;
  }

  rclcpp::Node::SharedPtr server_node_;
  std::unique_ptr<MockReconfigureServer> mock_server_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> server_executor_;
  std::thread server_thread_;
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
};

// ─── Tests ────────────────────────────────────────────────────────────────────

TEST_F(EnsureConfigurationTest, SetupSuccessChildSuccessDefaultTeardown)
{
  startServer(true, true);
  auto tree = factory_->createTreeFromText(
    makeXml("AlwaysSuccess", "identify_node", "explore_node"));

  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mock_server_->goals_received(), 2);

  auto goals = mock_server_->received_goals();
  ASSERT_EQ(goals.size(), 2u);
  // Teardown must deactivate what was activated (default behaviour)
  EXPECT_EQ(goals[1].inactive_features, std::vector<std::string>{"identify_node"});
  EXPECT_TRUE(goals[1].active_features.empty());
}

TEST_F(EnsureConfigurationTest, SetupSuccessChildFailureTeardownFires)
{
  startServer(true, true);
  auto tree = factory_->createTreeFromText(makeXml("AlwaysFailure", "identify_node"));

  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mock_server_->goals_received(), 2);
}

TEST_F(EnsureConfigurationTest, SetupFailureReturnsFailureChildNeverRuns)
{
  startServer(true, false);   // accepts goal but returns success=false
  auto tree = factory_->createTreeFromText(makeXml("AlwaysSuccess", "identify_node"));

  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
  // Only 1 goal (setup); no teardown when setup fails
  EXPECT_EQ(mock_server_->goals_received(), 1);
}

TEST_F(EnsureConfigurationTest, GoalRejectedReturnsFailure)
{
  startServer(false, true);   // rejects every goal
  auto tree = factory_->createTreeFromText(makeXml("AlwaysSuccess", "identify_node"));

  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
}

TEST_F(EnsureConfigurationTest, TeardownSkippedWithNoneKeyword)
{
  startServer(true, true);
  auto tree = factory_->createTreeFromText(
    makeXml("AlwaysSuccess", "identify_node", "", "none"));

  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
  // Only setup goal — no teardown
  EXPECT_EQ(mock_server_->goals_received(), 1);
}

TEST_F(EnsureConfigurationTest, CustomTeardownListUsed)
{
  startServer(true, true);
  auto tree = factory_->createTreeFromText(
    makeXml("AlwaysSuccess", "identify_node", "", "custom_node_a"));

  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mock_server_->goals_received(), 2);

  auto goals = mock_server_->received_goals();
  ASSERT_EQ(goals.size(), 2u);
  EXPECT_EQ(goals[1].inactive_features, std::vector<std::string>{"custom_node_a"});
}

TEST_F(EnsureConfigurationTest, SetupGoalCarriesCorrectNodes)
{
  startServer(true, true);
  auto tree = factory_->createTreeFromText(
    makeXml("AlwaysSuccess", "node_a;node_b", "node_c"));

  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);

  auto goals = mock_server_->received_goals();
  ASSERT_EQ(goals.size(), 2u);
  EXPECT_EQ(goals[0].active_features, (std::vector<std::string>{"node_a", "node_b"}));
  EXPECT_EQ(goals[0].inactive_features, std::vector<std::string>{"node_c"});
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
