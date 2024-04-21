#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/action_node.h"
namespace bt_moveit2_nodes {
using namespace BT;
class ExecutePlan : public StatefulActionNode {
 public:
  ExecutePlan(const std::string& name, const NodeConfig& config,
              const std::shared_ptr<rclcpp::Node> nh)
      : StatefulActionNode(name, config), node_(nh) {
    Expected<std::string> planing_group =
        getInput<std::string>("planing_group");
    if (!planing_group) {
      throw BT::RuntimeError("missing required input [planing_group]: ",
                             planing_group.error());
    }
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface_ =
        std::make_shared<MoveGroupInterface>(node_, planing_group.value());
  }

  static PortsList providedPorts() {
    return {InputPort<std::string>("planing_group"),
            InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory")};
  }

  NodeStatus onStart() override {
    isRunning_ = true;
    executeSuccess_ = false;

    execute_thread_ = std::make_shared<std::thread>(
        std::bind(&ExecutePlan::execute, this, std::ref(isRunning_),
                  std::ref(executeSuccess_)));
    return NodeStatus::RUNNING;
  }
  NodeStatus onRunning() override {
    if (isRunning_) {
      return NodeStatus::RUNNING;
    }
    execute_thread_->join();
    if (executeSuccess_) {
      return NodeStatus::SUCCESS;
    } else {
      return NodeStatus::FAILURE;
    }
  }

  void onHalted() override {
    isRunning_ = false;
    move_group_interface_->stop();
    execute_thread_->detach();
  }

 private:
  void execute(std::atomic<bool>& isRunning,
               std::atomic<bool>& executeSuccess) {
    if (auto any_ptr = getLockedPortContent("trajectory")) {
      if (!any_ptr->empty()) {
        auto* plan_ptr = any_ptr->castPtr<moveit_msgs::msg::RobotTrajectory>();
        if (move_group_interface_->execute(*plan_ptr)) {
          executeSuccess = true;
        }
      }
    }
    isRunning = false;
  }
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_interface_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<std::thread> execute_thread_;
  std::atomic<bool> isRunning_;
  std::atomic<bool> executeSuccess_;
};

}  // namespace bt_moveit2_nodes
