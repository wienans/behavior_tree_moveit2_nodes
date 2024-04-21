#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/action_node.h"
namespace bt_moveit2_nodes {
using namespace BT;
class PlanToPose : public StatefulActionNode {
 public:
  PlanToPose(const std::string& name, const NodeConfig& config,
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
    return {InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped"),
            InputPort<std::string>("planing_group"),
            OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory")};
  }

  NodeStatus onStart() override {
    isRunning_ = true;
    planSuccess_ = false;

    plan_thread_ = std::make_shared<std::thread>(
        std::bind(&PlanToPose::plan, this, std::ref(isRunning_),
                  std::ref(planSuccess_), std::ref(finalPlan_)));
    return NodeStatus::RUNNING;
  }
  NodeStatus onRunning() override {
    if (isRunning_) {
      return NodeStatus::RUNNING;
    }
    plan_thread_->join();
    if (planSuccess_) {
      setOutput("trajectory", finalPlan_.trajectory_);
      return NodeStatus::SUCCESS;
    } else {
      return NodeStatus::FAILURE;
    }
  }

  void onHalted() override {
    isRunning_ = false;
    plan_thread_->detach();
  }

 private:
  void plan(std::atomic<bool>& isRunning, std::atomic<bool>& planSuccess,
            moveit::planning_interface::MoveGroupInterface::Plan& finalPlan) {
    if (auto any_ptr = getLockedPortContent("pose_stamped")) {
      if (!any_ptr->empty()) {
        auto* pose_ptr = any_ptr->castPtr<geometry_msgs::msg::PoseStamped>();
        move_group_interface_->setPoseTarget(*pose_ptr);
        auto const [success, plan] = [this] {
          moveit::planning_interface::MoveGroupInterface::Plan msg;
          auto const ok = static_cast<bool>(move_group_interface_->plan(msg));
          return std::make_pair(ok, msg);
        }();
        planSuccess = success;
        finalPlan = plan;
      }
    }
    isRunning = false;
  }
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_interface_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<std::thread> plan_thread_;
  std::atomic<bool> isRunning_;
  std::atomic<bool> planSuccess_;
  moveit::planning_interface::MoveGroupInterface::Plan finalPlan_;
};

}  // namespace bt_moveit2_nodes
