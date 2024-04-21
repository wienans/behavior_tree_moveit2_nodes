#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/action_node.h"
namespace bt_moveit2_nodes {
using namespace BT;
class PlanToPoseCartesian : public StatefulActionNode {
 public:
  PlanToPoseCartesian(const std::string& name, const NodeConfig& config,
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
        std::bind(&PlanToPoseCartesian::plan, this, std::ref(isRunning_),
                  std::ref(planSuccess_), std::ref(finalTrajectory_)));
    return NodeStatus::RUNNING;
  }
  NodeStatus onRunning() override {
    if (isRunning_) {
      return NodeStatus::RUNNING;
    }
    plan_thread_->join();
    if (planSuccess_) {
      setOutput("trajectory", finalTrajectory_);
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
            moveit_msgs::msg::RobotTrajectory& finalTrajectory) {
    if (auto any_ptr = getLockedPortContent("pose_stamped")) {
      if (!any_ptr->empty()) {
        auto* pose_ptr = any_ptr->castPtr<geometry_msgs::msg::PoseStamped>();
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(pose_ptr->pose);
        auto const [success, trajectory] = [this, waypoints] {
          moveit_msgs::msg::RobotTrajectory trajectory;
          const double jump_threshold = 0.0;
          const double eef_step = 0.01;
          double fraction = move_group_interface_->computeCartesianPath(
              waypoints, eef_step, jump_threshold, trajectory);
          if (fraction < 0.99) {
            return std::make_pair(false, trajectory);
          } else {
            return std::make_pair(true, trajectory);
          }
        }();
        planSuccess = success;
        finalTrajectory = trajectory;
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
  moveit_msgs::msg::RobotTrajectory finalTrajectory_;
};

}  // namespace bt_moveit2_nodes
