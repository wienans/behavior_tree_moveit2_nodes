#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
namespace bt_moveit2_nodes
{
using namespace BT;
class MoveToPose : public StatefulActionNode
{
public:
  MoveToPose(
    const std::string & name, const NodeConfig & config,
    const std::shared_ptr<rclcpp::Node> nh)
  :StatefulActionNode(name, config), node_(nh)
  {
    Expected<std::string> planing_group = getInput<std::string>("planing_group");
    if (!planing_group) {
      throw
        BT::RuntimeError("missing required input [planing_group]: ", planing_group.error());
    }
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface_ = std::make_shared<MoveGroupInterface>(node_, planing_group.value());
  }

  static PortsList providedPorts()
  {
    return {InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped"),
      InputPort<std::string>("planing_group")};
  }

  NodeStatus onStart() override
  {
    isRunning_ = true;
    threadSuccess_ = false;

    plan_thread_ = std::make_shared<std::thread>(
      std::bind(
        &MoveToPose::plan, this, std::ref(isRunning_), std::ref(threadSuccess_))
    );
    return NodeStatus::RUNNING;
  }
  NodeStatus onRunning() override
  {
    if (isRunning_) {
      return NodeStatus::RUNNING;
    }
    plan_thread_->join();
    if (threadSuccess_) {
      return NodeStatus::SUCCESS;
    } else {
      return NodeStatus::FAILURE;
    }
  }

  void onHalted() override
  {
    isRunning_ = false;
    move_group_interface_->stop();
    plan_thread_->detach();
  }

private:
  void plan(
    std::atomic<bool> & isRunning, std::atomic<bool> & successThread)
  {
    // Get Input Pose to Move to
    Expected<geometry_msgs::msg::PoseStamped> pose =
      getInput<geometry_msgs::msg::PoseStamped>("pose_stamped");
    // Check if expected is valid. If not, Fail Node
    if (pose) {
      // Set Pose as Target
      move_group_interface_->setPoseTarget(pose.value());
      // Plan to Pose
      auto const [successPlan, plan] = [this] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface_->plan(msg));
        return std::make_pair(ok, msg);
      }();
      // Check if Plan is valid
      if (successPlan) {
        // Move to Pose
        auto const successExecute = [this, plan] {
          return static_cast<bool>(move_group_interface_->execute(plan));
        }();
        // Check if Move was successful
        if (successExecute) {
          successThread = true;
        } else {
          successThread = false;
        }
      } else {
        successThread = false;
      }
    } else {
      successThread = false;
    }
    isRunning = false;
  }
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<std::thread> plan_thread_;
  std::atomic<bool> isRunning_;
  std::atomic<bool> threadSuccess_;
};

} // namespace name
