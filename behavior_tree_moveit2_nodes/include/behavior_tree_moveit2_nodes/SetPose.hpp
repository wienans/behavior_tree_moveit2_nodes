#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace bt_moveit2_nodes {
using namespace BT;
class SetPose : public SyncActionNode {
 public:
  SetPose(const std::string& name, const NodeConfig& config)
      : SyncActionNode(name, config) {}

  static PortsList providedPorts() {
    return {InputPort<double>("x"),
            InputPort<double>("y"),
            InputPort<double>("z"),
            InputPort<double>("qx"),
            InputPort<double>("qy"),
            InputPort<double>("qz"),
            InputPort<double>("qw"),
            InputPort<std::string>("frame_id"),
            OutputPort<geometry_msgs::msg::PoseStamped>("pose_stamped")};
  }

  NodeStatus tick() override {
    Expected<double> x = getInput<double>("x");
    Expected<double> y = getInput<double>("y");
    Expected<double> z = getInput<double>("z");
    Expected<double> qx = getInput<double>("qx");
    Expected<double> qy = getInput<double>("qy");
    Expected<double> qz = getInput<double>("qz");
    Expected<double> qw = getInput<double>("qw");
    Expected<std::string> frame_id = getInput<std::string>("frame_id");
    // Check if expected is valid. If not, throw its error
    if (!x | !y | !z | !frame_id) {
      return NodeStatus::FAILURE;
    }
    double q[4] = {0, 0, 0, 1.0};
    if (qx && qy && qz && qw) {
      q[0] = qx.value();
      q[1] = qy.value();
      q[2] = qz.value();
      q[3] = qw.value();
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = frame_id.value();
    msg.pose.orientation.w = 1.0;
    msg.pose.position.x = x.value();
    msg.pose.position.y = y.value();
    msg.pose.position.z = z.value();
    msg.pose.orientation.x = q[0];
    msg.pose.orientation.y = q[1];
    msg.pose.orientation.z = q[2];
    msg.pose.orientation.w = q[3];

    if (auto any_ptr = getLockedPortContent("pose_stamped")) {
      any_ptr.assign(msg);
    }
    // setOutput("pose_stamped", msg);

    return NodeStatus::SUCCESS;
  }
};

}  // namespace bt_moveit2_nodes
