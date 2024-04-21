#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace bt_moveit2_nodes {
using namespace BT;
class PrintPose : public SyncActionNode {
 public:
  PrintPose(const std::string& name, const NodeConfig& config)
      : SyncActionNode(name, config) {}

  static PortsList providedPorts() {
    return {InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped")};
  }

  NodeStatus tick() override {
    if (auto any_ptr = getLockedPortContent("pose_stamped")) {
      if (!any_ptr->empty()) {
        auto* pose_ptr = any_ptr->castPtr<geometry_msgs::msg::PoseStamped>();

        std::cout << pose_ptr->pose.position.x << " "
                  << pose_ptr->pose.position.y << " "
                  << pose_ptr->pose.position.z << std::endl;
      }
    }

    return NodeStatus::SUCCESS;
  }
};

}  // namespace bt_moveit2_nodes
