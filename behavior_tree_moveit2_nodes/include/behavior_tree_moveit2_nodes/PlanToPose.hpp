#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace bt_moveit2_nodes
{
using namespace BT;
class PlanToPose : public SyncActionNode
{
public:
  PlanToPose(const std::string & name, const NodeConfig & config)
  :SyncActionNode(name, config)
  {
    plan_thread_ = std::make_shared<std::thread>(&PlanToPose::plan, this);
  }

  static PortsList providedPorts()
  {
    return {InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped")};
  }


  NodeStatus tick() override
  {
    if (plan_thread_->joinable()) {
      return NodeStatus::RUNNING;
    } else {
      return NodeStatus::SUCCESS;
    }
  }

private:
  void plan()
  {
    if (auto any_ptr = getLockedPortContent("pose_stamped")) {
      if (!any_ptr->empty()) {
        auto * pose_ptr = any_ptr->castPtr<geometry_msgs::msg::PoseStamped>();

        std::cout << pose_ptr->pose.position.x << " " << pose_ptr->pose.position.y << " " <<
          pose_ptr->pose.position.z << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }


  std::shared_ptr<std::thread> plan_thread_;
};

} // namespace name
