#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace bt_moveit2_nodes
{
using namespace BT;
class PlanToPose : public ThreadedAction
{
public:
  PlanToPose(const std::string & name, const NodeConfig & config)
  :ThreadedAction(name, config)
  {
  }

  static PortsList providedPorts()
  {
    return {InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped")};
  }


  NodeStatus tick() override
  {
    std::atomic<bool> isRunning(true);

    std::cout << "start" << std::endl;
    plan_thread_ = std::make_shared<std::thread>(
      std::bind(&PlanToPose::plan, this, std::ref(isRunning))
    );

    std::cout << "tick" << std::endl;
    while (!isHaltRequested() && isRunning) {
      std::cout << isRunning << " " << isHaltRequested() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Check if we stopped because of halt
    if (isHaltRequested()) {
      // plan_thread_->~thread();
      return NodeStatus::IDLE;
    }
    plan_thread_->join();
    return NodeStatus::SUCCESS;
  }

  void halt() override
  {
    ThreadedAction::halt();
  }

private:
  void plan(std::atomic<bool> & isRunning)
  {
    std::cout << "start plan" << std::endl;
    if (auto any_ptr = getLockedPortContent("pose_stamped")) {
      std::cout << "lock" << std::endl;
      if (!any_ptr->empty()) {
        auto * pose_ptr = any_ptr->castPtr<geometry_msgs::msg::PoseStamped>();

        std::cout << "Test" << pose_ptr->pose.position.x << " " << pose_ptr->pose.position.y <<
          " " <<
          pose_ptr->pose.position.z << std::endl;
      } else {
        std::cout << "empty" << std::endl;
      }
    } else {
      std::cout << "not locked" << std::endl;
    }
    for (int i = 0; i < 5; i++) {
      std::cout << i << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    isRunning = false;
  }


  std::shared_ptr<std::thread> plan_thread_;
};

} // namespace name
