#include "behavior_tree_executor/behavior_tree_executor.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  // Create Nodes
  std::shared_ptr<behavior_tree_executor::BehaviorTreeExecutor> node =
      std::make_shared<behavior_tree_executor::BehaviorTreeExecutor>(options);
  // Create Executor and Spin
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
