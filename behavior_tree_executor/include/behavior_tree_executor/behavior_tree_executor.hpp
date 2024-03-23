#pragma once
// C Header
#include <eigen3/Eigen/Dense>

// Standard CPP
#include <memory>
#include <functional>
#include <vector>
#include <filesystem>
#include <string>
#include <thread>
#include <chrono>

// ROS
#include "rcpputils/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"

//Behavior Tree
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

// Messages
#include "builtin_interfaces/msg/time.hpp"

namespace behavior_tree_executor
{

class BehaviorTreeExecutor : public rclcpp::Node
{
public:
  RCPPUTILS_PUBLIC BehaviorTreeExecutor(rclcpp::NodeOptions options);
  ~BehaviorTreeExecutor();

private:
  /**
   * @brief Handles Parameter Server and changes
   */
  void registerParameters();
  /**
   * @brief Callback to handle the Parameter Change event
   * @param event incomming message
   */
  void onParamEvent(rcl_interfaces::msg::ParameterEvent::UniquePtr event);

  std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  std::shared_ptr<BT::Tree> main_tree_;
  std::shared_ptr<BT::Groot2Publisher> groot_logger_;
  std::shared_ptr<BT::FileLogger2> file_logger_;
  std::shared_ptr<BT::StdCoutLogger> cout_logger_;
  /**
   * @brief Thread to tick the behavior tree
   */
  std::shared_ptr<std::thread> bt_tick_thread_;
  /**
   * @brief The example parameter
   */
  std::string tree_folder_path_;
  /**
   * @brief Callback Group One
   */
  rclcpp::CallbackGroup::SharedPtr cbg_one_;
  /**
   * @brief Subscriber for Parameter Events / Parameter changes
   */
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;
  /**
   * @brief Asynchron Parameter Client
   */
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
};
}
