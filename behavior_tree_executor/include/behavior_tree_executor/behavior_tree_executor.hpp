#pragma once
// C Header
#include <eigen3/Eigen/Dense>

// Standard CPP
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "rcpputils/visibility_control.hpp"

// Behavior Tree
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

// Messages
#include "builtin_interfaces/msg/time.hpp"

namespace behavior_tree_executor {

class BehaviorTreeExecutor : public rclcpp::Node {
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

  /**
   * @brief
   *
   */
  void init();

  /**
   * @brief Timer Callback to Tick behavior tree
   */
  void behaviortreeTick();

  std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  std::shared_ptr<BT::Tree> main_tree_;
  std::shared_ptr<BT::Groot2Publisher> groot_logger_;
  std::shared_ptr<BT::FileLogger2> file_logger_;
  std::shared_ptr<BT::StdCoutLogger> cout_logger_;
  /**
   * @brief Thimer to syncronisly tick the behavior tree
   */
  rclcpp::TimerBase::SharedPtr timer_bt_tick_;
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
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
      param_sub_;
  /**
   * @brief Asynchron Parameter Client
   */
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
};
}  // namespace behavior_tree_executor
