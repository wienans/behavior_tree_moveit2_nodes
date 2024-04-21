#include "behavior_tree_executor/behavior_tree_executor.hpp"

#include "behavior_tree_moveit2_nodes/bt_moveit2_nodes.hpp"
namespace behavior_tree_executor {

BehaviorTreeExecutor::BehaviorTreeExecutor(rclcpp::NodeOptions options)
    : rclcpp::Node("behavior_tree_executor", options) {
  this->declare_parameter<std::string>("tree_folder_path");

  cbg_one_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Register Interfaces
  registerParameters();
  // Reuse the bt tick timer to get out of the Constructor for initialization
  // the this->shared_from_this() only works out of constructor and is needed
  // for Node init
  timer_bt_tick_ =
      create_wall_timer(std::chrono::milliseconds(1),
                        std::bind(&BehaviorTreeExecutor::init, this));
}

BehaviorTreeExecutor::~BehaviorTreeExecutor() {}
void BehaviorTreeExecutor::init() {
  // Cancle the timer directly (one time use)
  timer_bt_tick_->cancel();
  // Get the Shared Pointer from the node
  auto node = this->shared_from_this();

  // Create Factory
  factory_ = std::make_shared<BT::BehaviorTreeFactory>();

  // Register Nodes
  factory_->registerNodeType<bt_moveit2_nodes::SetPose>("SetPose");
  factory_->registerNodeType<bt_moveit2_nodes::PrintPose>("PrintPose");
  factory_->registerNodeType<bt_moveit2_nodes::PlanToPose>("PlanToPose", node);
  factory_->registerNodeType<bt_moveit2_nodes::PlanToPoseCartesian>(
      "PlanToPoseCartesian", node);
  factory_->registerNodeType<bt_moveit2_nodes::ExecutePlan>("ExecutePlan",
                                                            node);
  factory_->registerNodeType<bt_moveit2_nodes::MoveToPose>("MoveToPose", node);

  // Register Behavior Trees
  for (auto const& entry :
       std::filesystem::directory_iterator(tree_folder_path_)) {
    std::cout << entry.path() << std::endl;
    if (entry.path().extension() == ".xml") {
      factory_->registerBehaviorTreeFromFile(entry.path().string());
    }
  }

  main_tree_ = std::make_shared<BT::Tree>(factory_->createTree("main"));

  // Groot 2
  groot_logger_ = std::make_shared<BT::Groot2Publisher>(*main_tree_);

  // Lightweight serialization
  // file_logger_ = std::make_shared<BT::FileLogger2>(*main_tree_,
  // "test.btlog");
  cout_logger_ = std::make_shared<BT::StdCoutLogger>(*main_tree_);

  timer_bt_tick_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&BehaviorTreeExecutor::behaviortreeTick, this));
}
void BehaviorTreeExecutor::behaviortreeTick() { main_tree_->tickOnce(); }

void BehaviorTreeExecutor::registerParameters() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Reading Parameters");

  // For Handling Parameter changes
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, this->get_name(), rmw_qos_profile_parameters, cbg_one_);
  param_sub_ = param_client_->on_parameter_event(std::bind(
      &BehaviorTreeExecutor::onParamEvent, this, std::placeholders::_1));

  // Read Parameter
  this->get_parameter("tree_folder_path", tree_folder_path_);

  RCLCPP_INFO_STREAM(this->get_logger(), tree_folder_path_);
}

void BehaviorTreeExecutor::onParamEvent(
    rcl_interfaces::msg::ParameterEvent::UniquePtr event) {
  // ignore qos overrides
  event->new_parameters.erase(
      std::remove_if(event->new_parameters.begin(), event->new_parameters.end(),
                     [](const auto& item) {
                       const char* param_override_prefix = "qos_overrides.";
                       return std::strncmp(
                                  item.name.c_str(), param_override_prefix,
                                  sizeof(param_override_prefix) - 1) == 0u;
                     }),
      event->new_parameters.end());
  // return if no new parameters or changes happened
  if (!event->new_parameters.size() && !event->changed_parameters.size() &&
      !event->deleted_parameters.size()) {
    return;
  }
  std::string namespaced_node_name;
  if (this->get_namespace() == std::string("/")) {
    namespaced_node_name =
        std::string(this->get_namespace()) + this->get_name();
  } else {
    namespaced_node_name =
        this->get_namespace() + std::string("/") + this->get_name();
  }
  // Change Parameters which are dedicated to this Node
  if (event->node == namespaced_node_name) {
    for (auto& changed_parameter : event->changed_parameters) {
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Changed Parameter : " << changed_parameter.name << "of Value Type: "
                                 << std::to_string(changed_parameter.value.type)
                                 << ", to:  "
                                 << changed_parameter.value.string_value);
      if (changed_parameter.name == "tree_folder_path") {
        tree_folder_path_ = changed_parameter.value.string_value;
      }
    }
  }
}

}  // namespace behavior_tree_executor
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(behavior_tree_executor::BehaviorTreeExecutor)
