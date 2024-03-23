cmake_minimum_required(VERSION 3.8)
project(behavior_tree_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)

find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behavior_tree_moveit2_nodes REQUIRED)

find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_components
    rcpputils
    behaviortree_cpp
    behaviortree_ros2
    behavior_tree_moveit2_nodes
    std_msgs
    Eigen3 )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
set(node_plugins "")

# ##########
# BUILD  #
# ##########
add_library(behavior_tree_executor SHARED
  src/behavior_tree_executor.cpp)
target_compile_definitions(behavior_tree_executor
  PRIVATE "BEHAVIOR_TREE_EXECUTOR_DLL")
ament_target_dependencies(behavior_tree_executor ${THIS_PACKAGE_DEPS})

rclcpp_components_register_nodes(behavior_tree_executor "behavior_tree_executor::BehaviorTreeExecutor")
set(node_plugins "${node_plugins}behavior_tree_executor::BehaviorTreeExecutor;$<TARGET_FILE:behavior_tree_executor>\n")


# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(behavior_tree_executor_node src/behavior_tree_executor_node.cpp)
target_link_libraries(behavior_tree_executor_node behavior_tree_executor)
target_compile_features(behavior_tree_executor_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(behavior_tree_executor_node rclcpp)

# ############
# INSTALL  #
# ############
install(TARGETS
  behavior_tree_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  behavior_tree_executor_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tree
  DESTINATION share/${PROJECT_NAME}
)

# ############
# TESTING  #
# ############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(behaviortree_ros2)

ament_package()
