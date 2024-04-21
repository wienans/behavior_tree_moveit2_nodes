# BehaviorTree.CPP Moveit2 Nodes

This package includes some [BehaviorTree.CPP v4](https://github.com/BehaviorTree/BehaviorTree.CPP) Nodes for [MoveIt2](https://github.com/ros-planning/moveit2).

This is clearly inspired by [MoveItPro](https://picknik.ai/pro/) which is a closed source behavior development platform for robotic arms. But with the Tools of [BehaviorTree.CPP v4](https://github.com/BehaviorTree/BehaviorTree.CPP) and [Groot2](https://www.behaviortree.dev/groot) we only need the power of additional Nodes to create a more open and acessible move to Behavior Trees also in the world of robotic arms.

## behavior_tree_moveit2_nodes

This package includes the Node implementation as header only files.

To add a new Node create a new header file add it to the `bt_moveit2_nodes.hpp` and compile with colcon.

Currently Available Nodes:

- PlanToPose
  - Input: PoseStamped, Planing Group
  - Output: RobotTrajectory
- PlanToPoseCartesian
  - Input: PoseStamped, Planing Group
  - Output: RobotTrajectory
- PrintPose (Debugging)
  - Input: PoseStamped
- SetPose
  - Input: x,y,z,qx,qy,qz,qw,frame_id
  - Output: PoseStamped
- ExecutePlan:
  - Input: RobotTrajectory, Planing Group
- MoveToPose
  - Input: PoseStamped, Planing Group

## behavior_tree_executor

This package includes a example behaviortree executor loading the given nodes into a factory.

## Contribution

Feel free use and contribute new Nodes to the Repository or improve the Nodes.
I hope we can start something open source to advance the use of Behavior Trees in the work with Robotic Arms.
