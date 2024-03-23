#!/usr/bin/env python3

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('behavior_tree_executor'),
        'config',
        'behavior_tree_executor.yaml'
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='example_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='behavior_tree_executor',
                    plugin='behavior_tree_executor::BehaviorTreeExecutor',
                    name='behavior_tree_executor_node',
                    #remappings=[("/aito_example_node/subscribe", "/aito_example_node/publish")],
                    parameters=[ParameterFile(config, allow_substs=True)],
                    extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        output='both',
    )

    return launch.LaunchDescription([container])
