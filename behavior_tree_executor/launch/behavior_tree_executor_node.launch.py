
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('behavior_tree_executor'),
        'config',
        'behavior_tree_executor.yaml'
    )

    node = Node(
        package='behavior_tree_executor',
        namespace='',
        executable='behavior_tree_executor_node',
        name='behavior_tree_executor_node',
        output="screen",
        #remappings=[("/aito_example_node/subscribe", "/aito_example_node/publish")],
        parameters=[
            ParameterFile(config, allow_substs=True)
        ]
    )
    ld.add_action(node)

    return ld
