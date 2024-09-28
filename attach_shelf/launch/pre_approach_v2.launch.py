from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.3"
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90"
    )
    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="false"
    )

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f = LaunchConfiguration('final_approach')

    attach_shelf_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2',
        output='screen',
        name='attach_shelf_node',
        emulate_tty=True,
        parameters=[  # Parameters should be passed here
            {"obstacle": obstacle_f},
            {"degrees": degrees_f},
            {"final_approach": final_approach_f}
        ]

    )

    # create and return launch description object
    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            attach_shelf_node
        ]
    )