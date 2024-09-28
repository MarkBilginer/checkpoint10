from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    package_description = "attach_shelf"

    # Declare the launch arguments
    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.3", description="Distance to obstacle"
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90", description="Degrees to rotate"
    )
    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="false", description="Perform final approach"
    )

    rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file_name", default_value=PathJoinSubstitution(
            [FindPackageShare(package_description), 'rviz_config', 'attach_shelf.rviz']),
            description="Full path to RViz config file"
    )

    # Get the launch configurations
    obstacle_f = LaunchConfiguration("obstacle")
    degrees_f = LaunchConfiguration("degrees")
    final_approach_f = LaunchConfiguration("final_approach")
    rviz_config_file_name_f = LaunchConfiguration("rviz_config_file_name")

    # Node for final approach service
    approach_service_node = Node(
        package="attach_shelf",
        executable="approach_service_server",  # Make sure the executable name is correct
        name="approach_service_server",
        output="screen"
    )

    # Include the pre-approach launch file
    attach_shelf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_description),
                'launch',
                'pre_approach_v2.launch.py'
            ])
        ]),
        launch_arguments={
            'obstacle': obstacle_f,
            'degrees': degrees_f,
            "final_approach": final_approach_f
        }.items()
    )

    # Include the RViz launch file
    start_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_description),
                'launch',
                'start_rviz_with_arguments.launch.py'
            ])
        ]),
        launch_arguments={'rviz_config_file_name': rviz_config_file_name_f}.items()
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        rviz_config_file_arg,
        approach_service_node,
        TimerAction(
            period=5.0,  # Wait 5 seconds before starting the client node
            actions=[attach_shelf_launch]
        ),
        start_rviz_launch
    ])
