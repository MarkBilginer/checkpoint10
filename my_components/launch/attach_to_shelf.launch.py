import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_description = "my_components"

    # Declare RViz configuration file argument
    rviz_config_file_name_arg = DeclareLaunchArgument(
        "rviz_config_file_name", 
        default_value='attach_shelf.rviz',
        description="Full path to RViz config file"
    )
    
    rviz_config_file_name = LaunchConfiguration("rviz_config_file_name")

    # Create the composable node container
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package=package_description,
                plugin='my_components::PreApproach',
                name='pre_approach_component'
            ),
            ComposableNode(
                package=package_description,
                plugin='my_components::AttachServer',
                name='attach_server_component'
            ),
        ],
        output='screen',
    )

    # Include the RViz launch file
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_description),
                'launch',
                'start_rviz_with_arguments.launch.py'
            ])
        ]),
        launch_arguments={'rviz_config_file_name': rviz_config_file_name}.items()
    )

    return LaunchDescription([
        rviz_config_file_name_arg,
        container,
        rviz_launch
    ])