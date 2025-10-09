import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory for the example_sweeping_generator package
    pkg_share = get_package_share_directory('sweeping_generator')

    # Define the path to the ROS parameters file
    params_file = os.path.join(pkg_share, 'config', 'sweeping_generator.yaml')

    # 1. Declare the launch argument 'UAV_NAME'
    # This is the equivalent of <arg name="UAV_NAME" ... />
    # It checks for an environment variable 'UAV_NAME' and defaults to 'uav1' if not set.
    uav_name_arg = DeclareLaunchArgument(
        'UAV_NAME',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='The namespace of the drone.'
    )

    # Create a LaunchConfiguration to use the value of the argument
    uav_name = LaunchConfiguration('UAV_NAME')

    # 2. Define the Node to launch
    # This replaces the <node ...> tag and its contents.
    sweeping_generator_node = Node(
        package='weeping_generator',
        executable='sweeping_generator.py',
        name='sweeping_generator',
        output='screen',
        # The 'ns' attribute in the <group> tag is replaced by the 'namespace' parameter
        namespace=uav_name,
        # The <rosparam> tag is replaced by the 'parameters' parameter
        parameters=[params_file],
        # The <remap> tags are replaced by the 'remappings' parameter
        # Note: The '~' is implicit for the 'from' side in ROS 2 remappings.
        remappings=[
            ('control_manager_diag_in', 'control_manager/diagnostics'),
            ('path_out', 'trajectory_generation/path'),
            ('start_in', 'start'),
        ]
    )

    # 3. Create the LaunchDescription and add the actions
    return LaunchDescription([
        uav_name_arg,
        sweeping_generator_node,
    ])
