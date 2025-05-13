import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # File paths for URDF and RViz configuration
    urdf_file = os.path.join(get_package_share_directory('mobRobURDF_description'), 'urdf', 'mobRob.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('mobRobURDF_launch'), 'rviz', 'rviz_test.rviz')

    # Declare the launch argument for GUI visibility
    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='True', description='Use joint state publisher GUI')

    # Joint State Publisher (GUI) node
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_gui'))
    )

    # Robot State Publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # RViz2 node with configuration file
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Return the launch description with all the nodes and the launch argument
    return LaunchDescription([
        use_gui_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz2_node,
    ])
