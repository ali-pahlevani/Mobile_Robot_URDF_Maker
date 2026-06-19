import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_xacro_path = os.path.join(
        get_package_share_directory('mobRobURDF_description'),
        'urdf',
        'mobRob.urdf.xacro'
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory('mobRobURDF_launch'),
        'rviz',
        'rviz_test.rviz'
    )

    processed_urdf = xacro.process_file(urdf_xacro_path).toxml()

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='True',
        description='Use joint state publisher GUI'
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': processed_urdf,
            'use_sim_time': True
        }]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        use_gui_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz2_node,
    ])
