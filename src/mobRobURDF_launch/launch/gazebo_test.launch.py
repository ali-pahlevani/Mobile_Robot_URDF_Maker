from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    urdf_path = get_package_share_directory('mobRobURDF_description') + '/urdf/mobRob.urdf.xacro'
    
    processed_urdf = xacro.process_file(urdf_path).toxml()

    rviz_config_path = PathJoinSubstitution([ 
        get_package_share_directory('mobRobURDF_launch'),
        'rviz',
        'rviz_gazebo_test.rviz'
    ])

    default_world = os.path.join(
        get_package_share_directory('mobRobURDF_gazebo'),
        'worlds',
        'warehouse_world.sdf'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': processed_urdf,
            'use_sim_time': True
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mobRobURDF', '-z', '0.5'],
        output='screen'
    )

    controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffDrive_controller"],
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    bridge_params = os.path.join(get_package_share_directory('mobRobURDF_gazebo'),'config','gz_bridge.yaml')

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        world_arg,
        gazebo,
        spawn_entity,
        controllers,
        joint_state_broadcaster,
        ros_gz_bridge,
        ros_gz_image_bridge,
        rviz
    ])