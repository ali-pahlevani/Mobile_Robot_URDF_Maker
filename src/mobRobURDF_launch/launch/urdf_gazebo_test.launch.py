from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def read_selected_controller(description_share):
    """Read the controller spawner name chosen in the wizard, with a fallback."""
    controller_file = os.path.join(description_share, 'urdf', 'selected_controller.txt')
    try:
        with open(controller_file, 'r') as f:
            name = f.read().strip()
            if name:
                return name
    except OSError:
        pass
    return 'diffDrive_controller'


def generate_launch_description():

    description_share = get_package_share_directory('mobRobURDF_description')

    # Define URDF path
    urdf_path = os.path.join(
        description_share,
        'urdf',
        'mobRob.urdf'
    )

    # Verify URDF file exists
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

    # Read the URDF file directly
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    controller_name = read_selected_controller(description_share)

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
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true',
            'extra_gz_args': '--ros-args --params-file ' + os.path.join(
                get_package_share_directory('mobRobURDF_gazebo'), 'config', 'use_sim_time.yaml'
            )
        }.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mobRobURDF', '-z', '0.5'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name],
        parameters=[{'use_sim_time': True}]
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}]
    )

    bridge_params = os.path.join(get_package_share_directory('mobRobURDF_gazebo'),'config','gz_bridge.yaml')

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[{'use_sim_time': True}]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        parameters=[{'use_sim_time': True}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
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