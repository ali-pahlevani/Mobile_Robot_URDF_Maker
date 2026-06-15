from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
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


def _find_gz_ros2_control_lib_dir():
    """Return the lib dir of the workspace-built gz_ros2_control if available."""
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix('gz_ros2_control')
        lib_dir = os.path.join(prefix, 'lib')
        if os.path.exists(os.path.join(lib_dir, 'libgz_ros2_control-system.so')):
            return lib_dir
    except Exception:
        pass
    return None


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

    # Ensure the workspace-built gz_ros2_control (Harmonic) shadows the apt Fortress version.
    plugin_lib_dir = _find_gz_ros2_control_lib_dir()
    gz_plugin_path_actions = []
    if plugin_lib_dir:
        existing = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        new_path = plugin_lib_dir + (':' + existing if existing else '')
        gz_plugin_path_actions = [
            SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', new_path)
        ]

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

    # Gazebo Sim major version: 8 = Harmonic (default), 6 = Fortress.
    gz_version = LaunchConfiguration('gz_version')
    gz_version_arg = DeclareLaunchArgument(
        'gz_version',
        default_value='8',
        description='Gazebo Sim major version (8 = Harmonic, 6 = Fortress)'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_version': gz_version,
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

    # Relay /cmd_vel → the active controller's velocity topic so all controllers
    # share a single /cmd_vel interface for teleoperation.
    cmd_vel_relay = Node(
        package='mobRobURDF_launch',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{
            'controller_name': controller_name,
            'use_sim_time': True,
        }]
    )

    return LaunchDescription(
        gz_plugin_path_actions + [
            node_robot_state_publisher,
            world_arg,
            gz_version_arg,
            gazebo,
            spawn_entity,
            controllers,
            joint_state_broadcaster,
            ros_gz_bridge,
            ros_gz_image_bridge,
            rviz,
            cmd_vel_relay,
        ]
    )
