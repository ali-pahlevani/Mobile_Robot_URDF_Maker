from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from mobRobURDF_launch.ros_compat import uses_stamped_twist
import os


def _find_gz_ros2_control_lib_dir():
    """Returns lib dir of workspace-built gz_ros2_control, or None if not installed."""
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix('gz_ros2_control')
        lib_dir = os.path.join(prefix, 'lib')
        if os.path.exists(os.path.join(lib_dir, 'libgz_ros2_control-system.so')):
            return lib_dir
    except Exception:
        pass
    return None


def read_selected_controller(description_share):
    """Read selected_controller.txt written by the wizard; falls back to diffDrive_controller."""
    controller_file = os.path.join(description_share, 'urdf', 'selected_controller.txt')
    try:
        with open(controller_file, 'r') as f:
            name = f.read().strip()
            if name:
                return name
    except OSError:
        pass
    return 'diffDrive_controller'


def _bridge_params_path(gazebo_config):
    """Use wizard-generated bridge YAML if present, otherwise fall back to the static one."""
    generated = os.path.join(gazebo_config, 'gz_bridge_generated.yaml')
    if os.path.exists(generated):
        return generated
    return os.path.join(gazebo_config, 'gz_bridge.yaml')


def _camera_image_topics(gazebo_config):
    """Read camera image topics from gz_image_topics.txt written by the wizard."""
    topics_file = os.path.join(gazebo_config, 'gz_image_topics.txt')
    try:
        with open(topics_file, 'r') as f:
            return [t.strip() for t in f.read().splitlines() if t.strip()]
    except OSError:
        return ['/camera/image_raw']


def generate_launch_description():

    description_share = get_package_share_directory('mobRobURDF_description')
    gazebo_config = os.path.join(get_package_share_directory('mobRobURDF_gazebo'), 'config')

    urdf_path = os.path.join(description_share, 'urdf', 'mobRob.urdf.xacro')
    processed_urdf = xacro.process_file(urdf_path).toxml()

    controller_name = read_selected_controller(description_share)

    # On Humble/Iron the apt gz_ros2_control targets Fortress, so the
    # workspace-built Harmonic plugin must shadow it. On Jazzy+ the apt plugin
    # is already Harmonic and on the default path, so the shim is skipped.
    plugin_lib_dir = None if uses_stamped_twist() else _find_gz_ros2_control_lib_dir()
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
        'world', default_value=default_world, description='World to load')

    gz_version = LaunchConfiguration('gz_version')
    gz_version_arg = DeclareLaunchArgument(
        'gz_version', default_value='8',
        description='Gazebo Sim major version (8 = Harmonic, 6 = Fortress)')

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
        parameters=[{'robot_description': processed_urdf, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mobRobURDF', '-z', '0.5'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller_name],
        parameters=[{'use_sim_time': True}]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': True}]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={_bridge_params_path(gazebo_config)}'],
        parameters=[{'use_sim_time': True}]
    )

    image_topics = _camera_image_topics(gazebo_config)
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=image_topics,
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

    cmd_vel_relay = Node(
        package='mobRobURDF_launch',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'controller_name': controller_name, 'use_sim_time': True}]
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
