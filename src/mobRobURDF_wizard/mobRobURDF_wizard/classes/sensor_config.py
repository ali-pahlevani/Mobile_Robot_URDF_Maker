"""Data model for a single configurable sensor (lidar or camera).

Each SensorConfig instance represents one physical sensor mounted on the robot.
The module also provides helpers to generate URDF XML fragments and Gazebo bridge
YAML content from a list of SensorConfig objects.
"""

from dataclasses import dataclass

_COLOR_RGBA = {
    'Gray':  '0.5  0.5  0.5  1',
    'Black': '0.05 0.05 0.05 1',
    'Red':   '1.0  0.0  0.0  1',
    'Blue':  '0.0  0.2  1.0  1',
    'Green': '0.0  0.8  0.0  1',
    'White': '1.0  1.0  1.0  1',
}


@dataclass
class SensorConfig:
    sensor_type: str = 'lidar'   # 'lidar' or 'camera'
    name: str = 'lidar_1'

    # Mount pose (relative to chassis link)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.3
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Visual / physics
    color: str = 'Black'
    mass: float = 0.1

    # Lidar geometry + sensor params
    radius: float = 0.1
    length: float = 0.08
    h_samples: int = 360
    h_min_angle: float = -3.14159
    h_max_angle: float = 3.14159
    min_range: float = 0.3
    max_range: float = 12.0
    update_rate: float = 10.0

    # Camera geometry + sensor params
    cam_depth: float = 0.08
    cam_width: float = 0.08
    cam_height: float = 0.06
    h_fov: float = 1.089
    v_fov: float = 0.785
    img_width: int = 640
    img_height: int = 480
    near_clip: float = 0.05
    far_clip: float = 8.0
    cam_update_rate: float = 10.0


# ── XML / YAML helpers ────────────────────────────────────────────────────────

def _rgba(color: str) -> str:
    return _COLOR_RGBA.get(color, '1.0 1.0 1.0 1')


def _ixx_cylinder(m: float, r: float, l: float):
    ixy = (1 / 12) * m * (3 * r * r + l * l)
    izz = 0.5 * m * r * r
    return ixy, izz


def _ixx_box(m: float, d: float, w: float, h: float):
    ix = (1 / 12) * m * (w * w + h * h)
    iy = (1 / 12) * m * (d * d + h * h)
    iz = (1 / 12) * m * (d * d + w * w)
    return ix, iy, iz


def lidar_urdf_xml(s: SensorConfig) -> str:
    ixy, izz = _ixx_cylinder(s.mass, s.radius, s.length)
    rgba = _rgba(s.color)
    return f"""\
  <!-- Lidar: {s.name} -->
  <link name="{s.name}">
    <visual>
      <geometry><cylinder length="{s.length}" radius="{s.radius}"/></geometry>
      <material name="{s.color}"/>
    </visual>
    <collision>
      <geometry><cylinder length="{s.length}" radius="{s.radius}"/></geometry>
    </collision>
    <inertial>
      <mass value="{s.mass}"/>
      <inertia ixx="{ixy:.6f}" ixy="0" ixz="0" iyy="{ixy:.6f}" iyz="0" izz="{izz:.6f}"/>
    </inertial>
  </link>
  <joint name="{s.name}_joint" type="fixed">
    <parent link="chassis"/>
    <child link="{s.name}"/>
    <origin xyz="{s.x} {s.y} {s.z}" rpy="{s.roll} {s.pitch} {s.yaw}"/>
  </joint>
  <gazebo reference="{s.name}">
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <emissive>0 0 0 1</emissive>
    </material>
    <sensor name="{s.name}_sensor" type="gpu_lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>{s.update_rate}</update_rate>
      <lidar>
        <scan>
          <horizontal>
            <samples>{s.h_samples}</samples>
            <min_angle>{s.h_min_angle}</min_angle>
            <max_angle>{s.h_max_angle}</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>{s.min_range}</min>
          <max>{s.max_range}</max>
        </range>
      </lidar>
      <topic>{s.name}/scan</topic>
      <gz_frame_id>{s.name}</gz_frame_id>
    </sensor>
  </gazebo>
"""


def camera_urdf_xml(s: SensorConfig) -> str:
    ix, iy, iz = _ixx_box(s.mass, s.cam_depth, s.cam_width, s.cam_height)
    rgba = _rgba(s.color)
    optical = f"{s.name}_optical"
    return f"""\
  <!-- Camera: {s.name} -->
  <link name="{s.name}">
    <visual>
      <geometry><box size="{s.cam_depth} {s.cam_width} {s.cam_height}"/></geometry>
      <material name="{s.color}"/>
    </visual>
    <collision>
      <geometry><box size="{s.cam_depth} {s.cam_width} {s.cam_height}"/></geometry>
    </collision>
    <inertial>
      <mass value="{s.mass}"/>
      <inertia ixx="{ix:.6f}" ixy="0" ixz="0" iyy="{iy:.6f}" iyz="0" izz="{iz:.6f}"/>
    </inertial>
  </link>
  <joint name="{s.name}_joint" type="fixed">
    <parent link="chassis"/>
    <child link="{s.name}"/>
    <origin xyz="{s.x} {s.y} {s.z}" rpy="{s.roll} {s.pitch} {s.yaw}"/>
  </joint>
  <link name="{optical}"/>
  <joint name="{optical}_joint" type="fixed">
    <parent link="{s.name}"/>
    <child link="{optical}"/>
    <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
  </joint>
  <gazebo reference="{s.name}">
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <emissive>0 0 0 1</emissive>
    </material>
    <sensor name="{s.name}_sensor" type="camera">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>{s.cam_update_rate}</update_rate>
      <camera>
        <camera_info_topic>{s.name}/camera_info</camera_info_topic>
        <horizontal_fov>{s.h_fov}</horizontal_fov>
        <image>
          <format>R8G8B8</format>
          <width>{s.img_width}</width>
          <height>{s.img_height}</height>
        </image>
        <clip>
          <near>{s.near_clip}</near>
          <far>{s.far_clip}</far>
        </clip>
      </camera>
      <topic>{s.name}/image_raw</topic>
      <gz_frame_id>{optical}</gz_frame_id>
    </sensor>
  </gazebo>
"""


def sensor_urdf_xml(s: SensorConfig) -> str:
    if s.sensor_type == 'lidar':
        return lidar_urdf_xml(s)
    return camera_urdf_xml(s)


def build_user_sensors_xacro(sensors: list) -> str:
    """Return the full content of user_sensors.xacro for the given sensor list."""
    body = '\n'.join(sensor_urdf_xml(s) for s in sensors)
    return (
        '<?xml version="1.0"?>\n'
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        + body
        + '\n</robot>\n'
    )


def build_bridge_yaml(sensors: list) -> str:
    """Build gz_bridge YAML content for clock + all sensors."""
    lines = [
        '- ros_topic_name: "clock"',
        '  gz_topic_name: "clock"',
        '  ros_type_name: "rosgraph_msgs/msg/Clock"',
        '  gz_type_name: "gz.msgs.Clock"',
        '  direction: GZ_TO_ROS',
    ]
    for s in sensors:
        if s.sensor_type == 'lidar':
            lines += [
                f'- ros_topic_name: "{s.name}/scan"',
                f'  gz_topic_name: "{s.name}/scan"',
                '  ros_type_name: "sensor_msgs/msg/LaserScan"',
                '  gz_type_name: "gz.msgs.LaserScan"',
                '  direction: GZ_TO_ROS',
            ]
        elif s.sensor_type == 'camera':
            lines += [
                f'- ros_topic_name: "{s.name}/camera_info"',
                f'  gz_topic_name: "{s.name}/camera_info"',
                '  ros_type_name: "sensor_msgs/msg/CameraInfo"',
                '  gz_type_name: "gz.msgs.CameraInfo"',
                '  direction: GZ_TO_ROS',
            ]
    return '\n'.join(lines) + '\n'


def camera_image_topics(sensors: list) -> list:
    """Return a list of ROS image topic names for all camera sensors."""
    return [f'/{s.name}/image_raw' for s in sensors if s.sensor_type == 'camera']


def default_sensors(robot_type: str, chassis_size_str: str = '1.2 0.8 0.3') -> list:
    """Create one default lidar + one default camera sized to the chassis."""
    try:
        parts = chassis_size_str.split()
        L, W, H = float(parts[0]), float(parts[1]), float(parts[2])
    except Exception:
        L, W, H = 1.2, 0.8, 0.3

    lidar = SensorConfig(
        sensor_type='lidar', name='lidar_1',
        x=0.0, y=0.0, z=round(H / 2 + 0.04, 4),
        color='Black', mass=0.1,
        radius=0.1, length=0.08,
    )
    camera = SensorConfig(
        sensor_type='camera', name='camera_1',
        x=round(L / 2 + 0.04, 4), y=0.0, z=0.0,
        color='Blue', mass=0.1,
    )
    return [lidar, camera]


def sensor_to_dict(s: SensorConfig) -> dict:
    """Serialize a SensorConfig to a plain dict for preset storage."""
    return {
        'sensor_type': s.sensor_type,
        'name': s.name,
        'x': s.x, 'y': s.y, 'z': s.z,
        'roll': s.roll, 'pitch': s.pitch, 'yaw': s.yaw,
        'color': s.color, 'mass': s.mass,
        'radius': s.radius, 'length': s.length,
        'h_samples': s.h_samples,
        'h_min_angle': s.h_min_angle, 'h_max_angle': s.h_max_angle,
        'min_range': s.min_range, 'max_range': s.max_range,
        'update_rate': s.update_rate,
        'cam_depth': s.cam_depth, 'cam_width': s.cam_width, 'cam_height': s.cam_height,
        'h_fov': s.h_fov,
        'v_fov': s.v_fov,
        'img_width': s.img_width, 'img_height': s.img_height,
        'near_clip': s.near_clip, 'far_clip': s.far_clip,
        'cam_update_rate': s.cam_update_rate,
    }


def sensor_from_dict(d: dict) -> SensorConfig:
    """Deserialize a SensorConfig from a preset dict."""
    s = SensorConfig()
    for field in s.__dataclass_fields__:
        if field in d:
            setattr(s, field, d[field])
    return s
