**Teleoperate Twist Keyboard (with Remapping)**
- ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_key

**Run Localization (using AMCL in nav2)**
- ros2 launch mobRobURDF_launch localization_launch.py map:=mapFile.yaml (save, not serialize) use_sim_time:=true

**Run Navigation (Nav2)**
- ros2 launch mobRobURDF_launch navigation_launch.py use_sim_time:=true
=> When after localization, we want to start Nav2, we should also give it an extra flag: "map_subscribe_transient_local:=true"

**Twist Mux**
- ros2 run twist_mux twist_mux --ros-args --params-file:=file.yaml -r cmd_vel_out:=input_cmd_vel

**Run Localization (using slam_toolbox and a pre-built map)**
- ros2 launch mobRobURDF_launch online_async_localizer_launch.py slam_params_file:=src/mobRobURDF_navigation/config/localization_config/localizer_params_online_async.yaml use_sim_time:=true

----------------------------------------------------------------------------

**Run SLAM (using slam_toolbox)**
- ros2 launch mobRobURDF_launch online_async_mapper_launch.py slam_params_file:=src/mobRobURDF_navigation/config/slam_config/mapper_params_online_async.yaml use_sim_time:=true

**How to save maps?**
- Using slam_toolbox services (save_map & serialize_map)
- Using Rviz plugin: "SlamToolboxPlugin"

**Loading a saved map**
- ros2 run nav2_map_server map_server --ros-args -p yaml_file_name:="file address".yaml -p use_sim_time:=true
- Activating the map_server: ros2 run nav2_util lifecycle_bringup map_server

**Adaptive Monte Carlo Localization (AMCL):**
- ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
- Activating the amcl: ros2 run nav2_util lifecycle_bringup amcl
- Set the "initial pose" (either by clicking or publishing to "/initialpose" topic)
=> For forklift, set the "use_sim_time" to "false"

