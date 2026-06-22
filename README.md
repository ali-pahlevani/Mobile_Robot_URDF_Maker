# Mobile Robot URDF Maker (V4)

![Mobile Robot URDF Maker Banner](https://github.com/user-attachments/assets/161b6c72-ac37-4727-98b4-ce089e223c78)

**Mobile Robot URDF Maker** is a *ROS 2* desktop application that **automates** the creation of a complete, simulation-ready **URDF** for your **mobile robot** — through a guided *PyQt5* wizard, with no hand-written *XML* required. V4 adds **six selectable `ros2_control` controllers**, a **dynamic multi-sensor system**, a **Controller Tuner**, **hardware-interface selection**, and **built-in teleoperation** — so you can pick a chassis, tune the kinematics, add sensors, and **drive the robot live in *Gazebo*, all from one window.**

The app walks you through building a mobile-robot description step by step: pick a chassis type, choose a controller, set dimensions, add as many lidars and cameras as you like, tune the controller, then simulate and drive. The result is a clean `.urdf` **and** a re-editable `.urdf.xacro`, plus matching controller configuration, ready to drop into your own stack.

- **Supported platforms:** *ROS 2* **Humble** (*Ubuntu 22.04*) and *ROS 2* **Jazzy** (*Ubuntu 24.04*) — the **same codebase** auto-adapts to whichever you build/source.
- **Simulator:** *Gazebo Harmonic* (`gz sim` 8) via `gz_ros2_control`.
- **GUI:** *PyQt5* with a live *OpenGL* 3D preview.

> **One codebase, both distros.** The app detects `$ROS_DISTRO` at runtime and adjusts the parts that differ between distributions — chiefly the controller command interface (*Humble*/*Iron* use unstamped `Twist`; *Jazzy*+ use `TwistStamped`). The detected environment is shown on the **Start Project** page. You don't pick a distro in the app; it follows the environment you sourced.

## What's New in V4

* **Six Selectable Controllers**: Differential, skid-steer differential, mecanum, tricycle, tricycle-steering and Ackermann — each backed by a real `ros2_control` plugin, and only the controllers valid for the chosen chassis are offered.
* **Dynamic Multi-Sensor System**: Add any number of lidars and cameras, each fully configurable, with a live 3D preview and an auto-generated *Gazebo* bridge.
* **Controller Tuner**: Edit publish/update rates, command timeout, velocity/acceleration and steering limits — or fill them with physics-based **Auto-suggest** values.
* **Hardware-Interface Selection**: *Gazebo* simulation, mock hardware, or your own custom `ros2_control` plugin, injected directly into the generated *URDF*.
* **Built-in Teleoperation**: Launch the simulation and drive with an on-screen D-pad or the keyboard — no extra terminals needed.
* **Presets & Sessions**: Save/load chassis+sensor presets, or snapshot the entire wizard state (including manual *URDF* edits) and resume later.
* **Dual Output**: Every build produces both a flattened `mobRob.urdf` and a re-editable `mobRob.urdf.xacro`, and the 2-wheeled + caster now uses a 3-DOF free joint.

## Key Features

* 🧭 **Guided 9-Step Wizard**: A side navigation bar lets you go forward, jump back, and revisit any completed step.
* 🤖 **Three Chassis Families, Six Controllers**: Differential, mecanum, tricycle, tricycle-steering and Ackermann.
* 🛞 **Geometry-Driven Kinematics**: Wheel separation, wheelbase, track and the mecanum projection term are computed from the dimensions you enter and written straight into the controller *YAML*.
* 🎛️ **Controller Tuner**: Motion and steering limits, with physics-based **Auto-suggest**.
* 🔌 **Selectable Hardware Interface**: *Gazebo*, mock hardware, or a custom plugin.
* 📡 **Dynamic Multi-Sensor System**: Any number of lidars and cameras, live in the 3D preview, auto-bridged from *Gazebo* to *ROS 2*.
* 🕹️ **Built-in Teleoperation**: On-screen D-pad or keyboard (*WASD* / arrows, diagonals, strafe).
* 💾 **Presets & Sessions**: Reusable presets and full-state session snapshots.
* 🧊 **Dual Output**: Both `mobRob.urdf` and `mobRob.urdf.xacro`.

## Code Structure

```
Mobile_Robot_URDF_Maker/
├── src/
│   ├── mobRobURDF_wizard/                          # PyQt5 wizard application (main entry point)
│   │   ├── mobRobURDF_wizard/
│   │   │   ├── RobotWizard.py                       # Main wizard: navigation sidebar, page orchestration
│   │   │   ├── classes/
│   │   │   │   ├── OpenGLWidget.py                  # Live OpenGL 3D robot preview
│   │   │   │   ├── URDFManager.py                   # URDF / xacro generation and flattening
│   │   │   │   ├── launch_manager.py                # Launches the Gazebo + RViz simulation
│   │   │   │   ├── sensor_config.py                 # Dynamic sensor model (lidars + cameras)
│   │   │   │   ├── SensorEditor.py                  # Per-sensor configuration cards
│   │   │   │   ├── cards.py                         # Robot / controller selection cards
│   │   │   │   ├── responsive_widgets.py            # Responsive buttons and layouts
│   │   │   │   └── pages/
│   │   │   │       ├── WelcomePage.py
│   │   │   │       ├── StartSessionPage.py          # New project / load saved session
│   │   │   │       ├── RobotTypeSelectionPage.py    # Choose chassis family
│   │   │   │       ├── ControlConfigurationPage.py  # Pick a compatible controller
│   │   │   │       ├── ConfigurationPage.py         # Dimensions, sensors, 3D preview
│   │   │   │       ├── ControllerTunerPage.py       # Limits + hardware interface + Auto-suggest
│   │   │   │       ├── FinalCheckPage.py            # Review / edit URDF, save session
│   │   │   │       ├── TeleoperationPage.py         # Launch the sim and drive
│   │   │   │       └── FutureFeaturesPage.py
│   │   │   └── utils/
│   │   │       ├── presets.py                       # Preset + session save / load
│   │   │       ├── style.py                         # Stylesheet
│   │   │       └── utils.py
│   │   └── images/                                  # Wizard imagery (robot types, controllers, future)
│   ├── mobRobURDF_description/                      # Xacro templates, macros, Gazebo files, generated URDF
│   │   └── urdf/
│   │       ├── mobRob.urdf                          # Generated flattened URDF
│   │       ├── mobRob.urdf.xacro                    # Generated re-editable xacro
│   │       ├── submodules/                          # 2_wheeled_caster / 3_wheeled / 4_wheeled
│   │       ├── macros/                              # inertial + material macros
│   │       └── gazebo_files/                        # gazebo properties, colors, sensors, control
│   ├── mobRobURDF_control/                          # Per-controller ros2_control YAML configs
│   │   └── config/
│   │       ├── drive/                               # diff-drive (2WC, 4WD), mecanum
│   │       ├── steer/                               # tricycle-steering, Ackermann
│   │       └── gazebo_controller_tricycle.yaml
│   ├── mobRobURDF_gazebo/                           # Gazebo worlds, bridge config, use_sim_time
│   │   ├── worlds/                                  # empty_world.sdf, warehouse_world.sdf
│   │   └── config/                                  # gz_bridge.yaml, use_sim_time.yaml
│   ├── mobRobURDF_launch/                           # Launch files + cmd_vel_relay node
│   │   ├── launch/                                  # gazebo_test, urdf_test, slam_launch, nav_launch
│   │   └── mobRobURDF_launch/                       # cmd_vel_relay.py, ros_compat.py
│   ├── mobRobURDF_navigation/                       # Nav2 / SLAM configs and maps (upcoming features)
│   └── gz_ros2_control/                             # Vendored gz_ros2_control plugin (Harmonic)
├── LICENSE
└── README.md
```

> **Note**: The vendored `gz_ros2_control` is included unmodified under its original [Apache 2.0 licence](src/gz_ros2_control/LICENSE) to provide *Gazebo Harmonic* support on *ROS 2 Humble* (where the apt package targets *Fortress*). On *Jazzy* the apt package is already *Harmonic*, so the vendored copy must be excluded from the build with `COLCON_IGNORE`.

## Supported Robots and Controllers

| Robot type | Controller | `ros2_control` plugin |
|---|---|---|
| **2-Wheeled + Caster** | Differential Drive | `diff_drive_controller/DiffDriveController` |
| **3-Wheeled (Tricycle)** | Tricycle | `tricycle_controller/TricycleController` |
| **3-Wheeled (Tricycle)** | Tricycle Steering | `tricycle_steering_controller/TricycleSteeringController` |
| **4-Wheeled** | Differential Drive (skid-steer) | `diff_drive_controller/DiffDriveController` |
| **4-Wheeled** | Mecanum Drive | `mecanum_drive_controller/MecanumDriveController` |
| **4-Wheeled** | Ackermann Steering | `ackermann_steering_controller/AckermannSteeringController` |

The wizard only offers the controllers that are valid for the chosen chassis. The matching controller config lives under `src/mobRobURDF_control/config/` and is regenerated from your parameters every time you click **Apply**.

![Controller List](https://github.com/user-attachments/assets/f71a06c0-73a0-431b-a4ce-6027c849359e)

## Installation and Usage

### Prerequisites

* **ROS 2**: *Humble* (*Ubuntu 22.04*) or *Jazzy* (*Ubuntu 24.04*).
* **Python Dependencies** (also declared in `package.xml`, so `rosdep` covers them):
  ```bash
  sudo apt install python3-pyqt5 python3-pyqt5.qtopengl python3-opengl python3-ruamel.yaml
  ```
* **Gazebo Harmonic + the Harmonic-built `ros_gz` stack**:

  | Distro | Install |
  |---|---|
  | *Humble* | `sudo apt install gz-harmonic ros-humble-ros-gzharmonic ros-humble-gz-ros2-control` |
  | *Jazzy* | `sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control` |

  On *Humble* the default `ros_gz` packages target *Fortress*, while this project uses *Harmonic* — so install the *Harmonic* variant above, and **keep** the vendored `gz_ros2_control` (built for *Harmonic*) which shadows the *Fortress* apt build.

  On *Jazzy*, `ros_gz` already targets *Harmonic*, so the apt packages are all you need — **and the vendored `gz_ros2_control` must be excluded from the build**:
  ```bash
  touch src/gz_ros2_control/COLCON_IGNORE
  ```

### Setup

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/ali-pahlevani/Mobile_Robot_URDF_Maker.git
   cd Mobile_Robot_URDF_Maker
   ```

2. **Install Dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build and Source**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Run the Wizard**:
   ```bash
   ros2 run mobRobURDF_wizard mobRobURDF_wizard
   ```

   Or launch the *Gazebo* + *RViz* simulation directly (the wizard does this for you on the **Simulate & Teleop** page):
   ```bash
   ros2 launch mobRobURDF_launch gazebo_test.launch.py
   # choose a different world:
   ros2 launch mobRobURDF_launch gazebo_test.launch.py world:=/path/to/world.sdf
   # target a different Gazebo version (defaults to 8 / Harmonic):
   ros2 launch mobRobURDF_launch gazebo_test.launch.py gz_version:=6
   ```

> **Note**: On *Ubuntu 24.04* avoid bare `pip install` (*PEP 668*); use the apt packages above, or a virtualenv if you must use pip.

### Troubleshooting

* **PyQt5 / Display Errors**: Make sure a display server is running. On *WSL*, set `export DISPLAY=:0` or use an *X server* like *Xming*.
* **Controllers Fail to Load on Humble**: After a *clean* `colcon build` on *Humble*, confirm the vendored `gz_ros2_control` built for *Harmonic* and not *Fortress* — a *Fortress* build silently breaks every controller. Rebuild it explicitly with `GZ_VERSION=harmonic` if needed.
* **Gazebo Not Found**:
  ```bash
  gz sim --version   # expect 8.x (Harmonic)
  ```
* **Robot Won't Move**: On the **Simulate & Teleop** page, click **Connect to /cmd_vel** first; the `cmd_vel_relay` node bridges `/cmd_vel` to whatever topic the active controller expects.
* **Path Issues**: If images or worlds are not found, re-source `install/setup.bash` after a fresh build.

## Tutorial: Building a Complete Mobile Robot

The wizard walks you through every step sequentially. Each page can be revisited using the left-hand navigation sidebar.

### Step 1: Welcome

A project intro. Click *Next* to begin.

![Welcome Page](https://github.com/user-attachments/assets/dbc519d5-a536-45b6-aaac-b4622cc32f55)

### Step 2: Start Project

Begin a **new project**, or **load a saved session** (`*.mobsession`) to resume exactly where you left off. The detected *ROS 2* environment (`$ROS_DISTRO`) is shown here.

![Start Project](https://github.com/user-attachments/assets/7f4a70c4-9bd8-474e-8cc9-dbed55a854f2)

### Step 3: Select Robot Type

Choose **4-wheeled**, **3-wheeled (tricycle)**, or **2-wheeled + caster**.

![Select Robot Type](https://github.com/user-attachments/assets/85e3500d-acf3-45f1-a23e-6cffdcbc1158)

### Step 4: Select Controller

Pick a controller compatible with that chassis — incompatible options are disabled automatically.

![Select Controller](https://github.com/user-attachments/assets/e6def256-380a-4857-a7dc-8cc0871babb8)

### Step 5: Configure Parameters

Set chassis size/mass/material and wheel dimensions, add sensors, and watch the **live 3D preview**. *Apply and Preview* regenerates the *URDF*; *Save URDF to Folder* exports it.

* **Add Sensors**: Click **+ Lidar** or **+ Camera** to add as many sensors as you need. Each gets its own card where you set its **pose** (x, y, z, roll, pitch, yaw), **visual/physics** (color, mass, geometry), and type-specific settings — **lidar** (samples, min/max angle, range, update rate) or **camera** (FOV, resolution, clip planes, update rate).
* Every sensor appears immediately in the 3D preview at its exact pose and color. On build, the wizard writes `user_sensors.xacro` and a generated `gz_bridge_generated.yaml`, so all sensor topics are bridged from *Gazebo* to *ROS 2* automatically.

![Configure Parameters](https://github.com/user-attachments/assets/71a247b8-ce8a-41ab-9591-1473de7ec17e)

### Step 6: Tune Controller

Review the auto-computed geometry (read-only), pick the **hardware interface** (*Gazebo*, mock hardware, or a custom plugin), and set motion/steering limits — manually or via **Auto-suggest**.

![Tune Controller](https://github.com/user-attachments/assets/01ccf15e-2143-4de0-9d2f-ad02f5f6deb8)

### Step 7: Final Check

Review the generated *URDF* in an editor, make manual edits, copy to clipboard, **Save URDF**, or **Save Session**.

![Final Check](https://github.com/user-attachments/assets/2707b4ae-4a44-47d7-8210-37251cf77dab)

### Step 8: Simulate & Teleop

Click **Launch Simulation**. The launch file (`mobRobURDF_launch/gazebo_test.launch.py`) brings up everything for you:

* *Gazebo Harmonic* with the selected world
* the robot spawned from `robot_description`
* `controller_manager` with your chosen controller + `joint_state_broadcaster`
* the `ros_gz` parameter bridge (clock + all sensor topics) and image bridge
* *RViz2* with a preconfigured view
* a `cmd_vel_relay` node

![Gazebo Scene](https://github.com/user-attachments/assets/c66fa3c5-59e6-4d80-b096-1d1de728fc7c)

**Driving the robot.** Different controllers listen on different command topics (`cmd_vel_unstamped`, `cmd_vel`, `reference_unstamped`, …). The built-in `cmd_vel_relay` hides this: it subscribes to a single **`/cmd_vel`** and republishes to whatever the active controller expects. So you can just:

* use the on-screen **D-pad** or the **keyboard** (`W A S D` / arrow keys; `Q E Z C` for diagonals; `Space` for emergency stop) right inside the wizard, after clicking **Connect to /cmd_vel**, **or**
* publish to `/cmd_vel` yourself from any tool:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

For the **mecanum** controller, the **Strafe Mode** toggle remaps left/right to lateral (`linear.y`) motion so the robot slides sideways instead of turning.

> All bundled controllers publish the `odom → base_link` TF and the odometry topic themselves (`enable_odom_tf: true`), so no extra odometry node is required.

![RViz2](https://github.com/user-attachments/assets/d2c4965e-c791-4a62-bfbb-2ac98521ec14)

### Step 9: Future Features

A look at what's coming next.

![Future Features](https://github.com/user-attachments/assets/a098dc10-00b9-44cb-b065-6d63ba819e20)

## Saving Your Work: URDF, Presets and Sessions

* **URDF** — every build writes both `mobRob.urdf` (flattened) and `mobRob.urdf.xacro` (re-editable) into `mobRobURDF_description/urdf/`, and you can export copies anywhere.
* **Presets** (`*.yaml`) — capture robot type, controller, chassis/wheel parameters and the full sensor list. Saved to `~/mobRobURDF_presets/` by default.
* **Sessions** (`*.mobsession`) — snapshot the **entire** wizard state: parameters, sensors, tuner values, hardware interface and the current *URDF* text (including any manual edits). Saved to `~/mobRobURDF_sessions/`. Reload one from the **Start Project** page to continue right where you stopped.

## Customisation

You always retain full access to the underlying files:

* **Gazebo physics**: `mobRobURDF_description/urdf/gazebo_files/gazebo_properties.xacro`
* **Worlds**: drop a new `.sdf` into `mobRobURDF_gazebo/worlds/` and pass it via `world:=`
* **Controller configs**: `mobRobURDF_control/config/` (regenerated on Apply, but yours to edit afterwards)
* **Xacro templates**: everything under `mobRobURDF_description/urdf/` is editable

## Future Visions

**Mobile Robot URDF Maker** is a foundation for an open-source mobile-robot description and simulation builder. Planned enhancements, shown on the wizard's **Future Features** page, include:

* **More Robot Models and Kinematics** — *in progress*
* **SLAM** — *in progress*
* **Navigation (Nav2)** — *planned*
* **Fleet Management** — *planned*
* **And definitely a lot more!**

I'd **love collaborations**! Contribute via pull requests on *GitHub* for bug fixes, new features, or documentation improvements. Open a *GitHub Issue* for questions, suggestions, or partnership ideas.

## Contributing

1. Fork the repository.
2. Create a branch: `git checkout -b feature/your-feature`
3. Commit changes: `git commit -m "Add your feature"`
4. Push: `git push origin feature/your-feature`
5. Open a pull request.

Please include documentation updates. For major changes, open a *GitHub Issue* first to discuss the approach.

---

+ Questions? Reach out: **a.pahlevani1998@gmail.com**
+ LinkedIn: **https://www.linkedin.com/in/ali-pahlevani/**

---

> ⭐ If you find this useful, please star the repo — and stay tuned for the next versions!
