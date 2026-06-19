# Mobile Robot URDF Maker

> **Automate** the creation of a complete, simulation-ready **URDF** for your **mobile robot** — through a guided **wizard**, with no hand-written XML required.

![Preview_Image](https://github.com/user-attachments/assets/f117642b-6f3c-4057-a417-f05a30a2baa8)

**Mobile Robot URDF Maker** is a ROS 2 desktop application that walks you through building a mobile-robot description step by step: pick a chassis type, choose a `ros2_control` controller, set dimensions, add as many lidars and cameras as you like, tune the controller, and then **simulate and drive the robot in Gazebo — all from inside the wizard.** The result is a clean `.urdf` **and** `.urdf.xacro` pair plus matching controller configuration, ready to drop into your own stack.

- **Supported platforms:** ROS 2 **Humble** (Ubuntu 22.04) and ROS 2 **Jazzy** (Ubuntu 24.04) — the **same codebase** auto-adapts to whichever you build/source.
- **Simulator:** **Gazebo Harmonic** (`gz sim` 8) via `gz_ros2_control`
- **GUI:** PyQt5 with a live OpenGL 3D preview

> **One codebase, both distros.** The app detects `$ROS_DISTRO` at runtime and
> adjusts the parts that differ between distributions — chiefly the controller
> command interface (Humble/Iron use unstamped `Twist`; Jazzy+ use
> `TwistStamped`). The detected environment is shown on the **Start Project**
> page. You don't pick a distro in the app; it follows the environment you
> sourced.

---

## Table of Contents

- [Highlights](#highlights)
- [Supported robots and controllers](#supported-robots-and-controllers)
- [The wizard, page by page](#the-wizard-page-by-page)
- [Simulation and teleoperation](#simulation-and-teleoperation)
- [Sensors](#sensors)
- [Saving your work: URDF, presets and sessions](#saving-your-work-urdf-presets-and-sessions)
- [Workspace layout](#workspace-layout)
- [Installation](#installation)
- [Running](#running)
- [Customisation](#customisation)
- [Roadmap](#roadmap)
- [Version history](#version-history)
- [Contact](#contact)

---

## Highlights

- 🧭 **Guided 9-step wizard** with a side navigation bar — go forward, jump back, and revisit any completed step.
- 🤖 **Three chassis families, six controllers** — differential, mecanum, tricycle, tricycle-steering and Ackermann, each backed by a real `ros2_control` plugin.
- 🛞 **Geometry-driven kinematics** — wheel separation, wheelbase, track and the mecanum projection term are computed from the dimensions you enter and written straight into the controller YAML.
- 🎛️ **Controller Tuner** — edit publish/update rates, command timeout, velocity/acceleration and steering limits, or fill them with physics-based **Auto-suggest** values.
- 🔌 **Selectable hardware interface** — Gazebo simulation, mock hardware, or your own custom `ros2_control` plugin, injected directly into the generated URDF.
- 📡 **Dynamic multi-sensor system** — add any number of lidars and cameras, each fully configurable, with a live 3D preview and auto-generated Gazebo bridge.
- 🕹️ **Built-in teleoperation** — launch the simulation and drive with an on-screen D-pad or the keyboard (WASD / arrows, diagonals, strafe), no extra terminals needed.
- 💾 **Presets and sessions** — save/load chassis+sensor presets, or snapshot the entire wizard state (including manual URDF edits) and resume later.
- 🧊 **Dual output** — every build produces both a flattened `mobRob.urdf` and a re-editable `mobRob.urdf.xacro`.

---

## Supported robots and controllers

| Robot type | Controller | `ros2_control` plugin |
|---|---|---|
| **2-Wheeled + Caster** | Differential Drive | `diff_drive_controller/DiffDriveController` |
| **3-Wheeled (Tricycle)** | Tricycle | `tricycle_controller/TricycleController` |
| **3-Wheeled (Tricycle)** | Tricycle Steering | `tricycle_steering_controller/TricycleSteeringController` |
| **4-Wheeled** | Differential Drive (skid-steer) | `diff_drive_controller/DiffDriveController` |
| **4-Wheeled** | Mecanum Drive | `mecanum_drive_controller/MecanumDriveController` |
| **4-Wheeled** | Ackermann Steering | `ackermann_steering_controller/AckermannSteeringController` |

The wizard only offers the controllers that are valid for the chosen chassis. The matching controller config lives under `src/mobRobURDF_control/config/` and is regenerated from your parameters every time you click **Apply**.

![Controller_List](https://github.com/user-attachments/assets/945a5620-d423-44a1-b807-2aa61d8e1d83)

---

## The wizard, page by page

1. **Welcome** — project intro.
2. **Start Project** — begin a **new project** or **load a saved session** to resume exactly where you left off.
3. **Select Robot Type** — choose 4-wheeled, 3-wheeled (tricycle) or 2-wheeled + caster.
4. **Select Controller** — pick a controller compatible with that chassis (incompatible options are disabled).
5. **Configure Parameters** — set chassis size/mass/material and wheel dimensions, add sensors, and watch the **live 3D preview**. *Apply and Preview* regenerates the URDF; *Save URDF to Folder* exports it.
6. **Tune Controller** — review auto-computed geometry (read-only), pick the **hardware interface**, and set motion/steering limits (manually or via **Auto-suggest**).
7. **Final Check** — review the generated URDF in an editor, make manual edits, copy to clipboard, **Save URDF**, or **Save Session**.
8. **Simulate & Teleop** — launch Gazebo + RViz and drive the robot.
9. **Future Features** — a look at what's coming next.

---

## Simulation and teleoperation

From the **Simulate & Teleop** page, click **Launch Simulation**. The launch file
(`mobRobURDF_launch/gazebo_test.launch.py`) brings up everything for you:

- Gazebo Harmonic with the selected world
- the robot spawned from `robot_description`
- `controller_manager` with your chosen controller + `joint_state_broadcaster`
- the `ros_gz` parameter bridge (clock + all sensor topics) and image bridge
- RViz2 with a preconfigured view
- a `cmd_vel_relay` node

![Gazebo_Scene](https://github.com/user-attachments/assets/2abbf489-6c52-4d09-b639-26b8e7e7771c)

**Driving the robot.** Different controllers listen on different command topics
(`cmd_vel_unstamped`, `cmd_vel`, `reference_unstamped`, …). The built-in
`cmd_vel_relay` hides this: it subscribes to a single **`/cmd_vel`** and republishes
to whatever the active controller expects. So you can just:

- use the on-screen **D-pad** or the **keyboard** (`W A S D` / arrow keys; `Q E Z C`
  for diagonals; `Space` for emergency stop) right inside the wizard, after clicking
  **Connect to /cmd_vel**, **or**
- publish to `/cmd_vel` yourself from any tool, e.g.:

  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

For the **mecanum** controller, the **Strafe Mode** toggle remaps left/right to lateral
(`linear.y`) motion so the robot slides sideways instead of turning.

> All bundled controllers publish the `odom → base_link` TF and the odometry topic
> themselves (`enable_odom_tf: true`), so no extra odometry node is required.

You can also launch the simulation directly from a terminal:

```bash
ros2 launch mobRobURDF_launch gazebo_test.launch.py
# choose a different world:
ros2 launch mobRobURDF_launch gazebo_test.launch.py world:=/path/to/world.sdf
```

![Rviz2](https://github.com/user-attachments/assets/e21d173e-c43c-43de-a6be-9f3a55366c62)

---

## Sensors

The **Configure Parameters** page hosts a dynamic sensor list. Click **+ Lidar** or
**+ Camera** to add as many sensors as you need; each gets its own card where you set:

- **Pose** relative to the chassis (x, y, z, roll, pitch, yaw)
- **Visual/physics** (color, mass, geometry)
- **Lidar**: samples, min/max angle, range, update rate
- **Camera**: FOV, image resolution, clip planes, update rate

Every sensor appears immediately in the 3D preview at its exact pose and color. On
build, the wizard writes `user_sensors.xacro` and a generated `gz_bridge_generated.yaml`
(plus the image-topic list), so all sensor topics are bridged from Gazebo to ROS 2
automatically.

---

## Saving your work: URDF, presets and sessions

- **URDF** — every build writes both `mobRob.urdf` (flattened) and `mobRob.urdf.xacro`
  (re-editable) into `mobRobURDF_description/urdf/`, and you can export copies anywhere.
- **Presets** (`*.yaml`) — capture robot type, controller, chassis/wheel parameters and
  the full sensor list. Saved to `~/mobRobURDF_presets/` by default.
- **Sessions** (`*.mobsession`) — snapshot the **entire** wizard state: parameters,
  sensors, tuner values, hardware interface and the current URDF text (including any
  manual edits). Saved to `~/mobRobURDF_sessions/`. Reload one from the **Start Project**
  page to continue right where you stopped.

---

## Workspace layout

| Package | Purpose |
|---|---|
| `mobRobURDF_wizard` | The PyQt5 wizard application (main entry point). |
| `mobRobURDF_description` | Xacro templates, macros, Gazebo files and the generated URDF. |
| `mobRobURDF_control` | Per-controller `ros2_control` YAML configs. |
| `mobRobURDF_gazebo` | Gazebo worlds, bridge config and `use_sim_time` settings. |
| `mobRobURDF_launch` | Launch files (Gazebo test, URDF test) + the `cmd_vel_relay` node. |
| `mobRobURDF_navigation` | Nav2 / SLAM configs and maps (used by upcoming features). |
| `gz_ros2_control` | Vendored [`gz_ros2_control`](https://github.com/ros-controls/gz_ros2_control) plugin (Harmonic), built in-workspace. Included unmodified under its original [Apache 2.0 licence](src/gz_ros2_control/LICENSE) to provide Gazebo Harmonic support on ROS 2 Humble (the apt package there targets Fortress). On Jazzy, add `COLCON_IGNORE` — the apt package is already Harmonic. |

---

## Installation

**1. Clone the workspace**

```bash
git clone https://github.com/ali-pahlevani/Mobile_Robot_URDF_Maker.git
cd Mobile_Robot_URDF_Maker
```

**2. Install Gazebo Harmonic and the Harmonic-built `ros_gz` stack**

<details open>
<summary><b>ROS 2 Humble (Ubuntu 22.04)</b></summary>

On Humble the default `ros_gz` packages target *Fortress*, while this project uses
*Harmonic*, so install the Harmonic variant. The workspace also **vendors**
`gz_ros2_control` (built for Harmonic) to shadow the Fortress apt build — keep it.

```bash
sudo apt update
sudo apt install gz-harmonic
sudo apt install ros-humble-ros-gzharmonic ros-humble-gz-ros2-control
```
</details>

<details>
<summary><b>ROS 2 Jazzy (Ubuntu 24.04)</b></summary>

On Jazzy, `ros_gz` already targets *Harmonic*, so the apt packages are all you need —
**and the vendored `gz_ros2_control` must be excluded from the build** (the apt one is
already Harmonic):

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control

# Exclude the Humble-pinned vendored plugin from the build:
touch src/gz_ros2_control/COLCON_IGNORE
```
</details>

**3. Install ROS and Python dependencies**

```bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install python3-pyqt5 python3-pyqt5.qtopengl python3-opengl python3-ruamel.yaml
```

> All Python deps (`python3-opengl`, `python3-ruamel.yaml`) are declared in
> `package.xml`, so `rosdep install` already covers them. On Ubuntu 24.04 avoid bare
> `pip install` (PEP 668); use the apt packages above, or a virtualenv if you must pip.

**4. Build and source**

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running

**Launch the wizard:**

```bash
ros2 run mobRobURDF_wizard mobRobURDF_wizard
```

**Launch the Gazebo + RViz simulation directly** (the wizard does this for you on the
Simulate & Teleop page):

```bash
ros2 launch mobRobURDF_launch gazebo_test.launch.py
```

The launch files default to **Gazebo 8 (Harmonic)**. To target another version
(e.g. Fortress) pass `gz_version`:

```bash
ros2 launch mobRobURDF_launch gazebo_test.launch.py gz_version:=6
```

---

## Customisation

You always retain full access to the underlying files:

- **Gazebo physics:** `mobRobURDF_description/urdf/gazebo_files/gazebo_properties.xacro`
- **Worlds:** drop a new `.sdf` into `mobRobURDF_gazebo/worlds/` and pass it via `world:=`
- **Controller configs:** `mobRobURDF_control/config/` (regenerated on Apply, but yours to edit afterwards)
- **Xacro templates:** everything under `mobRobURDF_description/urdf/` is editable

---

## Roadmap

Shown on the wizard's **Future Features** page:

- More robot models and kinematics — *in progress*
- SLAM — *in progress*
- Navigation (Nav2) — *planned*
- Fleet management — *planned*

---

## Version history

### Version 4 (current)
Six selectable `ros2_control` controllers, Gazebo Harmonic simulation, a dynamic
multi-sensor system, hardware-interface selection, a Controller Tuner, built-in
teleoperation, and full preset/session save & load. Generates both `.urdf` and
`.urdf.xacro`, and the 2WC caster now has a 3-DOF free joint.

### Version 2
Three chassis types to choose from (4-wheeled, tricycle, 2-wheeled + caster), an
organised wizard with a navigation bar, automatic URDF copy for the test launch file,
and a preview of upcoming features.

### Version 1
The original wizard for a single **4-wheeled** robot: chassis + 4 wheels + a 2D lidar
and RGB camera, with a 3D preview (rotate + zoom) and URDF export, plus a `urdf_test`
launch file (`robot_state_publisher` + `joint_state_publisher_gui` + RViz2) for visual
and joint checks.

---

## Contact

Questions or feedback? **a.pahlevani1998@gmail.com**

Check out the website: **https://www.SLAMbotics.org**

> ⭐ If you find this useful, please star the repo — and stay tuned for the next versions!
