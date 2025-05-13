# Mobile_Robot_URDF_Maker (v3)

**Automate** the process of making a **URDF** for your **mobile robot** using this "**Wizard**"

![Preview_Image](https://github.com/user-attachments/assets/28fcf660-d92d-4a99-831a-e6a29a9a5c78)

- Now, you don't have only 1 option (4-wheeled robot). In version 2, you'll have **3 options** to choose from (in the next version, I'll introduce **specific controllers** for each of these types, so that you can make **ros2_control config files** for each of these types of **kinematics**):
    - **4-Wheeled** Robot
    - **3-Wheeled** Robot (**Tricycle**)
    - **2-Wheeled** Robot with a **Caster Wheel**

![Wizard_Test](https://github.com/user-attachments/assets/948c821c-6540-4ec9-9b90-8443698d42aa)
 
- You can easily choose the type of robot you need to work with, build its URDF **automatically** (based on the parameters you choose), and get your built URDF file.
    - In this version, when you hit the save button, first a **copy of the URDF file** is **saved automatically** in a specific place for the launch file; so that **launch file** will recognize it automatically when you launch it (for **testing** purposes).
    - Other than that, you can also save **another copy** of it to any **directory you want** (or simply close the saving window if you **don't need** it!).

- As it be seen, the overall structure of the wizard has been enhanced and organized, so that not only you can **navigate to different pages** using **Navigation Bar**, also at the last page of the wizard (in this version), you can see the **upcoming updates** for the wizard in the next releases (adding *Gazebo* files, *SLAM* and *Navigation* capability, setting up *Controllers*, etc.).

---

## Just like the previous version, the path to run the wizard is as simple as you can see:
 
In order to run the **Wizard**, first you need to **clone** the workspace:

```bash
git clone https://github.com/ali-pahlevani/Mobile_Robot_URDF_Maker.git
cd Mobile_Robot_URDF_Maker
```
    
For the next step, you'll need to install the **dependencies**:

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install python3-pyqt5 python3-pyqt5.qtopengl
```

After that, you should **build** the workspace and **source** the installation:

```bash
colcon build --symlink-install
source install/setup.bash
```

Finally, you can easily run the following line in your **terminal**:

```bash
ros2 run mobRobURDF_wizard mobRobURDF_wizard
```
---

In order to run the launch file, you can run the following line in your **terminal**:

```bash
    ros2 launch mobRobURDF_launch urdf_test.launch.py
```

![Launch_Test](https://github.com/user-attachments/assets/cd118fb7-b052-4255-86fa-d7b7d663cfef)


---

If you have any question, please let me know: **a.pahlevani1998@gmail.com**

## Please stay tuned for the next versions of the app.

---
---
---

## Version 1

**Automate** the process of making a **URDF** for your **4-wheeled mobile robot** using this "**Wizard**"

---

This workspace has 3 **ROS2** packages (up to now. I have plan for adding other packages as well for **Gazebo**, **Control**, **Navigation**, etc.):

1. ### mobRobURDF_description:
    - This package includes all the **template Xacro** files necessary for creating the **final URDF** file. The *sub-directories* in this directory are:
        - **submodules:** containing **base.xacro** (*chassis*), **wheels.xacro** (*4 wheels*), and **sensors.xacro** (*2D-Lidar* + *RGB Camera*) files,
        - **macros:** containing **inertial_macros.xacro**, and **material.xacro** (*colors*) files,
        - **gazebo_files:** containing **gazebo_sensors.xacro** file for now (in the future, when the Gazebo package is added, not only this file will be used in the final URDF file, also other Xacro files for Gazebo will be added as well (e.g., **gazebo_properties.xacro** file)).
    - Finally, all the Xacro files are imported into the *main Xacro file* (**mobRob.xacro**), so that it can be converted to **mobRob.urdf** file. 
    - Good news is that not only you can use the Wizard to make the URDF file you need, but also you have **access** to all the Xacro files. If you ever wanted to **change** any of them and adapt them to your specific case, you're free to go.

---

2. ### mobRobURDF_wizard:
    - This is the *main package* of the workspace. This package contains the **source codes** of the Wizard. The codebase is composed of different classes and some utility functions, all imported into the main file **robot_wizard.py**. This package used to be a standalone codebase; however, now its a **ROS2 node**.
    - In the Wizard window, you can apply your changes step by step, with no pressure. You can even fill only *some of the fields* and only apply those changes. Finally, you can **save your created URDF file** to any directory you want. Additionally, for your convenience, you'll have **3-DOF camera rotation**, plus **zooming capability** in the preview window. 
    - **Attention:** If you want to use the launch file (that is provided for **testing purposes** (check the next part)), you'll need to **save** the created **URDF** file in the following location: **/mobRobURDF_description/urdf/mobRob.urdf**

    - In order to run the **Wizard**, first you need to **clone** the workspace:

    ```bash
    git clone https://github.com/ali-pahlevani/Mobile_Robot_URDF_Maker.git
    cd Mobile_Robot_URDF_Maker
    ```
    
    - For the next step, you'll need to install the **dependencies**:

    ```bash
    sudo apt update
    rosdep install --from-paths src --ignore-src -r -y
    sudo apt install python3-pyqt5 python3-pyqt5.qtopengl
    ```

    - After that, you should **build** the workspace and **source** the installation:

    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

    - Finally, you can easily run the following line in your **terminal**:

    ```bash
    ros2 run mobRobURDF_wizard mobRobURDF_wizard
    ```

---

3. ### mobRobURDF_launch:
    - This package includes 2 main directories:
        - **launch:** containing only one launch file (for now) for testing purposes (**urdf_test.launch.py**). By launching this launch file, 3 nodes will be launched:
            - **robot_state_publisher**
            - **joint_state_publisher_gui**
            - **rviz2**
        Using these nodes, everyone would be able to **test** the performance of the final URDF file (**visual check** + **testing the joints**)
        - **rviz:** containing only one *rviz2 config file* (**rviz_test.rviz**). This is the rviz2 config file that is loaded into the discussed launch file.

    - In order to run the launch file, you can run the following line in your **terminal**:

```bash
ros2 launch mobRobURDF_launch urdf_test.launch.py
```
