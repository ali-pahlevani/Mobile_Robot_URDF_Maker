# Mobile_Robot_URDF_Maker

Automate the process of making a URDF for your 4-wheeled mobile robot using this **Wizard**

---

This workspace has 3 packages (up to now. I have plan for adding other packages as well for **Gazebo**, **Control**, **Navigation**, etc.):

1. mobRobURDF_description:
    - This package includes all the **template xacro** files necessary for creating the **final URDF file**. The *sub-directories* in this directory are:
        - **submodules:** contains **base.xacro**, **wheels.xacro**, and **sensors.xacro** files,
        - **macros:** contains **inertial_macros.xacro**, and **material.xacro** files,
        - **gazebo_files:** contains **gazebo_sensors.xacro** file for now (in the future, when the Gazebo package is added, not only this file will be used in the final URDF file, also other xacro files for Gazebo will be added as well (e.g., **gazebo_properties.xacro** file)).
    - Finally, all the xacro files are brought into the *main xacro file* (**mobRob.xacro**), so that it can be converted to **mobRob.xacro** file.

2. mobRobURDF_wizard:
    - This is the main package of the workspace. This package contains the source codes of the Wizard. 


3. mobRobURDF_launch:
    - This package includes 2 main directories:
        - **launch:** contains only one launch file for now for testing purposes (**urdf_test.launch.py**). By launching this launch files, 3 nodes will be launched:
            - **robot_state_publisher**
            - **joint_state_publisher_gui**
            - **rviz2**
        Using these nodes, everyone would be able to test the performance of the final URDF file (visual check + testing the joints)
        - **rviz:** contains only one rviz2 config file (rviz_test.rviz). This is the rviz2 config file that is loaded into the discussed launch file.
