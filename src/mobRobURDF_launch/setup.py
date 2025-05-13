from setuptools import find_packages, setup

package_name = 'mobRobURDF_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gazebo_test.launch.py']),
        ('share/' + package_name + '/launch', ['launch/urdf_test.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/rviz_gazebo_test.rviz']),
        ('share/' + package_name + '/rviz', ['rviz/rviz_test.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Pahlevani',
    maintainer_email='a.pahlevani1998@gmail.com',
    description='Package for creating launch file for testing the generated URDF',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
