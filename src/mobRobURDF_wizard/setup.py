from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mobRobURDF_wizard'

setup(
    name=package_name,
    version='0.0.0',
    # Include sub-packages (classes, classes.pages, utils) so the wizard also
    # works with a non-symlink install, not only `colcon build --symlink-install`.
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'images', 'future_features'), glob('images/future_features/*.png')),
        (os.path.join('share', package_name, 'images', 'robot_types'), glob('images/robot_types/*.png')),
        (os.path.join('share', package_name, 'images', 'control_types'), glob('images/control_types/*.png')),
        (os.path.join('share', package_name, 'images'), glob('images/*.gif')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Pahlevani',
    maintainer_email='a.pahlevani1998@gmail.com',
    description='Package for generating URDF files with a GUI wizard.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobRobURDF_wizard = mobRobURDF_wizard.RobotWizard:main',
        ],
    },
)