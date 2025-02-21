from setuptools import setup
import os

package_name = 'mobRobURDF_wizard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
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