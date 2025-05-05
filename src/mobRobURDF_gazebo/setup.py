from setuptools import setup
import os
from glob import glob

package_name = 'mobRobURDF_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Pahlevani',
    maintainer_email='a.pahlevani1998@gmail.com',
    description='Package for creating gazebo worlds',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },

)
