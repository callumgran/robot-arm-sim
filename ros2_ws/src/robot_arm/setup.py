from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
		('share/' + package_name + '/config', ['config/robot_arm_controllers.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Callum',
    maintainer_email='callum.gran@gmail.com',
    description='Robot arm simulation in Gazebo with ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_arm = robot_arm.simple_arm_node:main',
        ],
    },
)
