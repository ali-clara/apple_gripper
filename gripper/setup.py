from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Jones',
    maintainer_email='jonesal9@oregonstate.edu',
    description='Package for ROS2 conversion of the existing ROS1 apple suction gripper code developed by Alejo Velasquez',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user = gripper.user:main',
            'arm_control = gripper.arm_control:main',
            'suction_gripper = gripper.suction_gripper:main',
            'viz = gripper.viz:main',
            'proxy_pick = gripper.proxy_pick:main',
        ],
    },
)
