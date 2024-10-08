import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'joy_hand_eye'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Irving Fang',
    maintainer_email='irving.fang@nyu.edu',
    description='The package perform hand-eye calibration. It expects the robot to be moved and pictures to be taken using a Joystick connected to the robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_client = joy_hand_eye.calibration_client:main',
            'calibration_server = joy_hand_eye.calibration_server:main',
        ],
    },
)
