from setuptools import find_packages, setup

package_name = 'joy_hand_eye_ROS2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irving',
    maintainer_email='zichuanfang2015@yahoo.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_client = joy_hand_eye_ROS2.calibration_client:main'
        ],
    },
)
