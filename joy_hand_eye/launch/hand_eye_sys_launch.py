
import os
from numpy import imag
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource



def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name='imaging_system', default_value='realsense_capture', description='The imaging system to use'),
            DeclareLaunchArgument(name='hand_eye_setup', default_value='eye_in_hand', description='What kind of hand-eye setup to use'),
            DeclareLaunchArgument(name='EEF_link', default_value='link_eef', description='The end effector frame'),
            DeclareLaunchArgument(name='base_link', default_value='link_base', description='The base frame'),
        ]
    )



def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)

    # try:
    with open(absolute_file_path) as file:
        return yaml.safe_load(file)
    # except OSError:  # parent of IOError, OSError *and* WindowsError where available
    #     return None
    
def generate_launch_description():

    imaging_system = LaunchConfiguration('imaging_system')
    hand_eye_setup = LaunchConfiguration('hand_eye_setup')
    EEF_link = LaunchConfiguration('EEF_link')
    base_link = LaunchConfiguration('base_link')


    ld = LaunchDescription()
    ld.add_entity(declare_arguments())
    
    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'), 
                                    'launch', 
                                    'foxglove_bridge_launch.xml')
        )
    )

    
    joy_launch = Node(
                executable='joy_node',
                package='joy',
                name='joy_node',
                parameters=[
                    # {'autorepeat_rate': 50.0},
                ],
            )

    calibration_client_launch = Node(
        package='joy_hand_eye',
        executable='calibration_client',
        name='calibration_client')
    
    calibration_server_launch = Node(
        package='joy_hand_eye',
        executable='calibration_server',
        name='calibration_server',
        parameters=[
            {'imaging_system': imaging_system},
            {'hand_eye_setup': hand_eye_setup},
            {'EEF_link': EEF_link},
            {'base_link': base_link},
        ]
    )

    # ld.add_action(foxglove_launch)
    # ld.add_action(joy_launch)
    # ld.add_action(usbcam_image_server_launch)
    # ld.add_action(usbcam_image_client_launch)
    # ld.add_action(realsense_image_server_launch)
    # ld.add_action(realsense_image_client_launch)
    ld.add_action(calibration_client_launch)
    ld.add_action(calibration_server_launch)

    return ld