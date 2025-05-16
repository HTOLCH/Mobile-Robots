import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoDrive',
            executable='autoDrive',
            name='autoDrive'),

        Node(
            package='phidget',
            executable='phidget',
            name='phidget'),

        DeclareLaunchArgument(
            'serial_port', 
            default_value='/dev/ttyACM0',  # Default port if no argument is provided
            description='Serial port for the GPS device'
        ),
        DeclareLaunchArgument(
            'baud_rate', 
            default_value='9600',  # Default baud rate
            description='Baud rate for the GPS device'
        ),
        # Launch the nmea_serial_driver node with the serial port and baud rate as parameters
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baud_rate'),
                'frame_id': 'gps_link',
            }]
        ),

        # Including the lakibeam1 driver launch file correctly
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('/workspace/src/Lakibeam_ROS2_Driver/launch/lakibeam1_scan.launch.py')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('/workspace/src/autoDrive/launch/spatial.launch.py')
        )

    ])
