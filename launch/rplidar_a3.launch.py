from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

configurable_parameters=[
    {'name': 'serial_port', 'default': '/dev/ttyUSB0', 'description': ''},
    {'name': 'serial_baudrate', 'default': '256000', 'description': ''}, # A3
    {'name': 'frame_id', 'default': 'base_scan', 'description': ''},
    {'name': 'inverted', 'default': 'false', 'description': ''},
    {'name': 'angle_compensate', 'default': 'true', 'description': ''},
    {'name': 'scan_mode', 'default': 'Sensitivity', 'description': ''},
    {'name': 'topic_name', 'default': 'scan', 'description': ''},
]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) + [
        Node(
            node_name='rplidar_composition',
            package='rplidar_ros2',
            node_executable='rplidar_composition',
            output='screen',
            parameters=[set_configurable_parameters(configurable_parameters)],
            emulate_tty=True
            ),
    ])
