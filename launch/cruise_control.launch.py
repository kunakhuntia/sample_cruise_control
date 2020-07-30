import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


global_parameters = os.path.join(
        get_package_share_directory('cruise_control'),
        'config',
        'ParamServer.yaml'
    )


def generate_launch_description():
    return LaunchDescription([
        Node(
        package='cruise_control',
        executable='Paramserver',
        name='parameter_server',
        parameters=[global_parameters]
        ),
        Node(
            package='cruise_control',
            namespace='',
            executable='sensor',
            name='SensorNode',
            parameters=[global_parameters]
        ),
        Node(
            package='cruise_control',
            namespace='',
            executable='algorithm',
            name='AlgorithmNode',
            parameters=[global_parameters]
        )])
        
