import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Read the ROS_IP environment variable
    ros_ip_env = os.getenv('ROS_IP', '0.0.0.0:10000')
    
    # Split the environment variable into IP and port
    ip_address, port = ros_ip_env.split(':')
    
    # Convert port to an integer
    port = int(port)

    return LaunchDescription(
        [
            Node(
                package="tcp_endpoint_ros2",
                executable="default_server_endpoint",
                emulate_tty=True,
                parameters=[{"ROS_IP": ip_address}, {"ROS_TCP_PORT": port}],
            ),
        ]
    )
