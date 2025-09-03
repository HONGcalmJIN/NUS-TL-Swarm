from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Group action with namespace
        GroupAction([
            PushRosNamespace('agent001'),

            # Node configuration
            Node(
                package='pbtm_ros2',
                executable='pbtm_ros2_node',
                name='pbtm_ros2_node',
                output='screen',
                parameters=[
                    {'agent_id': 'agent001'},
                    {'send_command_rate': 20.0},
                    {'timeout': 0.5},
                    {'takeoff_height': 1.3},
                    {'global_start_position': [1.0, 0.0, 0.0]},  # In NWU coordinate system
                    {'height_range': [1.0, 5.0]},
                    {'yaw_offset_rad': 0.0},
                    {'order': 4.0},
                    {'max_velocity': 2.0},
                    {'knot_division': 3}
                ]
            ),
        ])
    ])

