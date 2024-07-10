from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='az_body_tracker',
                executable='az_body_tracker',
                parameters=[
                    {'preset_path': '/home/do/ros2_ws/src/az_body_tracker/params/preset.yaml'}
                ],
                output='screen',
                emulate_tty=True
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', '/home/do/ros2_ws/src/az_body_tracker/cfg/view.rviz'],
                output='screen'
            ),
            Node(
                package='rqt_reconfigure',
                executable='rqt_reconfigure',
                output='screen'
            )


        ]
    )
