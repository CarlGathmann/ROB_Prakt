from launch import LaunchDescription
from launch_ros.actions import Node
#TODO: set_workspace Node needs to be added

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='morris_engine',
            namespace='morris_engine',
            executable='morris',
            name='morris'
        ),
        Node(
            package='robot_control',
            namespace='control_service',
            executable='runner',
            name='robot_control'
        ),
        Node(
            package='vacuum_control',
            namespace='vacuum_service',
            executable='gripper',
            name='vacuum_control'
        )
        Node(
            package='tracking_service',
            namespace='tracking_service',
            executable='tracking',
            name='tracking_service'
        )
    ])