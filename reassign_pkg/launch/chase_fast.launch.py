import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package=  'turtlesim',
            executable= 'turtlesim_node',
            output= 'screen'
        ),
        Node(
            package = 'reassign_pkg',
            executable= 'spawn_chase',
            output= 'screen',
        ),
        Node(
            package = 'reassign_pkg',
            executable= 'robber_rotate',
            output= 'screen'
        ),
        Node(
            package = 'reassign_pkg',
            executable= 'node4_v2',
            output= 'screen'
        ),
    ])