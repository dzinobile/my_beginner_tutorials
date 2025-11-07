from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
            package='beginner_tutorials',
            executable='talker',
            name='publisher',
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    
    listener_node = Node(
        package='beginner_tutorials',
            executable='listener',
            name='subscriber',
            output='screen'
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)
    
    return ld




