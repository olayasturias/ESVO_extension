from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='events_repacking_helper',
            executable='EventMessageEditor_Mono',
            name='EventMessageEditor_Mono',
            output='screen',
            arguments=[
                '/Monster/dataset/event_camera/ijrr_rpg_dataset/slider_far.bag',
                '/Monster/dataset/event_camera/ijrr_rpg_dataset/slider_far.bag.events'
            ],
            on_exit=[launch.actions.Shutdown()]
        )
    ])
