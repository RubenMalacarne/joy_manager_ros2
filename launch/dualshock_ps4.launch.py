from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0'
            }]
        ),
        Node(
            package='joy_manager',
            executable='dualshock_ps4',
            name='dualshock4_joystick_controller',
            output='screen',
            parameters=[{
                'rpyt_topic': 'joystick_dualshock4/rpyt',
                'button_X_topic': 'joystick/button_X',
                'button_Circle_topic': 'joystick/button_Circle',
                'button_Triangle_topic': 'joystick/button_Triangle',
                'button_Square_topic': 'joystick/button_Square',
                'button_L1_topic': 'joystick/button_L1',
                'button_R1_topic': 'joystick/button_R1',
                'button_L2_topic': 'joystick/button_L2',
                'button_R2_topic': 'joystick/button_R2',
                'button_Share_topic': 'joystick/button_Share',
                'button_Options_topic': 'joystick/button_Options',
                'button_L3_topic': 'joystick/button_L3',
                'button_R3_topic': 'joystick/button_R3',
                'r2_gear_topic': 'joystick/r2_gear',
                'r2_gear_topic': 'joystick/r2_gear'
            }]
        )
    ])
    