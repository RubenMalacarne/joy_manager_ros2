from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        output='screen'
    )

    ds4_node = Node(
        package='joy_manager',
        executable='dualshock_ps4',
        name='dualshock4_joystick_controller',
        output='screen',
        parameters=[{
            # Topic configurabili da CLI
            'roll_topic': 'joystick/roll',
            'pitch_topic': '/arganello/sx/target_torque',
            'yaw_topic': 'joystick/yaw',
            'thrust_topic': '/arganello/dx/target_torque',
            #services
            'button_X_service': 'joystick/button_X',
            'button_Circle_service': 'joystick/button_Circle',
            'button_Triangle_service': 'joystick/button_Triangle',
            'button_Square_service': 'joystick/button_Square',
            'button_L1_service': 'joystick/button_L1',
            'button_R1_service': 'joystick/button_R1',
            'button_L2_service': 'joystick/button_L2',
            'button_R2_service': 'joystick/button_R2',
            'button_Share_service': 'joystick/button_Share',
            'button_Options_service': 'joystick/button_Options',
            'button_L3_service': 'joystick/button_L3',
            'button_R3_service': 'joystick/button_R3',
            'axis_6_right_srv': 'joystick/axis_6_left',
            'axis_6_left_srv': 'joystick/axis_6_right',
            'axis_7_up_service': 'joystick/axis_7_up',
            'axis_7_down_service': 'joystick/axis_7_down',
            'l2_gear_topic': 'joystick/l2_gear',
            'r2_gear_topic': 'joystick/r2_gear',
            
            # Parametri di rimappatura assi
            'axis_0_min': -1.0, 'axis_0_max': 1.0,
            'axis_1_min': -1.0, 'axis_1_max': 1.0,
            'axis_2_min': -1.0, 'axis_2_max': 1.0,
            'axis_3_min': -1.0, 'axis_3_max': 1.0,
            'axis_4_min': -1.0, 'axis_4_max': 1.0,
            'axis_5_min': -1.0, 'axis_5_max': 1.0,
        }]
    )

    return LaunchDescription([joy_node, ds4_node])