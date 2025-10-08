from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction  # aggiunto

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0'
        }]
    )

    delayed_joystick_node = TimerAction(
        period=2.0,  # delay in secondi
        actions=[
            Node(
                package='joy_manager',
                executable='joystick_radiomaster_pocket_bl',
                name='radiomaster_joystick_controller_bl',
                output='screen',
                parameters=[{
                    'roll_topic': 'joystick/roll', #axis 3
                    'pitch_topic': 'joystick/pitch', #axis 4
                    'yaw_topic': 'joystick/yaw', #axis 0
                    'thrust_topic': 'joystick/thrust', #axis 1
                    'gear_S1_topic': 'joystick/gear_S1', #axis 2
                    'switch_SC_service': 'joystick/switch_SC', #axis 5
                    'switch_SB_service': 'joystick/switch_SB', #axis 6
                    'button_SD_service': 'joystick/SD', #button 0
                    'button_SA_service': 'joystick/SA', #button 1
                    'button_SE_service': '/alpine/jump', #button 2
                    'button_SAB_service': 'joystick/SAB', #button 3
                    
                    'axis_0_min': 1.0, 'axis_0_max': -1.0,  # Roll (fisicamente axis 3, ma usa index 0 per remap)
                    'axis_1_min': -1.0, 'axis_1_max': 1.0,  # Pitch (fisicamente axis 4, ma usa index 1 per remap)
                    'axis_2_min': -1.0, 'axis_2_max': 1.0,  # Yaw (fisicamente axis 0, ma usa index 2 per remap)
                    'axis_3_min': -1.0, 'axis_3_max': 1.0,  # Thrust (fisicamente axis 1, ma usa index 3 per remap)
                    'axis_4_min': -1.0, 'axis_4_max': 1.0,   # Gear S1 (fisicamente axis 2, ma usa index 4 per remap)
                    
                    'switch_SC_low_label': 'LOW',
                    'switch_SC_center_label': 'CENTER', 
                    'switch_SC_high_label': 'HIGH',
                    
                    'switch_SB_low_label': 'LOW',
                    'switch_SB_center_label': 'CENTER',
                    'switch_SB_high_label': 'HIGH'
                }]
            )
        ]
    )

    return LaunchDescription([
        joy_node,
        delayed_joystick_node
    ])