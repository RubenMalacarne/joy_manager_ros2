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
            executable='joystick_radiomaster_pocket',
            name='radiomaster_joystick_controller',
            output='screen',
            parameters=[{
                'roll_topic': 'joystick/roll',
                'pitch_topic': 'joystick/pitch',
                'yaw_topic': 'joystick/yaw',
                'thrust_topic': 'joystick/thrust',
                'gear_S1_topic': 'joystick/gear_S1',
                'switch_SC_service': 'joystick/switch_SC',
                'switch_SB_service': 'joystick/switch_SB',
                'button_SD_service': 'joystick/SD',
                'button_SA_service': 'joystick/SA',
                'button_SE_service': 'joystick/SE',
                'button_SAB_service': 'joystick/SAB',
                
                'axis_0_min': -1.0, 'axis_0_max': 1.0,  # Roll
                'axis_1_min': -1.0, 'axis_1_max': 1.0,  # Pitch
                'axis_2_min': -1.0, 'axis_2_max': 1.0,  # Yaw
                'axis_3_min': -1.0, 'axis_3_max': 1.0,  # Thrust
                'axis_4_min': 0.0, 'axis_4_max': 1.0,   # Gear S1
                
                'switch_SC_low_label': 'LOW',
                'switch_SC_center_label': 'CENTER', 
                'switch_SC_high_label': 'HIGH',
                
                'switch_SB_low_label': 'LOW',
                'switch_SB_center_label': 'CENTER',
                'switch_SB_high_label': 'HIGH'
            }]
        )
    ])