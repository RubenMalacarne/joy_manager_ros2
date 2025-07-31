from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'roll_topic',
            default_value='joystick/roll',
            description='Topic per il roll'
        ),
        DeclareLaunchArgument(
            'pitch_topic',
            default_value='/arganello/sx/target_torque',
            description='Topic per il pitch'
        ),
        DeclareLaunchArgument(
            'yaw_topic',
            default_value='joystick/yaw',
            description='Topic per il yaw'
        ),
        DeclareLaunchArgument(
            'thrust_topic',
            default_value='/arganello/dx/target_torque',
            description='Topic per il thrust'
        ),
        DeclareLaunchArgument(
            'axis_0_min',
            default_value='-1.0',
            description='Valore minimo dell\'asse 0'
        ),
        DeclareLaunchArgument(
            'axis_0_max',
            default_value='1.0',
            description='Valore massimo dell\'asse 0'
        ),
    ]

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
            'roll_topic': LaunchConfiguration('roll_topic'),
            'pitch_topic': LaunchConfiguration('pitch_topic'),
            'yaw_topic': LaunchConfiguration('yaw_topic'),
            'thrust_topic': LaunchConfiguration('thrust_topic'),
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
            
            # Parametri di rimappatura assi (asse 0 configurabile da CLI)
            'axis_0_min': PythonExpression(['float(', LaunchConfiguration('axis_0_min'), ')']),
            'axis_0_max': PythonExpression(['float(', LaunchConfiguration('axis_0_max'), ')']),
            
            # Gli altri assi restano con i valori di default
            'axis_1_min': -1.0, 'axis_1_max': 1.0,
            'axis_2_min': -1.0, 'axis_2_max': 1.0,
            'axis_3_min': -1.0, 'axis_3_max': 1.0,
            'axis_4_min': -1.0, 'axis_4_max': 1.0,
            'axis_5_min': -1.0, 'axis_5_max': 1.0,
        }]
    )

    # ────── 4. LaunchDescription finale ──────
    return LaunchDescription(launch_args + [joy_node, ds4_node])