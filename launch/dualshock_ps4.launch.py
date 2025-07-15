# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='joy',
#             executable='joy_node',
#             name='joy_node',
#             parameters=[{
#                 'dev': '/dev/input/js0'
#             }]
#         ),
#         Node(
#             package='joy_manager',
#             executable='dualshock_ps4',
#             name='dualshock4_joystick_controller',
#             output='screen',
#             parameters=[{
#                 'rpyt_topic': 'joystick/rpyt',
#                 'button_X_topic': 'joystick/button_X',
#                 'button_Circle_topic': 'joystick/button_Circle',
#                 'button_Triangle_topic': 'joystick/button_Triangle',
#                 'button_Square_topic': 'joystick/button_Square',
#                 'button_L1_topic': 'joystick/button_L1',
#                 'button_R1_topic': 'joystick/button_R1',
#                 'button_L2_topic': 'joystick/button_L2',
#                 'button_R2_topic': 'joystick/button_R2',
#                 'button_Share_topic': 'joystick/button_Share',
#                 'button_Options_topic': 'joystick/button_Options',
#                 'button_L3_topic': 'joystick/button_L3',
#                 'button_R3_topic': 'joystick/button_R3',
#                 'l2_gear_topic': 'joystick/l2_gear',
#                 'r2_gear_topic': 'joystick/r2_gear',
#                 # Parametri per la rimappatura degli assi (valori di default)
#                 'axis_0_min': -1.0,    # Stick sinistro X min
#                 'axis_0_max': 1.0,     # Stick sinistro X max
#                 'axis_1_min': -1.0,    # Stick sinistro Y min
#                 'axis_1_max': 1.0,     # Stick sinistro Y max
#                 'axis_2_min': -1.0,    # L2 gear min
#                 'axis_2_max': 1.0,     # L2 gear max
#                 'axis_3_min': -1.0,    # Stick destro X min
#                 'axis_3_max': 1.0,     # Stick destro X max
#                 'axis_4_min': -1.0,    # Stick destro Y min
#                 'axis_4_max': 1.0,     # Stick destro Y max
#                 'axis_5_min': -1.0,    # R2 gear min
#                 'axis_5_max': 1.0,     # R2 gear max
#             }]
#         )
#     ])


# dualshock_ps4.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # ────── 1. Argomenti lanciabili da CLI ──────
    launch_args = [
        DeclareLaunchArgument(
            'rpyt_topic',
            default_value='joystick/rpyt',
            description='Topic per l’array RPYT (o solo asse 0 se usi un filtro a valle)'
        ),
        DeclareLaunchArgument(
            'axis_0_min',
            default_value='-1.0',
            description='Valore minimo dell’asse 0'
        ),
        DeclareLaunchArgument(
            'axis_0_max',
            default_value='1.0',
            description='Valore massimo dell’asse 0'
        ),
        # → se vuoi rendere configurabili altri assi, replica qui sopra
    ]

    # ────── 2. Nodo joy_node (driver) ──────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        output='screen'
    )

    # ────── 3. Nodo di gestione DualShock 4 ──────
    ds4_node = Node(
        package='joy_manager',
        executable='dualshock_ps4',
        name='dualshock4_joystick_controller',
        output='screen',
        parameters=[{
            # Topic (quello che ora potrai cambiare da CLI)
            'rpyt_topic': LaunchConfiguration('rpyt_topic'),

            # Tutti gli altri topic rimangono fissi
            'button_X_topic':        'joystick/button_X',
            'button_Circle_topic':   'joystick/button_Circle',
            'button_Triangle_topic': 'joystick/button_Triangle',
            'button_Square_topic':   'joystick/button_Square',
            'button_L1_topic':       'joystick/button_L1',
            'button_R1_topic':       'joystick/button_R1',
            'button_L2_topic':       'joystick/button_L2',
            'button_R2_topic':       'joystick/button_R2',
            'button_Share_topic':    'joystick/button_Share',
            'button_Options_topic':  'joystick/button_Options',
            'button_L3_topic':       'joystick/button_L3',
            'button_R3_topic':       'joystick/button_R3',
            'l2_gear_topic':         'joystick/l2_gear',
            'r2_gear_topic':         'joystick/r2_gear',

            # Parametri di rimappatura assi (asse 0 configurabile da CLI)
            'axis_0_min': PythonExpression(['float(', LaunchConfiguration('axis_0_min'), ')']),
            'axis_0_max': PythonExpression(['float(', LaunchConfiguration('axis_0_max'), ')']),

            # Gli altri assi restano con i valori di default
            'axis_1_min': -1.0,  'axis_1_max': 1.0,
            'axis_2_min': -1.0,  'axis_2_max': 1.0,
            'axis_3_min': -1.0,  'axis_3_max': 1.0,
            'axis_4_min': -1.0,  'axis_4_max': 1.0,
            'axis_5_min': -1.0,  'axis_5_max': 1.0,
        }]
    )

    # ────── 4. LaunchDescription finale ──────
    return LaunchDescription(launch_args + [joy_node, ds4_node])





