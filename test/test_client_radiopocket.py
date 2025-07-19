#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from joy_manager.srv import SwitchCommand

class SetBoolServer(Node):
    def __init__(self, service_name):
        super().__init__('set_bool_server_' + service_name.replace('/', '_'))
        self.srv = self.create_service(SetBool, service_name, self.callback)

    def callback(self, request, response):
        self.get_logger().info(f'Received request: {request.data}')
        response.success = True
        response.message = 'Service called successfully'
        return response

class SwitchCommandServer(Node):
    def __init__(self, service_name):
        super().__init__('switch_command_server_' + service_name.replace('/', '_'))
        self.srv = self.create_service(SwitchCommand, service_name, self.callback)

    def callback(self, request, response):
        self.get_logger().info(f'Received switch request: step={request.step}, label={request.label}')
        response.success = True
        response.message = f'Switch set to {request.label} (step {request.step})'
        return response

def main(args=None):
    rclpy.init(args=args)

    # SetBool services for buttons
    button_service_names = [
        'joystick/SA',
        'joystick/SD',
        'joystick/SE',
        'joystick/SAB'
    ]

    # SwitchCommand services for switches
    switch_service_names = [
        'joystick/switch_SC',
        'joystick/switch_SB'
    ]

    nodes = []
    
    # Create SetBool servers for buttons
    for name in button_service_names:
        nodes.append(SetBoolServer(name))
    
    # Create SwitchCommand servers for switches
    for name in switch_service_names:
        nodes.append(SwitchCommandServer(name))

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    executor.spin()

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
