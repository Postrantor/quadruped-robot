#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
import sys


class ControllerActivator(Node):
    def __init__(self, controllers):
        super().__init__('controller_activator')
        self.controllers = controllers
        self.cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /controller_manager/switch_controller...')

        self.req = SwitchController.Request()
        self.req.activate_controllers = self.controllers
        self.req.stop_controllers = []
        self.req.strictness = 2  # Best effort

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Controllers activated successfully.')
        else:
            self.get_logger().error('Failed to activate controllers.')


def main(args=None):
    rclpy.init(args=args)

    # 从参数中获取控制器名称列表
    if len(sys.argv) < 2:
        print("Usage: activate_controllers.py <controller_name> [<controller_name> ...]")
        return

    controller_names = sys.argv[1:]

    controller_activator = ControllerActivator(controller_names)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
