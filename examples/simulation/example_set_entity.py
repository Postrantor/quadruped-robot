#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import Vector3, Quaternion


class SetJointState(Node):
    def __init__(self):
        super().__init__('set_joint_state_client')
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = SetEntityState.Request()

    def set_joint_state(self, entity_name, position, orientation, linear, angular, reference_frame):
        self.request.state.name = entity_name

        self.request.state.pose.position.x = position[0]
        self.request.state.pose.position.y = position[1]
        self.request.state.pose.position.z = position[2]

        self.request.state.pose.orientation.x = orientation[0]
        self.request.state.pose.orientation.y = orientation[1]
        self.request.state.pose.orientation.z = orientation[2]
        self.request.state.pose.orientation.w = orientation[3]

        self.request.state.twist.linear = Vector3(x=linear[0], y=linear[1], z=linear[2])
        self.request.state.twist.angular = Vector3(x=angular[0], y=angular[1], z=angular[2])

        self.request.state.reference_frame = reference_frame

        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    client = SetJointState()

    # set entity state
    entity_name = 'robot_a1'
    position = [
        0.0,
        0.0,
        0.0,
    ]
    orientation = [
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    linear = [
        0.0,
        0.0,
        0.0,
    ]
    angular = [
        0.0,
        0.0,
        0.0,
    ]
    reference_frame = 'ground_plane'

    response = client.set_joint_state(entity_name, position, orientation, linear, angular, reference_frame)
    if response.success:
        client.get_logger().info('Joint state set successfully')
    else:
        client.get_logger().info('Failed to set joint state')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
