#!/usr/bin/env python3


import math
import time

from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile

from wc_example.wc_patrol_server.wc_path \
    import wcPath
from wc_msgs.action import Patrol


class wcPatrolServer(Node):

    def __init__(self):
        super().__init__('wc_patrol_server')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.goal = Patrol.Goal()

        """************************************************************
        ** Initialise ROS publishers and servers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise servers
        self.action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info("wc patrol action server has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        radius = self.goal.radius  # unit: m
        speed = 0.5  # unit: m/s

        feedback_msg = Patrol.Feedback()
        total_driving_time = 2 * math.pi * radius / speed
        feedback_msg.left_time = total_driving_time
        last_time = self.get_clock().now()

        # Start executing an action
        while (feedback_msg.left_time > 0):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Patrol.Result()

            curr_time = self.get_clock().now()
            duration = Duration()
            duration = (curr_time - last_time).nanoseconds / 1e9  # unit: s

            feedback_msg.left_time = total_driving_time - duration
            self.get_logger().info(
                'Time left until the robot stops: {0}'.format(feedback_msg.left_time))
            goal_handle.publish_feedback(feedback_msg)

            # Give vel_cmd to wc
            twist = Twist()
            twist = wcPath.drive_circle(radius, speed)
            self.cmd_vel_pub.publish(twist)

            # Process rate
            time.sleep(0.010)  # unit: s

        # When the action is completed
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        goal_handle.succeed()
        result = Patrol.Result()
        result.success = True
        self.get_logger().info('Returning result: {0}'.format(result.success))

        return result