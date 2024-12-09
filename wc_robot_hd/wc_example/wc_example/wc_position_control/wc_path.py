#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist


class wcPath():

    def turn(angle, angular_velocity, step):
        twist = Twist()

        if math.fabs(angle) > 0.01:  # 0.01 is small enough value
            if angle >= math.pi:
                twist.angular.z = -angular_velocity
            elif math.pi > angle and angle >= 0:
                twist.angular.z = angular_velocity
            elif 0 > angle and angle >= -math.pi:
                twist.angular.z = -angular_velocity
            elif angle > -math.pi:
                twist.angular.z = angular_velocity
        else:
            step += 1

        return twist, step

    def go_straight(distance, linear_velocity, step):
        twist = Twist()

        if distance > 0.01:  # 0.01 is small enough value
            twist.linear.x = linear_velocity
        else:
            step += 1

        return twist, step