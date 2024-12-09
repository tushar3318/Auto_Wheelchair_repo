#!/usr/bin/env python3


import rclpy

from wc_example.wc_obstacle_detection.wc_obstacle_detection \
    import wcObstacleDetection


def main(args=None):
    rclpy.init(args=args)
    wc_obstacle_detection = wcObstacleDetection()
    rclpy.spin(wc_obstacle_detection)

    wc_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()