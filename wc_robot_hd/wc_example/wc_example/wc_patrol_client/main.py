#!/usr/bin/env python3


import rclpy

from wc_example.wc_patrol_client.wc_patrol_client \
    import wcPatrolClient


def main(args=None):
    rclpy.init(args=args)
    wc_patrol_client = wcPatrolClient()
    rclpy.spin(wc_patrol_client)


if __name__ == '__main__':
    main()