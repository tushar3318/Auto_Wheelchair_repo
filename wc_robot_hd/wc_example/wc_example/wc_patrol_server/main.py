#!/usr/bin/env python3

import rclpy

from wc_example.wc_patrol_server.wc_patrol_server \
    import wcPatrolServer


def main(args=None):
    rclpy.init(args=args)
    wc_patrol_server = wcPatrolServer()
    rclpy.spin(wc_patrol_server)

    wc_patrol_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()