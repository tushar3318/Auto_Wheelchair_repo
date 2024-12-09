#!/usr/bin/env python3

import rclpy

from wc_example.wc_position_control.wc_position_control \
    import wcPositionControl


def main(args=None):
    rclpy.init(args=args)
    wc_position_control = wcPositionControl()
    rclpy.spin(wc_position_control)

    wc_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()