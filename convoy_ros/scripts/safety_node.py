#!/usr/bin/env python3

import rclpy

from convoy_ros import safety


def main(args=None):
    rclpy.init(args=args)
    safety_node = safety.Safety()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
