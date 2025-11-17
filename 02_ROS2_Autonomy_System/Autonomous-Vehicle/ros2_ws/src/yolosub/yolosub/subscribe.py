#!/usr/bin/env python3
import rclpy


class Subscriber(Node):
    def __init__(self):
        # TODO: Implement node initialization
        # TODO: Register callback
        pass

    def listener_callback(self, msg):
        # TODO: Implemnt a callback
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
