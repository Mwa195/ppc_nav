#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

class localPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner")

def main(args=None):
    rclpy.init(args=args)
    myNode = localPlannerNode()
    rclpy.spin(myNode)
    rclpy.shutdown()

if __name__ == "__main__":
    main()