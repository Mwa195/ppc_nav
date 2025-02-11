#! /usr/bin/env python3
# Import necessary libs, msgs, services and actions
import rclpy
import rclpy.publisher
from my_interfaces_pkg.msg import StartMissionMsg
from my_interfaces_pkg.srv import StartMission
from rclpy.node import Node

class missionNode(Node):
    def __init__(self):
        super().__init__("mission_node") # Initialize Node
        self.server = self.create_service(StartMission, "/start_mission", self.startMissionCallback) # Create Server
        self.pub = self.create_publisher(StartMissionMsg, "/mission", 1) # Create Publisher (Topic: /mission)
        self.get_logger().info("Server Initialized")

    def startMissionCallback(self, request, response):
        name = request.mission_name # Get name to check if response is accepted or not
        msg = StartMissionMsg()
        if name == "GoTo":
            response.acceptance = True # Accepted
            # Get Pose
            pose = request.target_pose
            msg.mission_name = name
            msg.target_pose = pose
            self.get_logger().info(f"Recognized Mission: GoTo, Coordinates: {pose}")
            self.pub.publish(msg) # Publish
        elif name == "Stop":
            response.acceptance = True # Accepted
            msg.mission_name = name
            self.get_logger().info(f"Recognized Mission: Stop, Stopping")
            self.pub.publish(msg) # Publish
        else:
            response.acceptance = False # Rejected
            self.get_logger().info("Unrecognized Mission")

        return response # Respond to Client

def main(args=None):
    rclpy.init(args=args) # Initialize
    myNode = missionNode() # Create Node
    rclpy.spin(myNode) # Spin the Node
    rclpy.shutdown() # Shutdown on exit

if __name__ == "__main__":
    main()