#! /usr/bin/env python3
import rclpy
import rclpy.publisher
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from my_interfaces_pkg.msg import StartMissionMsg
from my_interfaces_pkg.srv import CreatePlan

class behaviourNode(Node):
    def __init__(self):
        super().__init__("behaviour_node")
        self.sub = self.create_subscription(StartMissionMsg, "/mission", self.missionCallbackFn, 1)
        self.pub = self.create_publisher(String, "/state", 1)
        self.pubPlan = self.create_publisher(Path, "/global_plan", 1)
        self.client = self.create_client(CreatePlan, "create_plan") 
        self.state = String()
        self.state.data = "idle"

    def missionCallbackFn(self, mission:StartMissionMsg):
        self.name = mission.mission_name
        self.target_pose = mission.target_pose
        self.get_logger().info(f"Current Mission: {self.name}")

        if self.name == "Stop":
            self.state.data = "idle"
            self.pub.publish(self.state)
        elif self.name == "GoTo":
            self.gotoFn()

    def gotoFn(self):
        self.state.data = "create_path"
        self.pub.publish(self.state)
        self.get_logger().info("Requesting a Path")

        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')
    #     self.requestPlan()


    # def requestPlan(self):
    #     requestPlanMsg = CreatePlan.Request()
    #     requestPlanMsg.target_pose = self.target_pose
    #     future = self.client.call_async(requestPlanMsg)
    #     try:
    #         self.response = future.result()
    #         self.get_logger().info(f"Received path with {self.response.path}")
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {str(e)}")
        
    #     self.get_logger().info("Publishing Path to global_plan")
    #     self.pubPlan.publish(self.response)
    #     self.state = "navigate"
    #     self.pub.publish(self.state)

        

def main(args=None):
    rclpy.init(args=args)
    myNode = behaviourNode()
    rclpy.spin(myNode)
    rclpy.shutdown()

if __name__ == "__main__":
    main()