#! /usr/bin/env python3
# Import necessary libs, msgs, services and actions
import rclpy
import rclpy.publisher
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from my_interfaces_pkg.msg import StartMissionMsg
from my_interfaces_pkg.srv import CreatePlan

class behaviourNode(Node):
    def __init__(self):
        super().__init__("behaviour_node") # Initialize node
        # Create a Subscriber to get the mission name
        self.sub = self.create_subscription(StartMissionMsg, "/mission", self.missionCallbackFn, 1)
        # Create a Publisher to publish the status
        self.pub = self.create_publisher(String, "/state", 1)
        # Create a Client to request the plan
        self.client = self.create_client(CreatePlan, "/create_plan")
        # Create another Publisher to publish the plan
        self.pubPlan = self.create_publisher(Path, "/global_plan", 1)

        # Initial state
        self.state = String()
        self.state.data = "idle"

    # Execute whenever a new mission arrives
    def missionCallbackFn(self, mission:StartMissionMsg):
        # Get mission data
        self.name = mission.mission_name
        self.target_pose = mission.target_pose
        self.get_logger().info(f"Current Mission: {self.name}")

        if self.name == "Stop":
            self.state.data = "idle"
            self.pub.publish(self.state) # Change state to idle and publish
        elif self.name == "GoTo":
            self.gotoFn()

    # Execute when GoTo mission is received
    def gotoFn(self):
        self.state.data = "create_path"
        self.pub.publish(self.state) # Change state to create_path and publish
        self.get_logger().info("Requesting a Path")

        # Wait for Server
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        # Request Plan
        self.requestPlan()

    def requestPlan(self):
        requestPlanMsg = CreatePlan.Request()
        requestPlanMsg.target_pose = self.target_pose
        self.future = self.client.call_async(requestPlanMsg)  # Request plan with received Pose
        self.get_logger().info("Sent request to /create_plan, waiting for response...")
        self.timer = self.create_timer(0.1, self.timer_callback)  # Keep a reference to the timer

    def timer_callback(self):
        if self.future.done():
            try:
                self.response = self.future.result()  # Get Response
                if self.response is not None:
                    self.get_logger().info(f"Received path with {self.response.path} poses")
                    self.get_logger().info("Publishing Path to global_plan")
                    self.pubPlan.publish(self.response.path)  # Publish the plan
                    self.state.data = "navigate"
                    self.pub.publish(self.state)  # Change state to navigate and publish
                else:
                    self.get_logger().error("Service call failed: No result received")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {str(e)}")
            finally:
                self.timer.cancel()  # Cancel the timer after processing the result
        

def main(args=None):
    rclpy.init(args=args) # Initialize
    myNode = behaviourNode() # Create Node
    rclpy.spin(myNode) # Spin the Node
    rclpy.shutdown() # Shutdown on exit

if __name__ == "__main__":
    main()
