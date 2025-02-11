#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces_pkg.srv import CreatePlan
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

class globalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner") # Modify
        self.server = self.create_service(CreatePlan, "/create_plan", self.createPlanCallback)
        self.get_logger().info("Server Initialized")

    def createPlanCallback(self, request, response):
        # Goal Poes:
        goal_pose = request.target_pose
        self.get_logger().info(f"Request received for a plan to {goal_pose}")

        # Planned Path:
        planned_path = Path()
        planned_path.header.stamp = self.get_clock().now().to_msg()
        planned_path.header.frame_id = "map"

        # Start Pose:
        start_pose = Pose()
        # Always start from origin (the Robot)
        start_pose.position.x = 0.0
        start_pose.position.y = 0.0
        start_pose.position.z = 0.0
        start_pose.orientation.w = 1.0
        # Stamp the pose in order to be appended to the path
        start_pose_stamped = PoseStamped()
        start_pose_stamped.header = planned_path.header
        start_pose_stamped.pose = start_pose

        # Stamp the pose in order to be appended to the path
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header = planned_path.header
        goal_pose_stamped.pose = goal_pose

        # Create the path
        planned_path.poses.append(start_pose_stamped)
        planned_path.poses.append(goal_pose_stamped)
        
        response.path = planned_path
        self.get_logger().info(f"Sending {response.path}")
        self.get_logger().info("Plan Created, Returning response!")
        return response

def main(args=None):
    rclpy.init(args=args)
    myNode = globalPlannerNode()
    rclpy.spin(myNode)
    rclpy.shutdown()

if __name__ == "__main__":
    main()