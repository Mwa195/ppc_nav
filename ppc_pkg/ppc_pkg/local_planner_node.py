#! /usr/bin/env python3
# Import necessary libs, msgs, services and actions
import rclpy
from math import atan2, sqrt, pow
from time import sleep
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from my_interfaces_pkg.action import Navigate
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler

class localPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner")
        # Create Navigate Action Server
        self.server = ActionServer(self, Navigate, "/navigate", self.actionCallback, cancel_callback= self.check_cancellation)
        # Create a Subscriber to get Odometry data about the rover
        self.sub = self.create_subscription(Odometry, "/odom", self.odomCallbackFn, 10)
        # Create a Publisher to publish Twist messages to the rover
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Initialize required variables
        self.moveMsg = Twist()
        self.moveMsg.angular.z = 0.0
        self.moveMsg.linear.x = 0.0
        self.yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.distance_moved = 0.0
        self.goal_handle = None
        self.state = "None"
        self.get_logger().info("Waiting for first odometry message...")
        # Wait for /odom to publish messages (Gazebo to open)
        while self.current_x == 0.0 and self.current_y == 0.0:
            self.get_logger().info("Waiting...")
            rclpy.spin_once(self)
        self.get_logger().info("Message Received, Node Initialized")

    def actionCallback(self, goal_handle: ServerGoalHandle):
        self.state = "None" # Set state to None to execute the code
        self.goal_handle = goal_handle
        goal_path = self.goal_handle.request.path # Accquire path
        self.get_logger().info(f"Path accquired {goal_path}")
        # Get data from path
        goal_x = goal_path.poses[-1].pose.position.x
        goal_y = goal_path.poses[-1].pose.position.y
        start_x = goal_path.poses[0].pose.position.x
        start_y = goal_path.poses[0].pose.position.y
        # Set starting position from current odometry data
        self.start_x = self.current_x
        self.start_y = self.current_y
        # Calculate required angle to turn to (tan(y/x)^-1)
        theta_goal =  atan2(goal_y, goal_x) # in rad
        # Calculate required distance to move (Goal Position - Current Position)
        goal_distance = sqrt(pow(goal_x - start_x, 2) + pow(goal_y - start_y, 2))
        self.get_logger().info(f"Goal Distance: {goal_distance}")

        # Rotate towards the goal
        while abs(self.yaw - theta_goal) > 0.005:
            rclpy.spin_once(self) # Keep the rest of the node running
            # Check if current goal is canceled
            if self.state == "Canceled":
                break
            self.moveMsg.linear.x = 0.0 # Make sure movement is stopped
            # Turn cw or ccw according to which is closer
            if self.yaw <= theta_goal:
                self.moveMsg.angular.z = 0.1
            else:
                self.moveMsg.angular.z = -0.1
            self.pub.publish(self.moveMsg) # Publish Twist message
            self.get_logger().info(f"Goal Angle: {theta_goal}")
            self.get_logger().info(f"My Angle: {self.yaw}")
            self.get_logger().info(f"Remaining Angle: {theta_goal - self.yaw}")
        
        # Stop the rover
        self.moveMsg.linear.x = 0.0
        self.moveMsg.angular.z = 0.0
        self.pub.publish(self.moveMsg) # Publish Twist message
        
        sleep(2) # Wait 2 seconds

        self.get_logger().info(f"Goal x: {goal_x}")
        self.get_logger().info(f"Goal y: {goal_y}")
        self.get_logger().info(f"Start x: {start_x}")
        self.get_logger().info(f"Start y: {start_x}")

        self.get_logger().info("Rotated towards the goal ✅")

        # Move towards the goal
        while goal_distance - self.distance_moved > 0.1:
            rclpy.spin_once(self) # Keep the rest of the node running
            # Check if current goal is canceled
            if self.state == "Canceled":
                break
            self.moveMsg.angular.z = 0.0 # Make sure rotation is stopped
            self.moveMsg.linear.x = 0.4
            self.pub.publish(self.moveMsg) # Publish Twist message
            self.get_logger().info(f"Goal Distance: {goal_distance}")
            self.get_logger().info(f"Moved Distance: {self.distance_moved}")
            self.get_logger().info(f"Remaining Distance: {goal_distance - self.distance_moved}")
            # Send Feedback
            feedbackMsg = Navigate.Feedback()
            feedbackMsg.remaining_distance = int(goal_distance - self.distance_moved)
            goal_handle.publish_feedback(feedbackMsg)

        # Stop the rover
        self.moveMsg.angular.z = 0.0
        self.moveMsg.linear.x = 0.0
        self.pub.publish(self.moveMsg) # Publish Twist message

        self.get_logger().info("✅ Arrived to the GOAL!!! ✅")

        # Handle returns according to goal state
        if self.goal_handle is not None:
            if self.state == "Canceled":
                self.goal_handle.abort()
                self.goal_handle = None
                return Navigate.Result(success=False)
            else:
                self.goal_handle.succeed()
                self.goal_handle = None
                return Navigate.Result(success=True)
        else:
            self.get_logger().warn("Goal handle is already destroyed or invalid.")
            return Navigate.Result(success=False)

    # Cancel Callback
    def check_cancellation(self, goal_handle):
        self.get_logger().info("Goal canceled during execution")
        # Stop the rover
        self.moveMsg.linear.x = 0.0
        self.moveMsg.angular.z = 0.0
        self.pub.publish(self.moveMsg)  # Publish Twist message
        self.state = "Canceled"

    # /odom Subscriber callback
    def odomCallbackFn(self, odomMsg: Odometry):
        quatMsg = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, 
                   odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z] # get Quaternion data
        _, _, self.yaw = quat2euler(quatMsg) # get yaw from quaternion using (quat2euler) function

        # Get current position data
        self.current_x = odomMsg.pose.pose.position.x
        self.current_y = odomMsg.pose.pose.position.y
        # Calculate distance moved towards goal
        self.distance_moved = sqrt(pow(self.current_x - self.start_x, 2) + pow(self.current_y - self.start_y, 2))

def main(args=None):
    rclpy.init(args=args) # Initialize
    myNode = localPlannerNode() # Create Node
    try:
        while rclpy.ok():
            rclpy.spin_once(myNode, timeout_sec=0.1) # Spin the Node
    except KeyboardInterrupt:
        pass
    finally:
        myNode.destroy_node()
        rclpy.shutdown() # Shutdown on exit
    rclpy.shutdown()

if __name__ == "__main__":
    main()
