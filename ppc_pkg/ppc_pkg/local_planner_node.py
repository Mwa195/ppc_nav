#! /usr/bin/env python3
import rclpy
import math
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
        self.server = ActionServer(self, Navigate, "/navigate", self.actionCallback)
        self.sub = self.create_subscription(Odometry, "/odom", self.odomCallbackFn, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.moveMsg = Twist()
        self.moveMsg.angular.z = 0.0
        self.moveMsg.linear.x = 0.0
        self.yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.distance_moved = 0.0
        self.get_logger().info("Waiting for first odometry message...")
        while self.current_x == 0.0 and self.current_y == 0.0:
            rclpy.spin_once(self)
        self.get_logger().info("Message Received, Node Initialized")


    def actionCallback(self,goal_handle: ServerGoalHandle):
        goal_path = goal_handle.request.path
        self.get_logger().info(f"Path accquired {goal_path}")
        goal_x = goal_path.poses[-1].pose.position.x
        goal_y = goal_path.poses[-1].pose.position.y
        self.start_x = self.current_x
        self.start_y = self.current_y
        theta_goal =  math.atan2(goal_y, goal_x) # in rad
        goal_distance = math.sqrt(math.pow(goal_x, 2) + math.pow(goal_y, 2))
        self.get_logger().info(f"Goal Distance: {goal_distance}")

        while abs(self.yaw - theta_goal) > 0.05:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                self.stopRobot()
                goal_handle.canceled()
                return Navigate.Result(success=False)
            
            if self.yaw < theta_goal:
                self.moveMsg.angular.z = 0.1 * (theta_goal - self.yaw)
            else:
                self.moveMsg.angular.z = 0.1 * (theta_goal - self.yaw)
            self.pub.publish(self.moveMsg)
            rclpy.spin_once(self)
        self.get_logger().info("Exited while 1")

        self.moveMsg.angular.z = 0.0
        self.pub.publish(self.moveMsg)

            # self.moveMsg.linear.x = 2
        while goal_distance - self.distance_moved > 0.1:
            self.moveMsg.linear.x = 0.2 * (goal_distance - self.distance_moved)
            self.pub.publish(self.moveMsg)
            self.get_logger().info(f"Remaining Distance: {goal_distance - self.distance_moved}")
            feedbackMsg = Navigate.Feedback()
            feedbackMsg.remaining_distance = int(goal_distance - self.distance_moved)
            goal_handle.publish_feedback(feedbackMsg)
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                self.stopRobot()
                goal_handle.canceled()
                return Navigate.Result(success=False)
            # else:
            rclpy.spin_once(self)
        
        self.moveMsg.linear.x = 0.0
        self.pub.publish(self.moveMsg)
        self.get_logger().info("Exited while 2")
        self.stopRobot()
        goal_handle.succeed()
        self.get_logger().info("Goal Handle Succeed")
        return Navigate.Result(success=True)

    def stopRobot(self):
        self.moveMsg.linear.x = 0.0
        self.moveMsg.angular.z = 0.0
        self.pub.publish(self.moveMsg)

    def odomCallbackFn(self, odomMsg: Odometry):
        quatMsg = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, 
                   odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z]
        _, _, self.yaw = quat2euler(quatMsg)

        self.current_x = odomMsg.pose.pose.position.x
        self.current_y = odomMsg.pose.pose.position.y
        self.distance_moved = math.sqrt(math.pow(self.current_x - self.start_x, 2) + math.pow(self.current_y - self.start_y, 2))

def main(args=None):
    rclpy.init(args=args)
    myNode = localPlannerNode()
    rclpy.spin(myNode)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


