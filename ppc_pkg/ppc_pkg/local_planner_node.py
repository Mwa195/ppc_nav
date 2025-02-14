#! /usr/bin/env python3
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
        self.server = ActionServer(self, Navigate, "/navigate", self.actionCallback, cancel_callback= self.check_cancellation)
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
        self.goal_handle = None
        self.state = "None"
        # self._goal_lock = threading.Lock()
        self.get_logger().info("Waiting for first odometry message...")
        while self.current_x == 0.0 and self.current_y == 0.0:
            self.get_logger().info("Waiting...")
            rclpy.spin_once(self)
        self.get_logger().info("Message Received, Node Initialized")

    def actionCallback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Goal_accquired {goal_handle}")
        self.state = "None"
        self.goal_handle = goal_handle
        goal_path = self.goal_handle.request.path
        self.get_logger().info(f"Path accquired {goal_path}")
        goal_x = goal_path.poses[-1].pose.position.x
        goal_y = goal_path.poses[-1].pose.position.y
        start_x = goal_path.poses[0].pose.position.x
        start_y = goal_path.poses[0].pose.position.y
        self.start_x = self.current_x
        self.start_y = self.current_y
        theta_goal =  atan2(goal_y, goal_x) # in rad
        goal_distance = sqrt(pow(goal_x - start_x, 2) + pow(goal_y - start_y, 2)) ################## check this later
        self.get_logger().info(f"Goal Distance: {goal_distance}")

        while abs(self.yaw - theta_goal) > 0.005:
            rclpy.spin_once(self)
            if self.state == "Canceled":
                break
            self.moveMsg.linear.x = 0.0
            # self.get_logger().info("in motherfucking while 1")
            if self.yaw <= theta_goal:
                self.moveMsg.angular.z = 0.1
            else:
                self.moveMsg.angular.z = -0.1
            self.pub.publish(self.moveMsg)
            self.get_logger().info(f"Goal Angle: {theta_goal}")
            self.get_logger().info(f"My Angle: {self.yaw}")
            self.get_logger().info(f"Remaining Angle: {theta_goal - self.yaw}")
        
        self.get_logger().info(f"OUTSIDEEE Goal Angle: {theta_goal}")
        self.get_logger().info(f"OUTSIDEEE My Angle: {self.yaw}")
        self.get_logger().info(f"OUTSIDEEE Remaining Angle: {theta_goal - self.yaw}")
            
        self.moveMsg.linear.x = 0.0
        self.moveMsg.angular.z = 0.0
        self.pub.publish(self.moveMsg)
        sleep(2)
        self.get_logger().info(f"OUTSIDEEE Goal Angle: {theta_goal}")
        self.get_logger().info(f"OUTSIDEEE My Angle: {self.yaw}")
        self.get_logger().info(f"OUTSIDEEE Remaining Angle: {theta_goal - self.yaw}")

        self.get_logger().info(f"Goal x: {goal_x}")
        self.get_logger().info(f"Goal x: {goal_y}")
        self.get_logger().info(f"Goal x: {start_x}")
        self.get_logger().info(f"Goal x: {start_x}")

        self.get_logger().info("Exited while 1")

        while goal_distance - self.distance_moved > 0.1:
            rclpy.spin_once(self)
            if self.state == "Canceled":
                break
            # self.get_logger().info("in motherfucking while 2")
            self.moveMsg.angular.z = 0.0
            self.moveMsg.linear.x = 0.4
            self.pub.publish(self.moveMsg)
            self.get_logger().info(f"Goal Distance: {goal_distance}")
            self.get_logger().info(f"Moved Distance: {self.distance_moved}")
            self.get_logger().info(f"Remaining Distance: {goal_distance - self.distance_moved}")
            feedbackMsg = Navigate.Feedback()
            feedbackMsg.remaining_distance = int(goal_distance - self.distance_moved)
            goal_handle.publish_feedback(feedbackMsg)
        
        self.get_logger().info(f"OUTSIDEEE Goal Distance: {goal_distance}")
        self.get_logger().info(f"OUTSIDEEE Moved Distance: {self.distance_moved}")
        self.get_logger().info(f"OUTSIDEEE Remaining Distance: {goal_distance - self.distance_moved}")

        self.moveMsg.angular.z = 0.0
        self.moveMsg.linear.x = 0.0
        self.pub.publish(self.moveMsg)

        self.get_logger().info(f"OUTSIDEEE Goal Distance: {goal_distance}")
        self.get_logger().info(f"OUTSIDEEE Moved Distance: {self.distance_moved}")
        self.get_logger().info(f"OUTSIDEEE Remaining Distance: {goal_distance - self.distance_moved}")
        self.get_logger().info("Exited while 2")

        if self.goal_handle is not None:  # Check if goal_handle is valid
            if self.state == "Canceled":
                self.goal_handle.abort()
                self.goal_handle = None  # Explicitly reset to None
                return Navigate.Result(success=False)
            else:
                self.goal_handle.succeed()
                self.goal_handle = None  # Explicitly reset to None
                return Navigate.Result(success=True)
        else:
            self.get_logger().warn("Goal handle is already destroyed or invalid.")
            return Navigate.Result(success=False)

    def check_cancellation(self, goal_handle):
        self.get_logger().info("Goal canceled during execution")
        self.moveMsg.linear.x = 0.0
        self.moveMsg.angular.z = 0.0
        self.pub.publish(self.moveMsg)
        self.state = "Canceled"

    def odomCallbackFn(self, odomMsg: Odometry):
        quatMsg = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, 
                   odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z]
        _, _, self.yaw = quat2euler(quatMsg)

        self.current_x = odomMsg.pose.pose.position.x
        self.current_y = odomMsg.pose.pose.position.y
        self.distance_moved = sqrt(pow(self.current_x - self.start_x, 2) + pow(self.current_y - self.start_y, 2))

def main(args=None):
    rclpy.init(args=args)
    myNode = localPlannerNode()
    try:
        while rclpy.ok():  # Continue until ROS 2 is shut down
            rclpy.spin_once(myNode, timeout_sec=0.1)  # Process one event
            # Add other non-blocking tasks here if needed
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        myNode.destroy_node()
        rclpy.shutdown()
    # executor = MultiThreadedExecutor()
    # executor.add_node(myNode)

    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     myNode.destroy_node()
    #     rclpy.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


# #! /usr/bin/env python3
# import rclpy
# import math
# from rclpy.node import Node
# from rclpy.action import ActionServer
# from rclpy.action.server import ServerGoalHandle
# from nav_msgs.msg import Path
# from nav_msgs.msg import Odometry
# from my_interfaces_pkg.action import Navigate
# from geometry_msgs.msg import Twist
# from transforms3d.euler import quat2euler

# class localPlannerNode(Node):
#     def __init__(self):
#         super().__init__("local_planner")
#         self.server = ActionServer(self, Navigate, "/navigate", self.actionCallback)
#         self.sub = self.create_subscription(Odometry, "/odom", self.odomCallbackFn, 10)
#         self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
#         self.moveMsg = Twist()
#         self.moveMsg.angular.z = 0.0
#         self.moveMsg.linear.x = 0.0
#         self.yaw = 0.0
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.start_x = 0.0
#         self.start_y = 0.0
#         self.distance_moved = 0.0
#         self.goal_handle = None
#         self.timer = None
#         self.goal_x = 0.0
#         self.goal_y = 0.0
#         self.theta_goal = 0.0
#         self.goal_distance = 0.0
#         self.state = "idle"  # Tracks the current state of the robot
#         self.get_logger().info("Waiting for first odometry message...")
#         while self.current_x == 0.0 and self.current_y == 0.0:
#             rclpy.spin_once(self)
#         self.get_logger().info("Message Received, Node Initialized")

#     def actionCallback(self, goal_handle: ServerGoalHandle):
#         self.goal_handle = goal_handle

#         # Check for cancellation at the start
#         if goal_handle.is_cancel_requested:
#             self.get_logger().info("Goal canceled at the start")
#             self.stopRobot()
#             goal_handle.canceled()
#             return Navigate.Result(success=False)

#         goal_path = goal_handle.request.path
#         self.get_logger().info(f"Path acquired {goal_path}")
#         self.goal_x = goal_path.poses[-1].pose.position.x
#         self.goal_y = goal_path.poses[-1].pose.position.y
#         self.start_x = self.current_x
#         self.start_y = self.current_y
#         self.theta_goal = math.atan2(self.goal_y, self.goal_x)  # in rad
#         self.goal_distance = math.sqrt(math.pow(self.goal_x, 2) + math.pow(self.goal_y, 2))
#         self.get_logger().info(f"Goal Distance: {self.goal_distance}")

#         # Start the timer for non-blocking execution
#         self.state = "rotating"
#         self.timer = self.create_timer(0.1, self.timerCallback)  # 10 Hz timer
#         return

#     def timerCallback(self):
#         if self.goal_handle is None:
#             return

#         # Check for cancellation
#         if self.goal_handle.is_cancel_requested:
#             self.get_logger().info("Goal canceled during execution")
#             self.stopRobot()
#             self.goal_handle.canceled()
#             self.timer.cancel()
#             self.state = "idle"
#             return

#         # State machine for motion control
#         if self.state == "rotating":
#             self.handleRotation()
#         elif self.state == "moving":
#             self.handleLinearMovement()
#         elif self.state == "idle":
#             pass

#     def handleRotation(self):
#         if abs(self.yaw - self.theta_goal) > 0.01:
#             if self.yaw < self.theta_goal:
#                 self.moveMsg.angular.z = 0.1 * (self.theta_goal - self.yaw)
#             else:
#                 self.moveMsg.angular.z = 0.1 * (self.theta_goal - self.yaw)
#             self.pub.publish(self.moveMsg)
#         else:
#             self.moveMsg.angular.z = 0.0
#             self.pub.publish(self.moveMsg)
#             self.get_logger().info("Rotation complete")
#             self.state = "moving"  # Transition to linear movement

#     def handleLinearMovement(self):
#         if self.goal_distance - self.distance_moved > 0.1:
#             self.moveMsg.linear.x = 0.2 * (self.goal_distance - self.distance_moved)
#             self.pub.publish(self.moveMsg)
#             self.get_logger().info(f"Remaining Distance: {self.goal_distance - self.distance_moved}")
#             feedbackMsg = Navigate.Feedback()
#             feedbackMsg.remaining_distance = int(self.goal_distance - self.distance_moved)
#             self.goal_handle.publish_feedback(feedbackMsg)
#         else:
#             self.moveMsg.linear.x = 0.0
#             self.pub.publish(self.moveMsg)
#             self.get_logger().info("Linear movement complete")
#             self.timer.cancel()
#             self.state = "idle"
#             self.goal_handle.succeed()
#             self.get_logger().info("Goal Handle Succeed")
#             return Navigate.Result(success=True)

#     def stopRobot(self):
#         self.moveMsg.linear.x = 0.0
#         self.moveMsg.angular.z = 0.0
#         self.pub.publish(self.moveMsg)

#     def odomCallbackFn(self, odomMsg: Odometry):
#         quatMsg = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x,
#                    odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z]
#         _, _, self.yaw = quat2euler(quatMsg)

#         self.current_x = odomMsg.pose.pose.position.x
#         self.current_y = odomMsg.pose.pose.position.y
#         self.distance_moved = math.sqrt(math.pow(self.current_x - self.start_x, 2) + math.pow(self.current_y - self.start_y, 2))

# def main(args=None):
#     rclpy.init(args=args)
#     myNode = localPlannerNode()
#     rclpy.spin(myNode)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()