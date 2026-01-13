#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class BumperbotTrajectory(Node):
    def __init__(self):
        super().__init__('bumperbot_trajectory')
        
        # Declare and get the odom_topic parameter
        self.declare_parameter('odom_topic', '/bumperbot_controller/odom')
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        
        # Create subscriber to odometry topic
        self.odom_sub_ = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        # Create publisher for trajectory topic
        self.trajectory_pub_ = self.create_publisher(
            Path,
            '/bumperbot_controller/trajectory',
            10
        )
        
        # Initialize the path message
        self.path_ = Path()
        self.path_.header.frame_id = 'odom'
        
        self.get_logger().info(f'Bumperbot Trajectory Node started. Subscribing to {odom_topic}')
        
    def odom_callback(self, msg):
        """
        Callback function that processes odometry messages and builds the trajectory
        """
        # Create a PoseStamped from the odometry pose
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # Append the pose to the path
        self.path_.poses.append(pose_stamped)
        
        # Update the path header timestamp
        self.path_.header.stamp = msg.header.stamp
        
        # Publish the trajectory
        self.trajectory_pub_.publish(self.path_)


def main(args=None):
    rclpy.init(args=args)
    node = BumperbotTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
