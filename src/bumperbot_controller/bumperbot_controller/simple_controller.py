#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np



class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        
        self.get_logger().info(f"Wheel Radius: {self.wheel_radius_}, Wheel Separation: {self.wheel_separation_}")
        
        self.wheel_command_publisher_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(Twist, "bumperbot_controller/cmd_vel", self.cmd_velCallback, 10)
        
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2,], 
                                          [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        
        self.get_logger().info(f"Speed Conversion Matrix:\n{self.speed_conversion_}")
        
    def cmd_velCallback(self, msg):
        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]
        self.wheel_command_publisher_.publish(wheel_speed_msg)
        
def main(args=None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()