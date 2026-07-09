#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class DoubleAckermannController(Node):
    def __init__(self):
        super().__init__('double_ackermann_controller')
        
        self.wheelbase = 0.4
        self.track_width = 0.6
        self.wheel_radius = 0.12
        self.max_steering = math.pi / 2
        
        self.steering_pub = self.create_publisher(
            Float64MultiArray, 
            '/steering_controller/commands', 
            10
        )
        self.drive_pub = self.create_publisher(
            Float64MultiArray, 
            '/drive_controller/commands', 
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        self.get_logger().info('Double Ackermann Controller started!')
    
    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        if abs(v) < 0.001 and abs(omega) < 0.001:
            self.stop_rover()
            return
        
        if abs(omega) < 0.001:
            fl_steer = 0.0
            fr_steer = 0.0
            rl_steer = 0.0
            rr_steer = 0.0
            
            wheel_speed = v / self.wheel_radius
            fl_speed = wheel_speed
            fr_speed = wheel_speed
            rl_speed = wheel_speed
            rr_speed = wheel_speed
        
        else:
            R = v / omega
            T_half = self.track_width / 2.0
            L = self.wheelbase
            
            fl_steer = math.atan2(L, (R - T_half))
            fr_steer = math.atan2(L, (R + T_half))
            rl_steer = math.atan2(-L, (R - T_half))
            rr_steer = math.atan2(-L, (R + T_half))
            
            fl_steer = max(-self.max_steering, min(self.max_steering, fl_steer))
            fr_steer = max(-self.max_steering, min(self.max_steering, fr_steer))
            rl_steer = max(-self.max_steering, min(self.max_steering, rl_steer))
            rr_steer = max(-self.max_steering, min(self.max_steering, rr_steer))
            
            fl_distance = math.sqrt((R - T_half)**2 + L**2)
            fr_distance = math.sqrt((R + T_half)**2 + L**2)
            rl_distance = math.sqrt((R - T_half)**2 + L**2)
            rr_distance = math.sqrt((R + T_half)**2 + L**2)
            
            fl_linear_v = omega * fl_distance
            fr_linear_v = omega * fr_distance
            rl_linear_v = omega * rl_distance
            rr_linear_v = omega * rr_distance
            
            fl_speed = fl_linear_v / self.wheel_radius
            fr_speed = fr_linear_v / self.wheel_radius
            rl_speed = rl_linear_v / self.wheel_radius
            rr_speed = rr_linear_v / self.wheel_radius
            
            if v < 0:
                fl_speed = -fl_speed
                fr_speed = -fr_speed
                rl_speed = -rl_speed
                rr_speed = -rr_speed
        
        steering_msg = Float64MultiArray()
        steering_msg.data = [fl_steer, fr_steer, rl_steer, rr_steer]
        self.steering_pub.publish(steering_msg)
        
        drive_msg = Float64MultiArray()
        drive_msg.data = [fl_speed, fr_speed, rl_speed, rr_speed]
        self.drive_pub.publish(drive_msg)
    
    def stop_rover(self):
        steering_msg = Float64MultiArray()
        steering_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.steering_pub.publish(steering_msg)
        
        drive_msg = Float64MultiArray()
        drive_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DoubleAckermannController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
