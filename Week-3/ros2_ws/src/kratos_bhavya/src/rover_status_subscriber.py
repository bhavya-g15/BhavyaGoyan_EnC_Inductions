#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool

class RoverStatusSubscriber(Node):
    def __init__(self):
        super().__init__('rover_status_subscriber')
        
        # Subscribe to all 3 topics
        self.battery_sub = self.create_subscription(Float32, '/battery_level', self.battery_callback, 10)
        self.mode_sub = self.create_subscription(String, '/rover_mode', self.mode_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        
        # Store latest values
        self.battery = 0.0
        self.mode = "UNKNOWN"
        self.emergency = False
        
    def battery_callback(self, msg):
        self.battery = msg.data
        self.print_status()
        
    def mode_callback(self, msg):
        self.mode = msg.data
        self.print_status()
        
    def emergency_callback(self, msg):
        self.emergency = msg.data
        self.print_status()
        
    def print_status(self):
        self.get_logger().info(
            f'Received: Battery={self.battery:.1f}%, Mode={self.mode}, E-Stop={self.emergency}'
        )

def main():
    rclpy.init()
    node = RoverStatusSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
    
