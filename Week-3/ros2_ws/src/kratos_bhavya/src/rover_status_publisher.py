#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool

class RoverStatusPublisher(Node):
    def __init__(self):
        super().__init__('rover_status_publisher')
        
        # Create 3 separate publishers
        self.battery_pub = self.create_publisher(Float32, '/battery_level', 10)
        self.mode_pub = self.create_publisher(String, '/rover_mode', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Publish at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_data)
        
        # Counter for changing values
        self.counter = 0
        
    def publish_data(self):
        # Create messages
        battery_msg = Float32()
        mode_msg = String()
        emergency_msg = Bool()
        
        # Set values (cycling through different values)
        battery_msg.data = 85.0 + (self.counter % 15)
        mode_msg.data = "AUTO" if self.counter % 2 == 0 else "MANUAL"
        emergency_msg.data = False if self.counter % 5 != 0 else True
        
        # Publish
        self.battery_pub.publish(battery_msg)
        self.mode_pub.publish(mode_msg)
        self.emergency_pub.publish(emergency_msg)
        
        # Print what we published
        self.get_logger().info(f'Published: Battery={battery_msg.data:.1f}%, Mode={mode_msg.data}, E-Stop={emergency_msg.data}')
        
        self.counter += 1

def main():
    rclpy.init()
    node = RoverStatusPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
