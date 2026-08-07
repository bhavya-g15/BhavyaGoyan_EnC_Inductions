#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class FKSolver(Node):
    def __init__(self):
        super().__init__('fk_solver')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.L1 = 0.5
        self.L2 = 0.4
        self.L3 = 0.3
        
        self.get_logger().info('FK Solver Node Started')
        self.compute_workspace()
        
    def forward_kinematics(self, theta1, theta2, theta3):
        x = (self.L2 * math.cos(theta2) + self.L3 * math.cos(theta2 + theta3)) * math.cos(theta1)
        y = (self.L2 * math.cos(theta2) + self.L3 * math.cos(theta2 + theta3)) * math.sin(theta1)
        z = self.L1 + self.L2 * math.sin(theta2) + self.L3 * math.sin(theta2 + theta3)
        return [x, y, z]
    
    def compute_workspace(self):
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        
        self.get_logger().info(f'Maximum reach: {max_reach:.3f} m')
        self.get_logger().info(f'Minimum reach: {min_reach:.3f} m')
        self.get_logger().info(f'Working range: {min_reach:.3f} to {max_reach:.3f} m')
        
        return max_reach, min_reach
    
    def joint_state_callback(self, msg):
        try:
            base_idx = msg.name.index('base_yaw_joint')
            shoulder_idx = msg.name.index('shoulder_joint')
            elbow_idx = msg.name.index('elbow_joint')
            
            theta1 = msg.position[base_idx]
            theta2 = msg.position[shoulder_idx]
            theta3 = msg.position[elbow_idx]
            
            ee_position = self.forward_kinematics(theta1, theta2, theta3)
            
            self.get_logger().info(
                f'End-effector position: '
                f'x={ee_position[0]:.3f}, '
                f'y={ee_position[1]:.3f}, '
                f'z={ee_position[2]:.3f}'
            )
            
        except ValueError:
            self.get_logger().warn('Joint not found in message')

def main():
    rclpy.init()
    node = FKSolver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
