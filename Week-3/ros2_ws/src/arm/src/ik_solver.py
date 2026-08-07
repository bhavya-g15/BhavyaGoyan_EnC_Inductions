#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self.target_callback,
            10
        )
        
        self.L1 = 0.5
        self.L2 = 0.4
        self.L3 = 0.3
        
        self.joint_names = ['base_yaw_joint', 'shoulder_joint', 'elbow_joint']
        
        self.get_logger().info('IK Solver Node Started')
        self.get_logger().info('Waiting for target position on /target_position topic')
        
    def inverse_kinematics(self, target_x, target_y, target_z):
        r = math.sqrt(target_x**2 + target_y**2)
        theta1 = math.atan2(target_y, target_x)
        d = math.sqrt(r**2 + (target_z - self.L1)**2)
        
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        
        if d > max_reach:
            self.get_logger().warn(f'Target unreachable. Distance {d:.3f} > max reach {max_reach:.3f}')
            return None
        
        if d < min_reach:
            self.get_logger().warn(f'Target too close. Distance {d:.3f} < min reach {min_reach:.3f}')
            return None
        
        cos_theta3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        theta3 = math.acos(max(-1.0, min(1.0, cos_theta3)))
        
        alpha = math.atan2(target_z - self.L1, r)
        beta = math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))
        theta2 = alpha - beta
        
        return [theta1, theta2, theta3]
    
    def target_callback(self, msg):
        target = [msg.x, msg.y, msg.z]
        self.get_logger().info(f'Target received: x={target[0]:.3f}, y={target[1]:.3f}, z={target[2]:.3f}')
        
        angles = self.inverse_kinematics(msg.x, msg.y, msg.z)
        
        if angles is not None:
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = 'base_link'
            joint_msg.name = self.joint_names
            joint_msg.position = angles
            
            self.publisher.publish(joint_msg)
            
            self.get_logger().info(
                f'IK Solution: '
                f'base_yaw={angles[0]:.3f}, '
                f'shoulder={angles[1]:.3f}, '
                f'elbow={angles[2]:.3f}'
            )

def main():
    rclpy.init()
    node = IKSolver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
