#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.joint_names = [
            'base_yaw_joint',
            'shoulder_joint',
            'elbow_joint'
        ]
        
        self.joint_positions = [0.0, 0.0, 0.0]
        
        self.L1 = 0.5
        self.L2 = 0.4
        self.L3 = 0.3
        
        self.demo_targets = [
            [0.0, 0.0, 0.0],
            [0.5, 0.3, 0.2],
            [1.0, 0.5, 0.4],
            [0.2, 0.8, 0.6],
            [0.0, 1.2, 0.8],
            [0.5, 0.8, 0.9],
            [1.5, 0.5, 0.8],
            [0.0, 0.0, 0.0]
        ]
        self.target_index = 0
        self.step_counter = 0
        
        self.get_logger().info('Arm Controller Node Started')
        self.get_logger().info('Publishing to /joint_states topic')
        
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.publisher.publish(msg)
        
        self.step_counter += 1
        if self.step_counter >= 50:
            self.step_counter = 0
            self.move_to_next_target()
            
    def move_to_next_target(self):
        self.target_index = (self.target_index + 1) % len(self.demo_targets)
        target = self.demo_targets[self.target_index]
        self.joint_positions = target.copy()
        
        ee_pos = self.forward_kinematics(target[0], target[1], target[2])
        
        self.get_logger().info(
            f'Target {self.target_index}: '
            f'angles=[{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}] '
            f'end_effector=[{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]'
        )
        
    def forward_kinematics(self, theta1, theta2, theta3):
        x = (self.L2 * math.cos(theta2) + self.L3 * math.cos(theta2 + theta3)) * math.cos(theta1)
        y = (self.L2 * math.cos(theta2) + self.L3 * math.cos(theta2 + theta3)) * math.sin(theta1)
        z = self.L1 + self.L2 * math.sin(theta2) + self.L3 * math.sin(theta2 + theta3)
        return [x, y, z]
    
    def inverse_kinematics(self, target_x, target_y, target_z):
        r = math.sqrt(target_x**2 + target_y**2)
        theta1 = math.atan2(target_y, target_x)
        d = math.sqrt(r**2 + (target_z - self.L1)**2)
        
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        
        if d > max_reach or d < min_reach:
            return None
        
        cos_theta3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        theta3 = math.acos(max(-1.0, min(1.0, cos_theta3)))
        
        alpha = math.atan2(target_z - self.L1, r)
        beta = math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))
        theta2 = alpha - beta
        
        return [theta1, theta2, theta3]

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down Arm Controller...')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
