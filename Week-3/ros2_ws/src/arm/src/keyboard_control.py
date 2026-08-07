#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import tty
import termios
import select
import math

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.joint_names = ['base_yaw_joint', 'shoulder_joint', 'elbow_joint']
        self.joint_positions = [0.0, 0.0, 0.0]
        self.step = 0.05
        
        self.limits = [
            (-math.pi, math.pi),
            (-math.pi/2, math.pi/2),
            (-math.pi/2, math.pi/2)
        ]
        
        self.get_logger().info('Keyboard Control Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        print('\n' + '='*50)
        print('KEYBOARD CONTROL FOR ROBOTIC ARM')
        print('='*50)
        print('  q/a  : Base Yaw (increase/decrease)')
        print('  w/s  : Shoulder (increase/decrease)')
        print('  e/d  : Elbow (increase/decrease)')
        print('  r    : Reset to home position')
        print('  p    : Print current joint angles')
        print('  i    : Print end-effector position')
        print('  h    : Show this help')
        print('  ESC  : Exit')
        print('='*50)
        print(f'  Step size: {math.degrees(self.step):.1f} degrees')
        print('='*50 + '\n')
        
    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))
        
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.publisher.publish(msg)
        
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
    def is_key_available(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
        
    def forward_kinematics(self, theta1, theta2, theta3):
        L1, L2, L3 = 0.5, 0.4, 0.3
        
        x = (L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)) * math.cos(theta1)
        y = (L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)) * math.sin(theta1)
        z = L1 + L2 * math.sin(theta2) + L3 * math.sin(theta2 + theta3)
        
        return [x, y, z]
        
    def run(self):
        print('Control the arm using keyboard. Press ESC to exit.')
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            
            if self.is_key_available():
                key = self.get_key()
                
                if key == 'q':
                    self.joint_positions[0] += self.step
                elif key == 'a':
                    self.joint_positions[0] -= self.step
                elif key == 'w':
                    self.joint_positions[1] += self.step
                elif key == 's':
                    self.joint_positions[1] -= self.step
                elif key == 'e':
                    self.joint_positions[2] += self.step
                elif key == 'd':
                    self.joint_positions[2] -= self.step
                elif key == 'r':
                    self.joint_positions = [0.0, 0.0, 0.0]
                    print('Reset to home position')
                elif key == 'p':
                    print(f'\nJoint angles:')
                    print(f'  base_yaw  : {math.degrees(self.joint_positions[0]):.1f} deg')
                    print(f'  shoulder  : {math.degrees(self.joint_positions[1]):.1f} deg')
                    print(f'  elbow     : {math.degrees(self.joint_positions[2]):.1f} deg')
                elif key == 'i':
                    ee = self.forward_kinematics(
                        self.joint_positions[0],
                        self.joint_positions[1],
                        self.joint_positions[2]
                    )
                    print(f'\nEnd-effector position:')
                    print(f'  x: {ee[0]:.3f} m')
                    print(f'  y: {ee[1]:.3f} m')
                    print(f'  z: {ee[2]:.3f} m')
                elif key == 'h':
                    self.print_instructions()
                elif key == '\x1b':
                    print('Exiting...')
                    break
                
                for i in range(3):
                    self.joint_positions[i] = self.clamp(
                        self.joint_positions[i],
                        self.limits[i][0],
                        self.limits[i][1]
                    )
                
                self.publish_joint_states()

def main():
    rclpy.init()
    node = KeyboardControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
