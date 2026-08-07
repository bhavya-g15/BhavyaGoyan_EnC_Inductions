import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool

class RoverStatusPublisher(Node):
    def __init__(self):
        super().__init__('rover_status_publisher')
        
        self.pub_battery = self.create_publisher(Float32, '/battery_level', 10)
        self.pub_mode = self.create_publisher(String, '/rover_mode', 10)
        self.pub_stop = self.create_publisher(Bool, '/emergency_stop', 10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_val = 100.0

    def timer_callback(self):
        bat_msg = Float32()
        bat_msg.data = self.battery_val
        
        mode_msg = String()
        mode_msg.data = "AUTONOMOUS"
        
        stop_msg = Bool()
        stop_msg.data = False

        self.pub_battery.publish(bat_msg)
        self.pub_mode.publish(mode_msg)
        self.pub_stop.publish(stop_msg)
        
        self.get_logger().info(f'Publishing: Battery={self.battery_val}%, Mode={mode_msg.data}, Stop={stop_msg.data}')
        self.battery_val -= 0.5

def main(args=None):
    rclpy.init(args=args)
    node = RoverStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
