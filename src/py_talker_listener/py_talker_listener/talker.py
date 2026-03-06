import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_msg)
        self.count = 0

    def publish_msg(self):
        msg = String()
        msg.data = f'Hello ROS 2 #{self.count}'
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    rclpy.shutdown()
