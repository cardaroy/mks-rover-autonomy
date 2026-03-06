import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class WheelSpinner(Node):
    def __init__(self):
        super().__init__('wheel_spinner')
        # Topic to control the joint
        self.publisher = self.create_publisher(Float64, '/front_left_wheel_to_leg/command', 10)
        self.timer = self.create_timer(0.1, self.spin_wheel)
        self.speed = 5.0  # radians per second

    def spin_wheel(self):
        msg = Float64()
        msg.data = self.speed
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelSpinner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
