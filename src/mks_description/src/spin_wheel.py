import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.publisher_ = self.create_publisher(Float64, '/front_left_wheel_to_leg/command', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)
        self.get_logger().info('Wheel controller node has started.')

    def publish_velocity(self):
        msg = Float64()
        msg.data = 1.0  # You can set the velocity here (positive for forward, negative for reverse)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing velocity: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    wheel_controller = WheelController()
    rclpy.spin(wheel_controller)
    wheel_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
