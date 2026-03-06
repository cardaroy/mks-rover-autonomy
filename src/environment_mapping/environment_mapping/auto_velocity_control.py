import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import random

class AutoVelocityControl(Node):
    def __init__(self):
        super().__init__('auto_velocity_control')

        # Declare parameters for max speeds
        self.declare_parameter('max_linear_speed', 8000.0)
        self.declare_parameter('max_angular_speed', 12000.0)

        # Get the parameters
        self.max_lin_ = self.get_parameter('max_linear_speed').value
        self.max_ang_ = self.get_parameter('max_angular_speed').value

        # Publisher for all 4 wheel velocity commands
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)

        # Current random speeds (initialized once)
        self.current_lin_speed = self.get_random_lin_speed(self.max_lin_)
        self.current_ang_speed = self.get_random_ang_speed(self.max_ang_)

        # Publish at 200 Hz (0.005 seconds)
        self.publish_timer = self.create_timer(0.005, self.publish_velocity)

        # Change speed every 5 seconds
        self.change_timer = self.create_timer(9.0, self.change_velocity)

    def publish_velocity(self):
        left_speed = self.current_lin_speed - self.current_ang_speed
        right_speed = self.current_lin_speed + self.current_ang_speed

        cmd = Float64MultiArray(data=[left_speed, right_speed, left_speed, right_speed])
        self.wheel_cmd_pub_.publish(cmd)

    def change_velocity(self):
        self.current_lin_speed = self.get_random_lin_speed(self.max_lin_)
        self.current_ang_speed = self.get_random_ang_speed(self.max_ang_)
        self.get_logger().info(
            f"New speeds - lin: {self.current_lin_speed}, ang: {self.current_ang_speed}"
        )

    def get_random_ang_speed(self, max_speed):
        min_pct = -0.3
        max_pct = 0.3
        random_pct = random.uniform(min_pct, max_pct)
        return random_pct * max_speed
    
    def get_random_lin_speed(self, max_speed):
        min_pct = 0.8
        max_pct = 1
        random_pct = random.uniform(min_pct, max_pct)
        return random_pct * max_speed


def main(args=None):
    rclpy.init(args=args)

    auto_velocity_control = AutoVelocityControl()

    try:
        rclpy.spin(auto_velocity_control)
    except KeyboardInterrupt:
        pass
    finally:
        auto_velocity_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
