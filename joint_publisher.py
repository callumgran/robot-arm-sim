import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 10)
        self.timer = self.create_timer(2.0, self.publish_positions)

    def publish_positions(self):
        msg = Float64MultiArray()
        msg.data = [1.0, -0.5, 0.3]  # Desired angles in radians
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint positions: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
