import rclpy
from rclpy.node import Node

class SimpleArmNode(Node):
    def __init__(self):
        super().__init__('simple_arm')
        self.get_logger().info("Robot Arm Node Started!")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

