import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        self.position_estimate = np.array([0.0, 0.0, 0.0])

    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)

    def _calculate_residual_element(self, range: Range) -> float:
        a = np.array([range.anchor.x, range.anchor.y, range.anchor.z])
        return range.range - np.linalg.norm(self.position_estimate - a)

    def _calculate_residual_gradient_element(self, range: Range) -> float:
        a = np.array([range.anchor.x, range.anchor.y, range.anchor.z])
        return -(self.position_estimate - a) / np.linalg.norm(self.position_estimate - a)
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return self.position_estimate
        
        residual = np.array([self._calculate_residual_element(range) for range in self.anchor_ranges])

        residual_gradient = np.array([self._calculate_residual_gradient_element(range) for range in self.anchor_ranges])
        
        self.position_estimate = self.position_estimate - np.linalg.pinv(residual_gradient) @ residual
        
        position = self.position_estimate[0], self.position_estimate[1], self.position_estimate[2]
        # self.get_logger().info(f'position = {position}')
        return position


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
