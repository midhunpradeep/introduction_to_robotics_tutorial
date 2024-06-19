import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
import tf_transformations

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        self.previous_position = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose_marker', 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
    
    @staticmethod
    def _calculate_distance(point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance
    
    @staticmethod
    def _calculate_angle(point1, point2):
        angle = 90 - math.degrees(math.atan2(
                point2[1] - point1[1], 
                point2[0] - point1[0]
            ))
        if (angle < 0):
            angle += 360
        return angle
        
    def timer_cb(self):
        if not self.position or not self.goal:
            return
        
        self.get_logger().info(f"posttion = {self.position}, goal = {self.goal}")
    
        if not self.previous_position:
            self.previous_position = self.position

        msg = Twist()

        angle = 0
        if VelocityController._calculate_distance(self.previous_position, self.position) > 0.01:
            angle = VelocityController._calculate_angle(self.previous_position, self.position)
            self.previous_position = self.position

        goal_distance = VelocityController._calculate_distance(self.position, self.goal)

        goal_angle = VelocityController._calculate_angle(self.position, self.goal)

        turn_angle = ((goal_angle - angle + 180) % 360) - 180

        distance = self.forward_distance - 0.3

        if (distance > 0.3):
            msg.linear.x = 0.075
            if abs(turn_angle) >= 10:
                if (turn_angle < 0):
                    msg.angular.z = 0.3
                elif (turn_angle > 0):
                    msg.angular.z = -0.3
        elif (goal_distance >= 0.3):
            msg.angular.z = 0.1

        self.publisher.publish(msg)
        self.publish_marker(self.position, angle, relative=True)
    
    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        forward_distance = msg.ranges[0]

        num_ranges = 31
        for i in range(num_ranges):
            j = i - (num_ranges // 2)
            forward_distance = min(forward_distance, msg.ranges[j])
        
        self.forward_distance = forward_distance
        
    def position_cb(self, msg):
        self.position = msg.point.x, msg.point.y
    
    def publish_marker(self, position, angle, relative=False):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if not relative:
            msg.header.frame_id = 'map'
        else:
            msg.header.frame_id = 'base_link'

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, angle)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

