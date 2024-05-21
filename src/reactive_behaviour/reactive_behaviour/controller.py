import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn

class VelocityController(DrivingSwarmNode):

    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0.0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()
        
    def timer_cb(self):
        if not self.started:
            return
        msg = Twist()

        distance = self.forward_distance - 0.3

        if (distance > 0.3):
            msg.linear.x = 0.075
        else:
            msg.angular.z = 0.1

        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        forward_distance = msg.ranges[0]

        num_ranges = 31
        for i in range(num_ranges):
            j = i - (num_ranges // 2)
            forward_distance = min(forward_distance, msg.ranges[j])
        
        self.forward_distance = forward_distance



def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
