#writting a node for forward kinematics
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        self.wheel_base = 0.59
        self.wheel_radius = 0.11

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.get_logger().info('Forward Kinematics Node has started!')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        right_wheel_rad_s = (linear_x + (angular_z * self.wheel_base / 2.0)) / self.wheel_radius
        left_wheel_rad_s = (linear_x - (angular_z * self.wheel_base / 2.0)) / self.wheel_radius
        
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        joint_state_msg.header.frame_id = 'base_link'
        joint_state_msg.name = [
            'steer1', 'steer2', 'steer_3', 'steer4',
            'wheel_1', 'wheel_2', 'wheel_3', 'wheel_4'
        ]
        
        joint_state_msg.position = [0.0] * 8 # Sets all 8 positions to 0.0
        
        # Velocities are assigned in the same order as the names above
        joint_state_msg.velocity = [
            0.0, 0.0, 0.0, 0.0,   # steer1, steer2, steer_3, steer4
            left_wheel_rad_s,    # wheel_1
            right_wheel_rad_s,   # wheel_2
            right_wheel_rad_s,   # wheel_3
            left_wheel_rad_s     # wheel_4
        ]
        
        self.joint_state_pub.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
