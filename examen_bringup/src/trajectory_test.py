#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryTest(Node):

    def __init__(self):
        super().__init__('trajectory_test')
        topic_name= "/scara_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name,10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.joints = ['link_1_joint', 'link_2_joint', 'link_3_joint']
        self.goal_positions = [1.57, 1.57, 1.57]
        self.get_logger().info('Controller is running and publishing to topic: {}'.format(topic_name))

    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher_node = TrajectoryTest()
    rclpy.spin(trajectory_publisher_node)
    trajectory_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        



