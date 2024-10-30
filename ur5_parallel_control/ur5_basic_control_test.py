import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PositionControlNode(Node):
    def __init__(self):
        super().__init__("position_control_node")
        self.publisher = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )
        self.timer = self.create_timer(1.0, self.send_joint_command)

    def send_joint_command(self):
        # Define joint names and target positions
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        target_positions = [0.0, -1.6, 1.8, -3.0, -1.7, 0.0]

        # Create a JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = 3  # move over 2 seconds
        msg.points.append(point)

        # Publish the command
        self.publisher.publish(msg)
        self.get_logger().info("Sent joint position command")


def main(args=None):
    rclpy.init(args=args)
    node = PositionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
