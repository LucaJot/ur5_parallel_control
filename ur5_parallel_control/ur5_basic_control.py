import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


class Ur5JointController(Node):
    def __init__(self):
        super().__init__("position_control_node")
        self.traj_publisher = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )
        self.state_subscriber = self.create_subscription(
            JointTrajectoryControllerState,
            "/scaled_joint_trajectory_controller/state",
            self.joint_state_callback,
            10,
        )
        self.current_joint_positions: dict = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.6,
            "elbow_joint": 1.8,
            "wrist_1_joint": -3.0,
            "wrist_2_joint": -1.7,
            "wrist_3_joint": 0.0,
        }
        self.traj_controller_state: JointTrajectoryControllerState = None

    def joint_state_callback(self, msg: JointState):
        self.traj_controller_state = msg
        self.current_joint_positions = dict(zip(msg.name, msg.position))

    def send_joint_command(
        self,
        target_angles: dict,
        duration: int = 3,
    ):
        """_summary_
        Function to send a joint position command to the UR5 robots joint trajectory controller.

        Args:
            target_angles (dict) : A dictionary containing the target joint angles for the UR5 robot.
        """

        # Create a JointTrajectory message
        jt_msg = JointTrajectory()
        jt_msg.joint_names = target_angles.keys()
        jt_point = JointTrajectoryPoint()
        jt_point.positions = target_angles.values()
        jt_point.time_from_start.sec = duration
        jt_msg.points.append(jt_point)

        # Publish the trajectory message
        self.traj_publisher.publish(jt_msg)
        self.get_logger().info("Sent joint position command")

    def set_joint_delta(self, angle_delta: list[float]):
        """_summary_
        Function to send a joint angle delta to the UR5 robots joint trajectory controller.
        Args:
            msg (list[float]): List of target joint angles in the following order
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",.
        """
        # Get the most recent joint positions
        current_joint_positions = self.current_joint_positions.position
        joint_names = self.current_joint_positions.name
        # If no joint positions received yet, warn the user and abort
        if current_joint_positions is None:
            self.get_logger().warn("No joint positions received yet")
            return
        else:
            # Calculate the new target joint positions by adding the delta to the current joint positions
            new_target = {
                "shoulder_pan_joint": angle_delta[0]
                + current_joint_positions["shoulder_pan_joint"],
                "shoulder_lift_joint": angle_delta[1]
                + current_joint_positions["shoulder_lift_joint"],
                "elbow_joint": angle_delta[2] + current_joint_positions["elbow_joint"],
                "wrist_1_joint": angle_delta[3]
                + current_joint_positions["wrist_1_joint"],
                "wrist_2_joint": angle_delta[4]
                + current_joint_positions["wrist_2_joint"],
                "wrist_3_joint": angle_delta[5]
                + current_joint_positions["wrist_3_joint"],
            }
            self.send_joint_command(new_target)

    def reset(self):
        """_summary_
        Function to reset the UR5 robot to its initial joint position.
        """
        self.get_logger().info("Resetting UR5 to initial joint position")
        init_positions: dict = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.6,
            "elbow_joint": 1.8,
            "wrist_1_joint": -3.0,
            "wrist_2_joint": -1.7,
            "wrist_3_joint": 0.0,
        }
        self.send_joint_command(self.init_positions, duration=3)


def main(args=None):
    rclpy.init(args=args)
    node = Ur5JointController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
