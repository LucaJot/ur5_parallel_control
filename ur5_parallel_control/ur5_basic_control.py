import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import rclpy.wait_for_message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64MultiArray
import threading
from builtin_interfaces.msg import Duration


class Ur5JointController(Node):
    def __init__(self, v_cm=25, f_update=60):  # Max v = 25 cm/s
        super().__init__("position_control_node")
        # self.lock = threading.Lock()
        self.d_t = 1 / f_update  # Time between updates
        # Check if max speed is within limits (longest link = 44 cm)
        if v_cm > 25:
            self.v_cm = 0.25
            self.get_logger().warn(
                "Max speed is too high. Setting Max speed v_cm = 25 cm/s"
            )

        self.d_phi = v_cm * (1 / f_update) / 44  # Max angle delta per update

        # Read the current joint positions from the joint state topic
        self.traj_controller_state: JointTrajectoryControllerState = None
        self.state_subscriber = self.create_subscription(
            JointTrajectoryControllerState,
            "/scaled_joint_trajectory_controller/controller_state",
            self.joint_state_callback,
            10,
        )
        # Create a publisher to send joint position commands to the UR5 robot
        self.traj_publisher = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )
        # Create a subscriber to read the agents control commands
        self.joint_cmd = self.create_subscription(
            Float64MultiArray, "/joint_cmd", self.set_joint_delta, 10
        )

        # Delta list to store the most recent control command
        self.current_angle_delta: Float64MultiArray = [0.0] * 6
        # Current joint positions as received from the joint state topic
        self.current_joint_positions: list[float] = None

        # Set the initial joint positions to a default pose  TODO (implement reset to home position)
        self.init_joint_positions: list[float] = [
            0.0,  # shoulder_pan_joint
            -1.6,  # shoulder_lift_joint
            1.6,  # elbow_joint
            -3.0,  # wrist_1_joint
            -1.7,  # wrist_2_joint
            0.0,  # wrist_3_joint
        ]

        # Define the joint names and order
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Set update rate for the joint command
        self.update_timer = self.create_timer(self.d_t, self.send_joint_command)

    def joint_state_callback(self, msg: JointTrajectoryControllerState):
        # Lock the thread when updating the joint positions

        self.traj_controller_state = msg
        # Create a dict to sort the values in defined order
        name_val_mapping = dict(zip(msg.joint_names, msg.feedback.positions))
        self.current_joint_positions = [
            name_val_mapping[joint]
            for joint in self.joint_names  # use dict to get the values in the order of joint_names
        ]

    def set_joint_delta(self, msg: Float64MultiArray):

        normalized_delta = msg.data
        # Check if the received message has the correct length
        if normalized_delta and (len(normalized_delta) != 6):
            self.get_logger().warn("Received invalid joint delta command")
            return
        # Denormalize the angles
        angle_delta = [norm_val * self.d_phi for norm_val in normalized_delta]
        self.current_angle_delta = angle_delta

    def send_joint_command(
        self, duration: float = 0
    ):  # TODO Duration maybe used for init to home?
        """ """

        # If no duration is provided, use the default duration
        if duration == 0:
            duration = self.d_t
        # Get the most recent joint positions dict
        self.current_joint_positions
        # If no joint positions received yet abort
        if self.current_joint_positions is None:
            return

        # Calculate the new target joint positions by adding the delta to the current joint positions
        intermidiate_target = [
            delta / 2 + position
            for delta, position in zip(
                self.current_angle_delta, self.current_joint_positions
            )
        ]

        new_target = [
            delta + position
            for delta, position in zip(
                self.current_angle_delta, self.current_joint_positions
            )
        ]

        # Enforce joint limits
        new_target = self.enforce_joint_limits(new_target)

        # Create a JointTrajectory message
        jt_msg = JointTrajectory()
        jt_msg.joint_names = self.joint_names
        jt_point = JointTrajectoryPoint()
        jt_point.positions = new_target
        jt_point.time_from_start = Duration(nanosec=int(duration * 1e9))
        jt_msg.points.append(jt_point)

        # Publish the trajectory message
        self.traj_publisher.publish(jt_msg)

    def enforce_joint_limits(self, target_angles: list[float]):
        """Safty layer to avoid damaging the robot by exceeding joint limits."""
        # TODO  Implement joint limits
        new_target_angles = target_angles
        return new_target_angles

    def reset(self):
        """_summary_
        Function to reset the UR5 robot to its initial joint position.
        """
        self.get_logger().info("Resetting UR5 to initial joint position")
        self.send_joint_command(self.init_joint_positions, duration=3)


def main(args=None):
    rclpy.init(args=args)
    node = Ur5JointController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
