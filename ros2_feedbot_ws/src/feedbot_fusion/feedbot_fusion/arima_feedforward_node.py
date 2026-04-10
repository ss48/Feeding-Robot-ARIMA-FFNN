"""
ARIMA Feedforward Node — Integrates ARIMA-FFNN predictions into arm control.

Sits between the FSM and the arm controller. When enabled, it adjusts
trajectory goals using ARIMA predictions to reduce tracking error
via feedforward compensation.

Mode: pid_only  — passes trajectory through unchanged (baseline)
Mode: pid_arima — adds feedforward offset from ARIMA prediction error

The feedforward offset is computed as:
  offset[i] = gain * (predicted[i] - current[i])

This anticipates where the joint will be, allowing the PID controller
to start correcting earlier rather than waiting for the error to appear.

Subscribes:
  /predicted_state        (Float64MultiArray) — ARIMA predicted joints
  /joint_states           (JointState)        — current joint positions
  /arm_controller/follow_joint_trajectory (action goal, intercepted)

Publishes:
  /benchmark/feedforward_active (Bool) — whether feedforward is applied

Parameters:
  mode             (str)   — 'pid_only' or 'pid_arima'
  feedforward_gain (float) — how much of the prediction to apply (0.0-1.0)
  prediction_horizon (float) — how far ahead to predict (seconds)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from builtin_interfaces.msg import Duration

JOINT_NAMES = ['base_y_joint', 'lower_z_joint', 'upper_z_joint', 'feeder_joint']


class ArimaFeedforwardNode(Node):

    def __init__(self):
        super().__init__('arima_feedforward_node')

        self.declare_parameter('mode', 'pid_only')
        self.declare_parameter('feedforward_gain', 0.3)
        self.declare_parameter('prediction_horizon', 0.2)

        self.mode = self.get_parameter('mode').value
        self.ff_gain = self.get_parameter('feedforward_gain').value
        self.horizon = self.get_parameter('prediction_horizon').value

        self.current_positions = [0.0] * 4
        self.predicted_positions = [0.0] * 4
        self.prediction_confidence = 0.0

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/predicted_state', self._pred_cb, 10)

        # Feedforward status publisher
        self.ff_pub = self.create_publisher(
            Bool, '/benchmark/feedforward_active', 10)

        # Action server: intercept trajectory goals from FSM
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            '/feedforward/follow_joint_trajectory',
            self._execute_goal)

        # Action client: forward (possibly modified) goals to real controller
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        self.get_logger().info(
            f'ARIMA feedforward node — mode: {self.mode}, '
            f'gain: {self.ff_gain}, horizon: {self.horizon}s')

    def _joint_cb(self, msg):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                self.current_positions[i] = msg.position[msg.name.index(name)]

    def _pred_cb(self, msg):
        if len(msg.data) >= 4:
            self.predicted_positions = list(msg.data[:4])

    async def _execute_goal(self, goal_handle):
        """Intercept trajectory goal, apply feedforward if enabled, forward to controller."""
        goal = goal_handle.request

        if self.mode == 'pid_arima':
            goal = self._apply_feedforward(goal)
            self.ff_pub.publish(Bool(data=True))
        else:
            self.ff_pub.publish(Bool(data=False))

        # Forward to real controller
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Arm controller not available')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        send_goal_future = self._action_client.send_goal_async(
            FollowJointTrajectory.Goal(trajectory=goal.trajectory))

        goal_result = await send_goal_future
        if not goal_result.accepted:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        result_future = goal_result.get_result_async()
        result = await result_future

        goal_handle.succeed()
        return result.result

    def _apply_feedforward(self, goal):
        """Apply ARIMA feedforward compensation to trajectory points."""
        for point in goal.trajectory.points:
            if len(point.positions) < 4:
                continue

            adjusted = list(point.positions)
            for i in range(4):
                # Prediction error: how far ARIMA thinks we'll be from target
                pred_error = self.predicted_positions[i] - self.current_positions[i]

                # Feedforward: pre-compensate by moving toward where ARIMA
                # predicts we need to be
                offset = self.ff_gain * pred_error
                adjusted[i] = point.positions[i] + offset

            point.positions = adjusted

            # Add velocity hints from prediction if not already set
            if not point.velocities:
                point.velocities = [0.0] * 4
                for i in range(4):
                    # Predicted velocity = (predicted - current) / horizon
                    pred_vel = (self.predicted_positions[i] - self.current_positions[i]) / max(self.horizon, 0.05)
                    point.velocities[i] = self.ff_gain * pred_vel

        self.get_logger().info(
            f'Feedforward applied (gain={self.ff_gain})',
            throttle_duration_sec=2.0)

        return goal


def main(args=None):
    rclpy.init(args=args)
    node = ArimaFeedforwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
