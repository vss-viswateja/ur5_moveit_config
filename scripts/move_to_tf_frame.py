#!/usr/bin/env python3
"""
Move to TF Frame Node

This node looks up a TF frame and moves the UR5 arm to that pose.
Uses MoveIt2 for motion planning and execution.

Usage:
    # Move to a specific TF frame
    ros2 run ur5_moveit_config move_to_tf_frame --ros-args -p target_frame:=aruco_0
    
    # Specify Z offset above the target (default: 0.1m)
    ros2 run ur5_moveit_config move_to_tf_frame --ros-args -p target_frame:=aruco_0 -p z_offset:=0.15
    
    # Plan only (don't execute)
    ros2 run ur5_moveit_config move_to_tf_frame --ros-args -p target_frame:=aruco_0 -p execute:=false
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit_msgs.msg import (
    MoveItErrorCodes,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from action_msgs.msg import GoalStatus


class MoveToTFFrameNode(Node):
    """Node that moves UR5 arm to a specified TF frame."""

    def __init__(self):
        super().__init__('move_to_tf_frame')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('target_frame', '')
        self.declare_parameter('robot_namespace', 'ur5')
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('end_effector_link', 'ur5/link6_1')
        self.declare_parameter('reference_frame', 'ur5/world')
        self.declare_parameter('execute', True)
        self.declare_parameter('z_offset', 0.1)  # Offset above target (meters)
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('position_tolerance', 0.01)  # 1cm
        self.declare_parameter('orientation_tolerance', 0.1)  # ~6 degrees
        self.declare_parameter('tf_timeout', 15.0)  # TF lookup timeout
        
        self.target_frame = self.get_parameter('target_frame').value
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.planning_group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.execute = self.get_parameter('execute').value
        self.z_offset = self.get_parameter('z_offset').value
        self.planning_time = self.get_parameter('planning_time').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        
        # Validate target frame
        if not self.target_frame:
            self.get_logger().error('No target_frame specified! Use: -p target_frame:=<frame_name>')
            sys.exit(1)
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State tracking
        self.joint_states_received = False
        self.latest_joint_state = None
        self.goal_done = False
        self.goal_result = None
        
        # Build topic names with namespace
        joint_states_topic = f'/{self.robot_namespace}/joint_states'
        move_action_topic = f'/{self.robot_namespace}/move_action'
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            move_action_topic,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Move to TF Frame Node Started')
        self.get_logger().info(f'  Target Frame: {self.target_frame}')
        self.get_logger().info(f'  Reference Frame: {self.reference_frame}')
        self.get_logger().info(f'  End Effector: {self.ee_link}')
        self.get_logger().info(f'  Z Offset: {self.z_offset}m')
        self.get_logger().info(f'  Execute: {self.execute}')
        self.get_logger().info('=' * 60)

    def joint_state_callback(self, msg: JointState):
        """Callback for joint states."""
        self.joint_states_received = True
        self.latest_joint_state = msg

    def wait_for_joint_states(self, timeout: float = 5.0) -> bool:
        """Wait until we receive joint states."""
        self.get_logger().info('Waiting for joint states...')
        start_time = time.time()
        while not self.joint_states_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.joint_states_received:
            self.get_logger().info('✓ Joint states received')
            return True
        else:
            self.get_logger().error('✗ No joint states received!')
            return False

    def wait_for_move_group(self, timeout: float = 10.0) -> bool:
        """Wait for MoveGroup action server."""
        self.get_logger().info('Waiting for MoveGroup action server...')
        if self.move_group_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().info('✓ MoveGroup action server available')
            return True
        else:
            self.get_logger().error('✗ MoveGroup action server not available!')
            return False

    def lookup_tf_frame(self) -> TransformStamped:
        """Look up the target TF frame relative to the reference frame."""
        timeout = self.tf_timeout
        self.get_logger().info(f'Looking up TF: {self.target_frame} in {self.reference_frame} (timeout: {timeout}s)...')
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    self.target_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                self.get_logger().info('✓ TF frame found')
                return transform
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'TF lookup failed: {e}, retrying...')
                rclpy.spin_once(self, timeout_sec=0.5)
        
        self.get_logger().error(f'✗ Could not find TF frame: {self.target_frame}')
        return None

    def create_pose_goal(self, transform: TransformStamped) -> MoveGroup.Goal:
        """Create a MoveGroup goal from a TF transform."""
        goal = MoveGroup.Goal()
        
        # Request configuration
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        
        # Workspace bounds
        goal.request.workspace_parameters.header.frame_id = self.reference_frame
        goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal.request.workspace_parameters.min_corner.x = -3.0
        goal.request.workspace_parameters.min_corner.y = -3.0
        goal.request.workspace_parameters.min_corner.z = -1.0
        goal.request.workspace_parameters.max_corner.x = 3.0
        goal.request.workspace_parameters.max_corner.y = 3.0
        goal.request.workspace_parameters.max_corner.z = 3.0
        
        # Extract position from transform (with Z offset)
        target_x = transform.transform.translation.x
        target_y = transform.transform.translation.y
        target_z = transform.transform.translation.z + self.z_offset
        
        # Use the frame's orientation or a fixed downward orientation
        # For picking, we typically want the gripper pointing down
        target_qx = transform.transform.rotation.x
        target_qy = transform.transform.rotation.y
        target_qz = transform.transform.rotation.z
        target_qw = transform.transform.rotation.w
        
        self.get_logger().info(f'Target pose (with z_offset={self.z_offset}m):')
        self.get_logger().info(f'  Position: [{target_x:.3f}, {target_y:.3f}, {target_z:.3f}]')
        self.get_logger().info(f'  Orientation: [{target_qx:.3f}, {target_qy:.3f}, {target_qz:.3f}, {target_qw:.3f}]')
        
        # Create pose goal using position and orientation constraints
        constraints = Constraints()
        constraints.name = 'tf_frame_goal'
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.reference_frame
        position_constraint.header.stamp = self.get_clock().now().to_msg()
        position_constraint.link_name = self.ee_link
        
        # Define a small bounding box around the target
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.position_tolerance]
        bounding_volume.primitives.append(primitive)
        
        # Target pose
        target_pose = Pose()
        target_pose.position.x = target_x
        target_pose.position.y = target_y
        target_pose.position.z = target_z
        bounding_volume.primitive_poses.append(target_pose)
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.reference_frame
        orientation_constraint.header.stamp = self.get_clock().now().to_msg()
        orientation_constraint.link_name = self.ee_link
        orientation_constraint.orientation.x = target_qx
        orientation_constraint.orientation.y = target_qy
        orientation_constraint.orientation.z = target_qz
        orientation_constraint.orientation.w = target_qw
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        # Planning options
        goal.planning_options.plan_only = not self.execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        
        return goal

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.goal_done = True
            return
        
        self.get_logger().info('Goal accepted, planning and executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle goal result."""
        result = future.result()
        self.goal_result = result.result
        self.goal_done = True
        
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('\033[92m✓ Motion completed successfully!\033[0m')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'Motion aborted! Error code: {self.goal_result.error_code.val}')
            self.print_error_code(self.goal_result.error_code.val)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Motion was canceled')
        else:
            self.get_logger().warn(f'Motion ended with status: {status}')

    def feedback_callback(self, feedback_msg):
        """Handle motion feedback."""
        feedback = feedback_msg.feedback
        state = feedback.state
        self.get_logger().info(f'Motion state: {state}', throttle_duration_sec=1.0)

    def print_error_code(self, code: int):
        """Print human-readable error code."""
        error_map = {
            MoveItErrorCodes.PLANNING_FAILED: "Planning failed",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "Invalid motion plan",
            MoveItErrorCodes.CONTROL_FAILED: "Control failed",
            MoveItErrorCodes.TIMED_OUT: "Timed out",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "Start state in collision",
            MoveItErrorCodes.GOAL_IN_COLLISION: "Goal in collision",
            MoveItErrorCodes.INVALID_GROUP_NAME: "Invalid group name",
            MoveItErrorCodes.INVALID_LINK_NAME: "Invalid link name",
            MoveItErrorCodes.NO_IK_SOLUTION: "No IK solution",
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "Frame transform failure",
        }
        msg = error_map.get(code, f"Unknown error ({code})")
        self.get_logger().error(f'  → {msg}')

    def send_goal(self, goal: MoveGroup.Goal) -> bool:
        """Send goal and wait for completion."""
        self.goal_done = False
        self.goal_result = None
        
        self.get_logger().info('Sending motion goal...')
        
        send_goal_future = self.move_group_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Wait for goal to complete
        timeout = self.planning_time + 60.0  # Extra time for execution
        start_time = time.time()
        
        while not self.goal_done and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.goal_done:
            self.get_logger().error('Goal timed out!')
            return False
        
        if self.goal_result:
            error_code = self.goal_result.error_code.val
            if error_code == MoveItErrorCodes.SUCCESS:
                return True
            else:
                self.get_logger().error(f'Motion failed with error code: {error_code}')
                self.print_error_code(error_code)
                return False
        
        return False

    def run(self):
        """Main execution loop."""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info(f'Moving arm to TF frame: {self.target_frame}')
        self.get_logger().info('=' * 60)
        
        # Wait for prerequisites
        if not self.wait_for_joint_states():
            return False
        if not self.wait_for_move_group():
            return False
        
        # Look up TF frame
        transform = self.lookup_tf_frame()
        if transform is None:
            return False
        
        # Log the transform
        t = transform.transform.translation
        r = transform.transform.rotation
        self.get_logger().info(f'\nTF Transform ({self.target_frame} in {self.reference_frame}):')
        self.get_logger().info(f'  Translation: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}')
        self.get_logger().info(f'  Rotation: x={r.x:.3f}, y={r.y:.3f}, z={r.z:.3f}, w={r.w:.3f}')
        
        # Create and send goal
        goal = self.create_pose_goal(transform)
        success = self.send_goal(goal)
        
        # Summary
        self.get_logger().info('\n' + '=' * 60)
        if success:
            self.get_logger().info(f'\033[92m✓ Successfully moved to {self.target_frame}!\033[0m')
        else:
            self.get_logger().info(f'\033[91m✗ Failed to move to {self.target_frame}!\033[0m')
        self.get_logger().info('=' * 60)
        
        return success


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveToTFFrameNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Give time for TF buffer to fill with transforms
        node.get_logger().info('Waiting for TF buffer to fill...')
        for i in range(30):  # 3 seconds of spinning
            rclpy.spin_once(node, timeout_sec=0.1)
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
