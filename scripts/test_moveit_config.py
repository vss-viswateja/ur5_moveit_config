#!/usr/bin/env python3
"""
MoveIt2 Motion Test Node for UR5

This node tests MoveIt2 by actually moving the robot arm to goal poses.

Usage:
    # Move to predefined joint positions
    ros2 run ur5_moveit_config test_moveit_config
    
    # Move to a specific named pose
    ros2 run ur5_moveit_config test_moveit_config --ros-args -p target_pose:=up
    
    # Move to a cartesian pose
    ros2 run ur5_moveit_config test_moveit_config --ros-args \
        -p use_cartesian:=true -p target_x:=0.4 -p target_y:=0.0 -p target_z:=0.5
    
    # Plan only (don't execute)
    ros2 run ur5_moveit_config test_moveit_config --ros-args -p execute:=false
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    RobotState,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive

import time
from action_msgs.msg import GoalStatus


class MoveItMotionTester(Node):
    """Test node that moves the UR5 arm using MoveIt2."""

    # Joint names for UR5
    JOINT_NAMES = [
        'base_link1_joint',
        'link1_link2p1_joint',
        'link2p3_link3_joint',
        'link3_link4_joint',
        'link4_link5_joint',
        'link5_link6_joint',
    ]
    
    # Predefined poses (joint values in radians)
    POSES = {
        'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'ready': [0.0, -0.5, 1.0, -0.5, 0.0, 0.0],
        'up': [0.0, -1.57, 0.0, 0.0, 0.0, 0.0],
        'left': [1.57, -0.5, 1.0, -0.5, 0.0, 0.0],
        'right': [-1.57, -0.5, 1.0, -0.5, 0.0, 0.0],
    }

    def __init__(self):
        super().__init__('moveit_motion_tester')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('execute', True)
        self.declare_parameter('target_pose', 'ready')  # Named pose to go to
        self.declare_parameter('target_x', 0.0)  # For cartesian goal
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('use_cartesian', False)  # Use cartesian vs joint goal
        self.declare_parameter('end_effector_link', 'link6')
        self.declare_parameter('planning_time', 5.0)
        
        self.planning_group = self.get_parameter('planning_group').value
        self.execute = self.get_parameter('execute').value
        self.target_pose_name = self.get_parameter('target_pose').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_z = self.get_parameter('target_z').value
        self.use_cartesian = self.get_parameter('use_cartesian').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.planning_time = self.get_parameter('planning_time').value
        
        # State tracking
        self.joint_states_received = False
        self.latest_joint_state = None
        self.goal_done = False
        self.goal_result = None
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            'move_action',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('MoveIt2 Motion Tester Started')
        self.get_logger().info(f'Planning Group: {self.planning_group}')
        self.get_logger().info(f'Execute Motion: {self.execute}')
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

    def get_current_joint_values(self) -> dict:
        """Get current joint values as a dictionary."""
        if not self.latest_joint_state:
            return {}
        
        values = {}
        for i, name in enumerate(self.latest_joint_state.name):
            if i < len(self.latest_joint_state.position):
                values[name] = self.latest_joint_state.position[i]
        return values

    def create_joint_goal(self, joint_values: list) -> MoveGroup.Goal:
        """Create a MoveGroup goal for joint positions."""
        goal = MoveGroup.Goal()
        
        # Request configuration
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Workspace bounds
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal.request.workspace_parameters.min_corner.x = -2.0
        goal.request.workspace_parameters.min_corner.y = -2.0
        goal.request.workspace_parameters.min_corner.z = -2.0
        goal.request.workspace_parameters.max_corner.x = 2.0
        goal.request.workspace_parameters.max_corner.y = 2.0
        goal.request.workspace_parameters.max_corner.z = 2.0
        
        # Create joint constraints as goal
        constraints = Constraints()
        constraints.name = 'joint_goal'
        
        for i, joint_name in enumerate(self.JOINT_NAMES):
            if i < len(joint_values):
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = joint_values[i]
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(constraints)
        
        # Planning options
        goal.planning_options.plan_only = not self.execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        
        return goal

    def create_pose_goal(self, x: float, y: float, z: float, 
                         qx: float = 0.0, qy: float = 0.707, 
                         qz: float = 0.0, qw: float = 0.707) -> MoveGroup.Goal:
        """Create a MoveGroup goal for a cartesian pose."""
        goal = MoveGroup.Goal()
        
        # Request configuration
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        
        # Workspace bounds
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal.request.workspace_parameters.min_corner.x = -2.0
        goal.request.workspace_parameters.min_corner.y = -2.0
        goal.request.workspace_parameters.min_corner.z = -2.0
        goal.request.workspace_parameters.max_corner.x = 2.0
        goal.request.workspace_parameters.max_corner.y = 2.0
        goal.request.workspace_parameters.max_corner.z = 2.0
        
        # Create pose goal using position and orientation constraints
        constraints = Constraints()
        constraints.name = 'pose_goal'
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'world'
        position_constraint.header.stamp = self.get_clock().now().to_msg()
        position_constraint.link_name = self.ee_link
        
        # Define a small bounding box around the target
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm radius tolerance
        bounding_volume.primitives.append(primitive)
        
        # Target pose
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        bounding_volume.primitive_poses.append(target_pose)
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.header.stamp = self.get_clock().now().to_msg()
        orientation_constraint.link_name = self.ee_link
        orientation_constraint.orientation.x = qx
        orientation_constraint.orientation.y = qy
        orientation_constraint.orientation.z = qz
        orientation_constraint.orientation.w = qw
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
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
        
        self.get_logger().info('Goal accepted, waiting for result...')
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
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Motion was canceled')
        else:
            self.get_logger().warn(f'Motion ended with status: {status}')

    def feedback_callback(self, feedback_msg):
        """Handle motion feedback."""
        feedback = feedback_msg.feedback
        state = feedback.state
        self.get_logger().info(f'Motion state: {state}', throttle_duration_sec=1.0)

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
        timeout = self.planning_time + 30.0  # Extra time for execution
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

    def print_error_code(self, code: int):
        """Print human-readable error code."""
        error_map = {
            MoveItErrorCodes.PLANNING_FAILED: "Planning failed",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "Invalid motion plan",
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "Plan invalidated by environment",
            MoveItErrorCodes.CONTROL_FAILED: "Control failed",
            MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "Unable to acquire sensor data",
            MoveItErrorCodes.TIMED_OUT: "Timed out",
            MoveItErrorCodes.PREEMPTED: "Preempted",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "Start state in collision",
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "Start state violates constraints",
            MoveItErrorCodes.GOAL_IN_COLLISION: "Goal in collision",
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "Goal violates constraints",
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "Goal constraints violated",
            MoveItErrorCodes.INVALID_GROUP_NAME: "Invalid group name",
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "Invalid goal constraints",
            MoveItErrorCodes.INVALID_ROBOT_STATE: "Invalid robot state",
            MoveItErrorCodes.INVALID_LINK_NAME: "Invalid link name",
            MoveItErrorCodes.INVALID_OBJECT_NAME: "Invalid object name",
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "Frame transform failure",
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "Collision checking unavailable",
            MoveItErrorCodes.ROBOT_STATE_STALE: "Robot state stale",
            MoveItErrorCodes.SENSOR_INFO_STALE: "Sensor info stale",
            MoveItErrorCodes.NO_IK_SOLUTION: "No IK solution",
        }
        msg = error_map.get(code, f"Unknown error ({code})")
        self.get_logger().error(f'  → {msg}')

    def move_to_named_pose(self, pose_name: str) -> bool:
        """Move to a predefined named pose."""
        if pose_name not in self.POSES:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            self.get_logger().info(f'Available poses: {list(self.POSES.keys())}')
            return False
        
        joint_values = self.POSES[pose_name]
        self.get_logger().info(f'\nMoving to pose: "{pose_name}"')
        self.get_logger().info(f'Joint values: {[f"{v:.2f}" for v in joint_values]}')
        
        goal = self.create_joint_goal(joint_values)
        return self.send_goal(goal)

    def move_to_cartesian_pose(self, x: float, y: float, z: float) -> bool:
        """Move end-effector to a cartesian position."""
        self.get_logger().info(f'\nMoving to cartesian pose: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        goal = self.create_pose_goal(x, y, z)
        return self.send_goal(goal)

    def run_demo(self):
        """Run a demo moving through multiple poses."""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Starting Motion Demo')
        self.get_logger().info('=' * 60)
        
        # Wait for prerequisites
        if not self.wait_for_joint_states():
            return
        if not self.wait_for_move_group():
            return
        
        # Print current joint state
        current = self.get_current_joint_values()
        self.get_logger().info('\nCurrent joint positions:')
        for name in self.JOINT_NAMES:
            if name in current:
                self.get_logger().info(f'  {name}: {current[name]:.3f} rad')
        
        # Determine what motion to do
        if self.use_cartesian and (self.target_x != 0.0 or self.target_y != 0.0 or self.target_z != 0.0):
            # Move to cartesian target
            success = self.move_to_cartesian_pose(self.target_x, self.target_y, self.target_z)
        else:
            # Move to named pose
            success = self.move_to_named_pose(self.target_pose_name)
        
        # Summary
        self.get_logger().info('\n' + '=' * 60)
        if success:
            self.get_logger().info('\033[92m✓ Motion test PASSED!\033[0m')
        else:
            self.get_logger().info('\033[91m✗ Motion test FAILED!\033[0m')
        self.get_logger().info('=' * 60)
        
        # Print final joint state
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        final = self.get_current_joint_values()
        self.get_logger().info('\nFinal joint positions:')
        for name in self.JOINT_NAMES:
            if name in final:
                self.get_logger().info(f'  {name}: {final[name]:.3f} rad')

    def run_sequence(self):
        """Run through a sequence of poses."""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Starting Pose Sequence Demo')
        self.get_logger().info('=' * 60)
        
        # Wait for prerequisites
        if not self.wait_for_joint_states():
            return
        if not self.wait_for_move_group():
            return
        
        sequence = ['home', 'ready', 'up', 'ready', 'left', 'ready', 'right', 'ready', 'home']
        
        self.get_logger().info(f'\nWill move through sequence: {sequence}')
        self.get_logger().info('Press Ctrl+C to abort\n')
        
        time.sleep(2.0)  # Give user time to read
        
        for i, pose_name in enumerate(sequence):
            self.get_logger().info(f'\n[{i+1}/{len(sequence)}] Moving to: {pose_name}')
            success = self.move_to_named_pose(pose_name)
            
            if not success:
                self.get_logger().error(f'Failed to reach pose: {pose_name}')
                break
            
            time.sleep(1.0)  # Brief pause between poses
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Sequence complete!')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    tester = MoveItMotionTester()
    executor = MultiThreadedExecutor()
    executor.add_node(tester)
    
    try:
        # Give some time for connections
        time.sleep(1.0)
        tester.run_demo()
    except KeyboardInterrupt:
        tester.get_logger().info('Interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
