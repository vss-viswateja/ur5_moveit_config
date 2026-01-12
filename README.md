# ur5_moveit_config

MoveIt 2 configuration package for the UR5 robotic arm, providing motion planning, inverse kinematics, collision detection, and trajectory execution capabilities. This package is auto-generated using the MoveIt Setup Assistant and customized for the UR5 assembly model with namespace support for multi-robot scenarios.

> **Note**: This package is part of the work-in-progress CHARS (Collaborative Heterogeneous Autonomous Robot Swarm) architecture.

## Features

- **Complete MoveIt2 Integration**: Motion planning and trajectory execution for UR5 6-DOF arm
- **Namespace Support**: Compatible with namespaced robots (`ur5/`, `robot1/`, etc.)
- **KDL Kinematics Solver**: Fast inverse kinematics using KDL plugin
- **Pre-defined Poses**: Named states (home, ready, up, left, right) for common configurations
- **Collision Detection**: Self-collision and scene collision checking
- **Pilz Industrial Motion**: Cartesian motion planning support
- **RViz Integration**: Interactive motion planning via RViz MoveIt plugin
- **Python Test Scripts**: Programmatic motion testing and validation
- **Controller Integration**: Seamless ros2_control integration with Gazebo

## Package Contents

```
ur5_moveit_config/
├── config/
│   ├── ur5_assembly.srdf                 # Semantic robot description (planning groups, poses)
│   ├── ur5_assembly.urdf.xacro           # Robot URDF used by MoveIt
│   ├── ur5_assembly.ros2_control.xacro   # ros2_control configuration
│   ├── kinematics.yaml                   # IK solver configuration (KDL)
│   ├── joint_limits.yaml                 # Velocity/acceleration limits
│   ├── moveit_controllers.yaml           # Controller interface config
│   ├── ros2_controllers.yaml             # Joint trajectory controller params
│   ├── pilz_cartesian_limits.yaml        # Cartesian motion limits
│   ├── initial_positions.yaml            # Default joint positions
│   └── moveit.rviz                       # RViz configuration with MoveIt plugin
├── launch/
│   ├── demo.launch.py                    # Full demo with fake controllers
│   ├── move_group.launch.py              # MoveIt move_group node only
│   ├── ur_moveit.launch.py               # Custom launch with namespace support
│   ├── moveit_rviz.launch.py             # RViz with MoveIt plugin
│   ├── spawn_controllers.launch.py       # Load ros2_control controllers
│   ├── rsp.launch.py                     # Robot state publisher
│   ├── static_virtual_joint_tfs.launch.py # TF for virtual joints
│   ├── warehouse_db.launch.py            # MongoDB warehouse (scene storage)
│   └── setup_assistant.launch.py         # Re-run MoveIt Setup Assistant
└── scripts/
    └── test_moveit_config.py             # Python test node for motion execution
```

## Planning Groups

### `arm` Planning Group
- **Type**: Serial kinematic chain
- **Joints**: 6 revolute joints
  - `base_link1_joint` - Base rotation
  - `link1_link2p1_joint` - Shoulder
  - `link2p3_link3_joint` - Elbow
  - `link3_link4_joint` - Wrist 1
  - `link4_link5_joint` - Wrist 2
  - `link5_link6_joint` - Wrist 3
- **End Effector**: `link6_1` (with namespace prefix: `ur5/link6_1`)
- **DOF**: 6

## Pre-defined Poses

| Pose Name | Description | Joint Values (radians) |
|-----------|-------------|------------------------|
| **zero** | All joints at zero | `[0, 0, 0, 0, 0, 0]` |
| **vertical** | Arm pointing upward | `[0, 0, 1.57, 1.57, 0, 0]` |
| **home** | Home position | `[0, 0, 0, 0, 0, 0]` |
| **ready** | Ready for operation | `[0, -0.5, 1.0, -0.5, 0, 0]` |
| **up** | Extended upward | `[0, -1.57, 0, 0, 0, 0]` |
| **left** | Rotated left | `[1.57, -0.5, 1.0, -0.5, 0, 0]` |
| **right** | Rotated right | `[-1.57, -0.5, 1.0, -0.5, 0, 0]` |

## Dependencies

### ROS 2 Packages
```bash
# MoveIt2 core
ros-humble-moveit-ros-move-group
ros-humble-moveit-kinematics
ros-humble-moveit-planners
ros-humble-moveit-simple-controller-manager
ros-humble-moveit-ros-visualization
ros-humble-moveit-ros-warehouse
ros-humble-moveit-setup-assistant
ros-humble-moveit-configs-utils

# Control
ros-humble-controller-manager
ros-humble-joint-trajectory-controller
ros-humble-joint-state-publisher
ros-humble-joint-state-publisher-gui

# Core dependencies
ros-humble-robot-state-publisher
ros-humble-tf2-ros
ros-humble-xacro
ros-humble-rviz2
```

### Workspace Packages
- `swarm_description` - UR5 URDF model and meshes

### Installation

1. **Install ROS 2 Humble** (if not already installed):
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

2. **Install MoveIt2 and dependencies**:
```bash
# MoveIt2 complete installation
sudo apt install ros-humble-moveit

# Additional MoveIt packages
sudo apt install ros-humble-moveit-configs-utils \
                 ros-humble-moveit-ros-warehouse \
                 ros-humble-moveit-setup-assistant

# Controllers and simulation
sudo apt install ros-humble-joint-trajectory-controller \
                 ros-humble-controller-manager \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers

# Visualization
sudo apt install ros-humble-rviz2 \
                 ros-humble-rviz-visual-tools

# Kinematics plugins
sudo apt install ros-humble-moveit-kinematics
```

3. **Optional: MongoDB for scene storage**:
```bash
sudo apt install ros-humble-warehouse-ros-mongo
sudo apt install mongodb
```

4. **Clone and build workspace**:
```bash
cd ~/swarm_ws/src
# Clone your repository here

cd ~/swarm_ws
colcon build --packages-select ur5_moveit_config swarm_description
source install/setup.bash
```

## Usage

### Demo Mode (Fake Controllers)

Run the full MoveIt2 demo with simulated controllers (no Gazebo):

```bash
# Default demo with RViz
ros2 launch ur5_moveit_config demo.launch.py

# Plan and execute motions using RViz:
# 1. In RViz, select "MotionPlanning" panel
# 2. Set goal state using interactive marker or "Goal State" dropdown
# 3. Click "Plan" to compute trajectory
# 4. Click "Execute" to run motion (on fake controller)
```

### Simulation Mode (Gazebo + MoveIt)

#### Step 1: Launch UR5 in Gazebo with ros2_control
```bash
# Terminal 1: Start Gazebo simulation with UR5
ros2 launch swarm_description ur5_control.launch.py
```

#### Step 2: Launch MoveIt move_group
```bash
# Terminal 2: Start MoveIt planning node
ros2 launch ur5_moveit_config ur_moveit.launch.py

# Or with custom namespace:
ros2 launch ur5_moveit_config ur_moveit.launch.py robot_namespace:=robot1
```

#### Step 3: Test Motion Programmatically
```bash
# Terminal 3: Send motion commands
# Move to 'ready' pose
ros2 run ur5_moveit_config test_moveit_config --ros-args -p target_pose:=ready

# Move to 'up' pose
ros2 run ur5_moveit_config test_moveit_config --ros-args -p target_pose:=up

# Cartesian goal
ros2 run ur5_moveit_config test_moveit_config --ros-args \
    -p use_cartesian:=true \
    -p target_x:=0.4 \
    -p target_y:=0.2 \
    -p target_z:=0.5

# Plan only (don't execute)
ros2 run ur5_moveit_config test_moveit_config --ros-args \
    -p execute:=false \
    -p target_pose:=left
```

### RViz Motion Planning Interface

```bash
# Launch RViz with MoveIt plugin only
ros2 launch ur5_moveit_config moveit_rviz.launch.py

# Or combined with Gazebo (use swarm_bringup launch):
ros2 launch swarm_bringup single_arm_moveit.launch.py
```

**Interactive Planning Steps**:
1. **Set Start State**: Current robot state (automatic)
2. **Set Goal State**:
   - Drag interactive marker to desired pose
   - OR select named pose from dropdown (home, ready, up, etc.)
   - OR set joint values manually
3. **Plan**: Click "Plan" button to compute trajectory
4. **Visualize**: View planned path in orange
5. **Execute**: Click "Execute" to send trajectory to robot

### Python API Example

Create a custom motion script:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

class UR5MotionPlanner(Node):
    def __init__(self):
        super().__init__('ur5_motion_planner')
        self.client = ActionClient(
            self, 
            MoveGroup, 
            '/ur5/move_action'
        )
        
    def plan_to_pose(self, x, y, z, qx, qy, qz, qw):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        
        # Set goal pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'ur5/base_link'
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        
        # Send goal
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        return future

def main():
    rclpy.init()
    planner = UR5MotionPlanner()
    # Plan to pose at (0.4, 0.0, 0.5)
    future = planner.plan_to_pose(0.4, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0)
    rclpy.spin_until_future_complete(planner, future)
    rclpy.shutdown()
```

### Multi-Robot with Namespaces

```bash
# Robot 1 (namespace: robot1)
# Terminal 1: Simulation
ros2 launch swarm_description ur5_control.launch.py robot_namespace:=robot1

# Terminal 2: MoveIt
ros2 launch ur5_moveit_config ur_moveit.launch.py robot_namespace:=robot1

# Robot 2 (namespace: robot2)
# Terminal 3: Simulation
ros2 launch swarm_description ur5_control.launch.py robot_namespace:=robot2

# Terminal 4: MoveIt
ros2 launch ur5_moveit_config ur_moveit.launch.py robot_namespace:=robot2
```

## Configuration Details

### Kinematics Solver

**Plugin**: `kdl_kinematics_plugin/KDLKinematicsPlugin`

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

**Alternative solvers** (requires additional installation):
- `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` (more accurate, slower)
- `cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin` (faster repeated queries)

### Joint Limits

Default safety scaling for motion planning:

```yaml
default_velocity_scaling_factor: 0.1      # 10% of max velocity
default_acceleration_scaling_factor: 0.1  # 10% of max acceleration

# Per-joint limits
joint_limits:
  base_link1_joint:
    max_velocity: 3.14 rad/s
    max_acceleration: 3.14 rad/s²
```

**Adjust for faster motion**: Increase scaling factors in [joint_limits.yaml](config/joint_limits.yaml) (max: 1.0)

### Motion Planners

Available planners (configured automatically):

- **OMPL** (default):
  - RRTConnect (fastest)
  - RRT
  - RRT*
  - PRM
  - BFMT
  - KPIECE
  - EST

- **Pilz Industrial Motion**:
  - PTP (Point-to-Point)
  - LIN (Linear in Cartesian space)
  - CIRC (Circular)

Select planner in RViz: `Planning Library` dropdown → `OMPL` → Choose algorithm

### Controller Configuration

MoveIt interfaces with ros2_control via `FollowJointTrajectory` action:

```yaml
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
  
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints: [base_link1_joint, link1_link2p1_joint, ...]
```

**Action Server**: `/ur5/arm_controller/follow_joint_trajectory`

## Testing and Validation

### Check MoveIt Configuration

```bash
# Verify move_group is running
ros2 node list | grep move_group

# Check planning groups
ros2 service call /ur5/move_group/get_group_names moveit_msgs/srv/GetPlanningScene "{}"

# List available planners
ros2 param get /ur5/move_group planning_plugin
```

### Test Kinematics

```bash
# Test IK solver service
ros2 service call /ur5/move_group/compute_ik moveit_msgs/srv/GetPositionIK \
    "{ik_request: {group_name: 'arm', pose_stamped: {header: {frame_id: 'ur5/base_link'}, pose: {position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}}}"
```

### Monitor Planning

```bash
# Watch planned trajectories
ros2 topic echo /ur5/move_group/display_planned_path

# Check planning scene
ros2 topic echo /ur5/move_group/monitored_planning_scene

# View controller feedback
ros2 topic echo /ur5/arm_controller/follow_joint_trajectory/feedback
```

### Collision Testing

Add collision objects via RViz or programmatically:

```python
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

# Create box obstacle
collision_object = CollisionObject()
collision_object.header.frame_id = 'ur5/base_link'
collision_object.id = 'box'

primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = [0.1, 0.1, 0.1]

# Publish to planning scene
pub = node.create_publisher(CollisionObject, '/ur5/planning_scene', 10)
pub.publish(collision_object)
```

## Troubleshooting

### MoveIt Planning Fails

**Issue**: No valid motion plan found.

**Solutions**:
1. **Increase planning time**:
   ```bash
   ros2 param set /ur5/move_group allowed_planning_time 10.0
   ```

2. **Try different planner**:
   - In RViz: Switch from RRTConnect to RRT*
   - Check if goal is reachable: `Tools` → `Check State Validity`

3. **Relax constraints**:
   - Increase position/orientation tolerances
   - Disable collision checking temporarily to test reachability

4. **Check joint limits**:
   ```bash
   ros2 topic echo /ur5/joint_states
   # Ensure current state within limits
   ```

### Controller Connection Issues

**Issue**: `MoveIt unable to connect to controller action server`.

**Solutions**:
```bash
# Verify controller is running
ros2 control list_controllers --controller-manager /ur5/controller_manager

# Check if arm_controller is active
ros2 control list_controllers | grep arm_controller

# Manually load and activate
ros2 control load_controller arm_controller --controller-manager /ur5/controller_manager
ros2 control set_controller_state arm_controller active --controller-manager /ur5/controller_manager

# Test action server
ros2 action list | grep follow_joint_trajectory
ros2 action info /ur5/arm_controller/follow_joint_trajectory
```

### IK Solver Failures

**Issue**: `IK solver could not find solution`.

**Causes and fixes**:
1. **Goal pose unreachable**: Check workspace limits
2. **Timeout too short**: Increase in [kinematics.yaml](config/kinematics.yaml):
   ```yaml
   kinematics_solver_timeout: 0.1  # Increase from 0.05
   ```
3. **Collisions**: Disable collision checking in IK request
4. **Singular configuration**: Try different approach angle

### Namespace/TF Issues

**Issue**: `Could not find transform from ur5/base_link to ur5/link6_1`.

**Solutions**:
```bash
# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Verify robot_state_publisher running
ros2 node list | grep robot_state_publisher

# Check virtual joint
ros2 topic echo /tf_static | grep virtual

# Verify SRDF virtual joint matches TF:
# SRDF: parent_frame="ur5/world"
# TF static: should publish ur5/world → base_link
```

### RViz Not Showing Robot

**Issue**: Robot model not visible in RViz.

**Solutions**:
1. **Add RobotModel display**: `Add` → `RobotModel`
2. **Set robot description**: 
   - Robot Description: `robot_description`
   - TF Prefix: (empty or `ur5`)
3. **Check Fixed Frame**: Should be `ur5/base_link` or `world`
4. **Verify robot_description parameter**:
   ```bash
   ros2 param get /ur5/robot_state_publisher robot_description
   ```

## Advanced Usage

### Custom Planning Pipeline

Modify [move_group.launch.py](launch/move_group.launch.py) to use custom planners:

```python
moveit_config = MoveItConfigsBuilder("ur5_assembly") \
    .planning_pipelines(
        pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"]
    ) \
    .to_moveit_configs()
```

### Trajectory Execution Monitoring

Monitor execution progress:

```python
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

client = ActionClient(node, FollowJointTrajectory, 
                      '/ur5/arm_controller/follow_joint_trajectory')

def feedback_callback(feedback_msg):
    print(f"Error: {feedback_msg.feedback.error}")
    print(f"Time from start: {feedback_msg.feedback.actual.time_from_start}")
```

### Scene Database (Warehouse)

Store planning scenes in MongoDB:

```bash
# Start warehouse
ros2 launch ur5_moveit_config warehouse_db.launch.py

# In RViz: Context tab → "Save Scene"
# Reload scenes from database as needed
```

### Motion Primitives

Use Pilz for Cartesian linear motion:

```python
# In Python API or RViz
# Set planner to "PTP" or "LIN"
# LIN ensures straight-line motion in Cartesian space
```

## Development Notes (CHARS)

- **Multi-Robot Planning**: Each robot instance needs separate `move_group` node
- **Shared Scene**: Implement centralized planning scene publisher for swarm coordination
- **Collision Avoidance**: Future work to integrate multi-robot collision checking
- **Formation Control**: Use MoveIt for coordinated multi-arm manipulation

## License

BSD License (MoveIt2 standard)

## Maintainer

**Viswa Teja Bottu**  
Email: vss.viswatejabottu@gmail.com

## Contributing

This package is part of ongoing research in heterogeneous multi-robot systems. For questions or collaboration opportunities, please contact the maintainer.

## See Also

- [swarm_description](../swarm_description/) - UR5 URDF model and robot descriptions
- [swarm_bringup](../swarm_bringup/) - Integrated launch files for UR5 + MoveIt + Gazebo
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
- [ros2_control Documentation](https://control.ros.org/humble/)

## References

- MoveIt Setup Assistant: Used to generate initial configuration
- KDL Kinematics: Orocos Kinematics and Dynamics Library
- OMPL: Open Motion Planning Library
- Pilz Industrial Motion: Industrial-grade trajectory generation
