# Modeling and Phasing in Humanoid Whole-Body MPC

## Table of Contents
1. [Robot Modeling](#robot-modeling)
2. [Phasing System](#phasing-system)
3. [How to Run Walking Commands](#how-to-run-walking-commands)

---

## Robot Modeling

### 1. State Vector Representation

The whole-body MPC uses a **full-order dynamic model** where the state vector `x` contains:

```
x = [q_b_lin, q_b_ang, q_j, qd_b_lin, qd_b_ang, qd_j]^T
```

Where:
- **`q_b_lin`** (3D): Base linear position `[p_x, p_y, p_z]` in world frame
- **`q_b_ang`** (3D): Base Euler angles `[yaw, pitch, roll]` (ZYX convention)
- **`q_j`** (N joints): Joint angles for all MPC-controlled joints
- **`qd_b_lin`** (3D): Base linear velocity `[v_x, v_y, v_z]` in world frame
- **`qd_b_ang`** (3D): Base Euler angle derivatives (not angular velocity!)
- **`qd_j`** (N joints): Joint velocities

**Total state dimension**: `2 × (6 + N_joints)`

### 2. Input Vector Representation

The input vector `u` contains:

```
u = [W_l, W_r, qdd_j]^T
```

Where:
- **`W_l`** (6D): Left foot contact wrench `[f_x, f_y, f_z, M_x, M_y, M_z]` in inertial frame
- **`W_r`** (6D): Right foot contact wrench `[f_x, f_y, f_z, M_x, M_y, M_z]` in inertial frame
- **`qdd_j`** (N joints): Joint accelerations

**Total input dimension**: `6 × N_CONTACTS + N_joints` (typically `12 + N_joints` for 2 feet)

### 3. Dynamics Model

The system uses **acceleration-based dynamics** (`WBAccelDynamicsAD`):

```
ẋ = f(x, u, mode)
```

The dynamics are:
- **Mode-dependent**: Different equations apply based on which feet are in contact
- **Automatic Differentiation (AD)**: Uses CppAD for efficient gradient computation
- **Full rigid-body dynamics**: Computed using Pinocchio library

For each contact mode:
- **Stance foot**: Zero velocity constraint (foot fixed to ground)
- **Swing foot**: Free motion (no contact forces)
- **Flying phase**: No contact forces on either foot

### 4. Constraints

The MPC enforces several constraint types:

#### Equality Constraints (Hard Constraints)
- **Zero wrench constraint**: For swing feet, contact wrench must be zero
- **Zero velocity constraint**: Stance feet must have zero velocity
- **Normal velocity constraint**: Feet cannot penetrate ground
- **Joint mimic constraints**: For coupled joints (e.g., knee joints)

#### Soft Constraints (Penalized in Cost)
- **Joint limits**: Joint angles and velocities within bounds
- **Foot collision**: Feet cannot collide with each other
- **Friction cone**: Contact forces must satisfy friction constraints
- **Contact moment XY**: Moments in X and Y directions are limited

#### Mode-Dependent Constraints
- **STANCE mode**: Both feet fixed, full contact forces allowed
- **LF mode**: Left foot fixed, right foot free (zero wrench)
- **RF mode**: Right foot fixed, left foot free (zero wrench)
- **FLY mode**: Both feet free (zero wrench on both)

---

## Phasing System

### 1. Contact Modes

The system defines **4 contact modes** for a bipedal robot:

```cpp
enum ModeNumber {
  FLY = 0,      // No feet in contact (aerial phase)
  RF = 1,       // Right foot only in contact
  LF = 2,       // Left foot only in contact
  STANCE = 3    // Both feet in contact
};
```

Each mode determines:
- Which feet can apply contact forces
- Which constraints are active
- The dynamics equations used

### 2. Mode Sequence Template

A **ModeSequenceTemplate** defines a periodic sequence of contact modes:

```cpp
struct ModeSequenceTemplate {
  std::vector<scalar_t> switchingTimes;  // [t0=0, t1, t2, ..., tN=T]
  std::vector<size_t> modeSequence;      // [mode0, mode1, ..., modeN-1]
};
```

**Example - Walking Gait**:
```
switchingTimes: [0.0, 0.6, 0.7, 1.3, 1.4]
modeSequence:   [LF, STANCE, RF, STANCE]
```

This means:
- `[0.0, 0.6)`: Left foot swing, right foot stance
- `[0.6, 0.7)`: Both feet stance (double support)
- `[0.7, 1.3)`: Right foot swing, left foot stance
- `[1.3, 1.4)`: Both feet stance (double support)
- Cycle repeats at `t = 1.4`

### 3. Gait Definition

A **Gait** is a phase-parameterized version of a mode sequence:

```cpp
struct Gait {
  scalar_t duration;                    // Total cycle time
  std::vector<scalar_t> eventPhases;    // Phase values (0.0 to 1.0) where mode switches
  std::vector<size_t> modeSequence;     // Sequence of modes
};
```

**Phase variable**: `φ ∈ [0.0, 1.0)` parameterizes the gait cycle
- `φ = 0.0` is the start of the cycle
- `φ = 1.0` wraps back to `0.0`

**Conversion**: `t = φ × duration`

### 4. Gait Schedule

The **GaitSchedule** manages the mode schedule over time:

```cpp
class GaitSchedule {
  ModeSchedule getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime);
  void insertModeSequenceTemplate(const ModeSequenceTemplate& template, 
                                   scalar_t startTime, scalar_t finalTime);
};
```

**Key Functions**:
1. **`getModeSchedule`**: Returns the mode schedule for the MPC horizon
2. **`insertModeSequenceTemplate`**: Inserts a new gait pattern at a specific time
3. **`tileModeSequenceTemplate`**: Repeats a template to fill the horizon

### 5. Available Gaits

From `gait.info`, the system supports:

| Gait Name | Mode Sequence | Duration | Description |
|-----------|--------------|----------|-------------|
| `stance` | `[STANCE]` | 0.5s | Both feet on ground |
| `walk` | `[LF, STANCE, RF, STANCE]` | 1.4s | Normal walking |
| `slow_walk` | `[LF, STANCE, RF, STANCE]` | 1.7s | Slower walking |
| `fast_walk` | `[LF, STANCE, RF, STANCE]` | 1.0s | Faster walking |
| `trot` | `[LF, RF]` | 1.0s | Alternating feet (no double support) |
| `slow_trot` | `[LF, RF]` | 1.1s | Slower trot |
| `fast_trot` | `[LF, RF]` | 0.8s | Faster trot |
| `run` | `[LF, FLY, RF, FLY]` | 0.8s | Running with aerial phases |
| `jump` | `[STANCE, FLY]` | 0.5s | Jumping motion |

### 6. How Phasing Works in MPC

1. **Reference Manager** (`ProceduralMpcMotionManager`):
   - Receives velocity commands or waypoint commands
   - Selects appropriate gait based on desired velocity
   - Updates `GaitSchedule` with the selected gait

2. **MPC Solver**:
   - Queries `GaitSchedule` for mode schedule over horizon `[t, t+T]`
   - For each time step, determines active mode
   - Applies mode-specific dynamics and constraints
   - Optimizes trajectory respecting mode transitions

3. **Mode Transitions**:
   - **Event times**: Exact moments when mode switches occur
   - **Continuity**: State must be continuous across mode switches
   - **Constraints**: Different constraint sets active in each mode

### 7. Gait Selection Logic

The system automatically selects gaits based on desired velocity:

```cpp
// From Ros2ProceduralMpcMotionManager
scalar_t linearVelMag = sqrt(vx² + vy²);

if (linearVelMag > 0.8) {
  gait = "trot";           // Fast forward motion
} else if (linearVelMag > 0.65) {
  gait = "slow_trot";
} else if (linearVelMag > 0.45) {
  gait = "slower_trot";
} else if (linearVelMag > 0.25) {
  gait = "walk";           // Normal walking
} else if (linearVelMag > 0.1) {
  gait = "slow_walk";      // Slow walking
} else {
  gait = "stance";         // Standing still
}
```

---

## How to Run Walking Commands

### Method 1: Keyboard Control

**Command**:
```bash
ros2 run remote_control keyboard_walking_command_publisher
```

**Controls**:
- **↑ Arrow**: Forward (`v_x = +1.0`)
- **↓ Arrow**: Backward (`v_x = -1.0`)
- **← Arrow**: Left (`v_y = +1.0`)
- **→ Arrow**: Right (`v_y = -1.0`)
- **`a`**: Rotate left (`v_yaw = +1.0 rad/s`)
- **`d`**: Rotate right (`v_yaw = -1.0 rad/s`)
- **`w`**: Increase pelvis height
- **`s`**: Decrease pelvis height
- **No key pressed**: Stop (all velocities = 0)

**Topic**: `/humanoid/walking_velocity_command`

### Method 2: GUI (Graphical Interface)

**Command**:
```bash
ros2 run remote_control base_velocity_controller_gui
```

**Features**:
- Sliders for velocity commands
- Real-time visualization
- Pelvis height control
- Gait selection (if implemented)

### Method 3: Xbox Controller

**Command**:
```bash
ros2 run remote_control xbox_walking_command_publisher
```

**Requirements**: Xbox controller connected via USB

### Method 4: Direct ROS 2 Topic Publishing

**Publish a single command**:
```bash
ros2 topic pub --once /humanoid/walking_velocity_command \
  humanoid_mpc_msgs/msg/WalkingVelocityCommand \
  "{linear_velocity_x: 0.5, linear_velocity_y: 0.0, desired_pelvis_height: 0.8, angular_velocity_z: 0.0}"
```

**Publish continuously** (25 Hz):
```bash
ros2 topic pub -r 25 /humanoid/walking_velocity_command \
  humanoid_mpc_msgs/msg/WalkingVelocityCommand \
  "{linear_velocity_x: 0.5, linear_velocity_y: 0.0, desired_pelvis_height: 0.8, angular_velocity_z: 0.0}"
```

### Method 5: Python Script

Create a Python script:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from humanoid_mpc_msgs.msg import WalkingVelocityCommand

class WalkingPublisher(Node):
    def __init__(self):
        super().__init__('walking_publisher')
        self.pub = self.create_publisher(
            WalkingVelocityCommand,
            '/humanoid/walking_velocity_command',
            10
        )
        self.timer = self.create_timer(0.04, self.publish)  # 25 Hz
    
    def publish(self):
        msg = WalkingVelocityCommand()
        msg.linear_velocity_x = 0.5  # Forward
        msg.linear_velocity_y = 0.0
        msg.desired_pelvis_height = 0.8
        msg.angular_velocity_z = 0.0
        self.pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    node = WalkingPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Method 6: Waypoint Following

**Publish a waypoint command**:
```bash
ros2 topic pub --once /humanoid/waypoint_command \
  humanoid_mpc_msgs/msg/WaypointCommand \
  "{target_x: 1.0, target_y: 0.0, target_z: 0.85, target_yaw: 0.0, time_to_reach: 0.0, motion_type: 'waypoint'}"
```

**Note**: 
- `target_x`, `target_y`, `target_yaw` are **relative** to current position
- `target_z` is **absolute** height
- `time_to_reach = 0.0` means auto-estimate

### Message Format

**WalkingVelocityCommand**:
```idl
struct WalkingVelocityCommand {
  double linear_velocity_x;      // [-1.0, 1.0] normalized forward velocity
  double linear_velocity_y;      // [-1.0, 1.0] normalized lateral velocity
  double desired_pelvis_height;  // [0.2, 1.0] meters, absolute height
  double angular_velocity_z;     // [-1.0, 1.0] normalized yaw velocity (rad/s)
};
```

**Values are normalized** to `[-1.0, 1.0]` and scaled by maximum velocities defined in the MPC configuration.

### Checking Robot State

**View current robot position**:
```bash
ros2 topic echo /g1/mpc_observation --once
```

**Quick position check** (Python script):
```bash
ros2 run remote_control check_position
```

### Stopping the Robot

**Method 1**: Stop velocity commands
```bash
ros2 topic pub --once /humanoid/walking_velocity_command \
  humanoid_mpc_msgs/msg/WalkingVelocityCommand \
  "{linear_velocity_x: 0.0, linear_velocity_y: 0.0, desired_pelvis_height: 0.8, angular_velocity_z: 0.0}"
```

**Method 2**: Publish waypoint at current position
```bash
ros2 topic pub --once /humanoid/waypoint_command \
  humanoid_mpc_msgs/msg/WaypointCommand \
  "{target_x: 0.0, target_y: 0.0, target_z: 0.85, target_yaw: 0.0, time_to_reach: 0.0, motion_type: 'waypoint'}"
```

---

## Summary

### Modeling
- **State**: Full pose + velocities (base + joints)
- **Input**: Contact wrenches + joint accelerations
- **Dynamics**: Mode-dependent rigid-body dynamics
- **Constraints**: Mode-specific (stance/swing/fly)

### Phasing
- **Modes**: FLY, LF, RF, STANCE
- **Gait**: Periodic sequence of modes with timing
- **Schedule**: Manages mode transitions over MPC horizon
- **Selection**: Automatic based on desired velocity

### Commands
- **Velocity commands**: Direct control via `/humanoid/walking_velocity_command`
- **Waypoint commands**: Goal-based control via `/humanoid/waypoint_command`
- **Multiple interfaces**: Keyboard, GUI, Xbox, Python, ROS topics

