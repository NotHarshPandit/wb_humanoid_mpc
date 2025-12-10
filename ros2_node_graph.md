# ROS 2 Node Graph - Message Flow

## Main Control Loop

```
┌─────────────────────────────────────────────────────────────┐
│                    External Commands                         │
│  (waypoint_publisher or keyboard commands)                  │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
        ┌──────────────────────────────┐
        │  /humanoid/waypoint_command  │
        │  /humanoid/walking_velocity  │
        └──────────────┬───────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────────┐
│         /humanoid_wb_mpc_sqp_node                           │
│  (MPC Solver - Sequential Quadratic Programming)             │
│                                                              │
│  SUBSCRIBES:                                                 │
│    • /g1/mpc_observation                                    │
│    • /humanoid/walking_velocity_command                      │
│    • /humanoid/waypoint_command                              │
│                                                              │
│  PUBLISHES:                                                  │
│    • /g1/mpc_policy (MpcFlattenedController)                │
└──────────────────────┬───────────────────────────────────────┘
                       │
                       │ /g1/mpc_policy
                       │ (control commands)
                       ▼
┌──────────────────────────────────────────────────────────────┐
│      /humanoid_wb_mpc_dummy_sim_node                        │
│  (Robot Simulation/State Publisher)                          │
│                                                              │
│  SUBSCRIBES:                                                 │
│    • /g1/mpc_policy                                          │
│                                                              │
│  PUBLISHES:                                                  │
│    • /g1/mpc_observation (MpcObservation) ◄──┐             │
│    • /joint_states                             │             │
│    • /terminal_state/joint_states              │             │
│    • /terminal_target/joint_states            │             │
│    • /tf (transform tree)                    │             │
│    • /cartesian_markers                       │             │
│    • /collision_markers                       │             │
│    • /optimized_state_markers                │             │
└──────────────────────┬───────────────────────────────────────┘
                       │
                       │ /g1/mpc_observation
                       │ (robot state feedback)
                       │
                       └──────────────────┐
                                          │
                                          ▼
                          [LOOP BACK TO SQP NODE]
```

## Visualization and State Publishing

```
┌──────────────────────────────────────────────────────────────┐
│      /humanoid_wb_mpc_dummy_sim_node                        │
│                                                              │
│  PUBLISHES:                                                  │
│    • /joint_states ──────────────┐                          │
│    • /terminal_state/joint_states│                          │
│    • /terminal_target/joint_states│                          │
└──────────────────────────────────┼──────────────────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
┌──────────────────┐  ┌──────────────────────────┐  ┌──────────────────────────┐
│/robot_state_     │  │/terminal_robot_state_     │  │/target_robot_state_       │
│publisher         │  │publisher                  │  │publisher                  │
│                  │  │                          │  │                          │
│Publishes TF      │  │Publishes TF with          │  │Publishes TF with          │
│for robot model   │  │"terminal_state/" prefix  │  │"terminal_target/" prefix │
└──────────────────┘  └──────────────────────────┘  └──────────────────────────┘
        │                          │                          │
        └──────────────────────────┼──────────────────────────┘
                                   │
                                   ▼
                            /tf, /tf_static
                            (Transform Tree)
                                   │
                                   ▼
                            /rviz2 (Visualization)
```

## Complete Message Flow Summary

### Control Loop (Main):
1. **External Commands** → `/humanoid/waypoint_command` or `/humanoid/walking_velocity_command`
2. **SQP Node** subscribes to commands + `/g1/mpc_observation`
3. **SQP Node** publishes → `/g1/mpc_policy` (control commands)
4. **Dummy Sim Node** subscribes to `/g1/mpc_policy`
5. **Dummy Sim Node** publishes → `/g1/mpc_observation` (robot state)
6. **Loop back to step 2**

### Visualization:
- **Dummy Sim Node** → `/joint_states` → **Robot State Publishers** → `/tf` → **RViz2**
- **Dummy Sim Node** → `/cartesian_markers`, `/collision_markers`, `/optimized_state_markers` → **RViz2**

### Key Topics:
- `/g1/mpc_policy`: Control commands from MPC solver to robot
- `/g1/mpc_observation`: Robot state feedback from simulation to MPC solver
- `/humanoid/walking_velocity_command`: Velocity commands (linear/angular)
- `/humanoid/waypoint_command`: Waypoint/target pose commands
- `/joint_states`: Current joint positions/velocities
- `/tf`: Transform tree for robot kinematics

