#!/usr/bin/env python3
"""
Base Coordinates Plotter for Humanoid Robot
Subscribes to MPC observation topic and plots robot base position and orientation.
Saves the plot as an image file.
"""
import rclpy
from rclpy.node import Node
from ocs2_ros2_msgs.msg import MpcObservation, MpcFlattenedController
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import signal
import sys


class BaseCoordinatesPlotter(Node):
    def __init__(self, max_data_points=1000, plot_update_rate=10.0):
        super().__init__("base_coordinates_plotter")
        
        # Data storage for actual state
        self.max_data_points = max_data_points
        self.time_data = deque(maxlen=max_data_points)
        self.base_x = deque(maxlen=max_data_points)
        self.base_y = deque(maxlen=max_data_points)
        self.base_z = deque(maxlen=max_data_points)
        self.base_yaw = deque(maxlen=max_data_points)
        self.base_pitch = deque(maxlen=max_data_points)
        self.base_roll = deque(maxlen=max_data_points)
        
        # Data storage for reference trajectory
        self.ref_base_x = deque(maxlen=max_data_points)
        self.ref_base_y = deque(maxlen=max_data_points)
        self.ref_base_z = deque(maxlen=max_data_points)
        self.ref_base_yaw = deque(maxlen=max_data_points)
        self.ref_base_pitch = deque(maxlen=max_data_points)
        self.ref_base_roll = deque(maxlen=max_data_points)
        
        # Store current reference trajectory
        self.current_ref_trajectory = None
        self.current_ref_times = None
        
        # Setup subscriber
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribe to observation (actual state)
        self.subscription = self.create_subscription(
            MpcObservation,
            "/g1/mpc_observation",  # Primary topic
            self.observation_callback,
            qos_profile,
        )
        
        # Fallback subscription for observation
        self.subscription2 = self.create_subscription(
            MpcObservation,
            "humanoid/mpc_observation",  # Alternative topic name
            self.observation_callback,
            qos_profile,
        )
        
        # Subscribe to policy (for reference trajectory)
        self.policy_subscription = self.create_subscription(
            MpcFlattenedController,
            "/g1/mpc_policy",  # Primary topic
            self.policy_callback,
            qos_profile,
        )
        
        # Fallback subscription for policy
        self.policy_subscription2 = self.create_subscription(
            MpcFlattenedController,
            "humanoid/mpc_policy",  # Alternative topic name
            self.policy_callback,
            qos_profile,
        )
        
        self.get_logger().info("Base Coordinates Plotter started")
        self.get_logger().info("Subscribing to /g1/mpc_observation and /g1/mpc_policy")
        self.get_logger().info("Press Ctrl+C to stop and save plot")
        
        self.start_time = time.time()
        self.last_plot_time = 0.0
        self.plot_interval = 1.0 / plot_update_rate  # Update plot every 0.1 seconds
        
        # Initialize matplotlib figure (single window)
        self.fig = None
        self.axes = None
        self.plot_initialized = False
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        self.shutdown_requested = False
        
    def observation_callback(self, msg):
        """Extract base coordinates from MPC observation message"""
        if len(msg.state.value) < 6:
            return
        
        # State vector structure for whole-body MPC:
        # [q_b_lin (3D), q_b_ang (3D), q_j (N joints), qd_b_lin (3D), qd_b_ang (3D), qd_j (N joints)]
        # q_b_lin = [p_x, p_y, p_z] - base linear position in world frame
        # q_b_ang = [yaw, pitch, roll] - base Euler angles (ZYX convention)
        
        state = np.array(msg.state.value)
        
        if len(state) >= 6:
            # Base linear position (first 3 elements)
            self.base_x.append(state[0])  # p_x
            self.base_y.append(state[1])  # p_y
            self.base_z.append(state[2])  # p_z
            
            # Base angular position (Euler angles, next 3 elements)
            # Note: Convention may be ZYX (yaw, pitch, roll)
            self.base_yaw.append(state[3])    # yaw
            self.base_pitch.append(state[4])  # pitch
            self.base_roll.append(state[5])   # roll
            
            # Time (relative to start)
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            
            # Extract reference trajectory at current time
            self.extract_reference_at_time(msg.time)
            
            # Update plot periodically
            if current_time - self.last_plot_time >= self.plot_interval:
                self.update_plot()
                self.last_plot_time = current_time
    
    def policy_callback(self, msg):
        """Extract reference trajectory from MPC policy message"""
        if len(msg.plan_target_trajectories.state_trajectory) == 0:
            return
        
        # Store reference trajectory
        ref_times = list(msg.plan_target_trajectories.time_trajectory)
        ref_states = []
        
        for state_msg in msg.plan_target_trajectories.state_trajectory:
            if len(state_msg.value) >= 6:
                ref_states.append(np.array(state_msg.value[:6]))  # First 6 elements: [x, y, z, yaw, pitch, roll]
            else:
                return
        
        self.current_ref_trajectory = np.array(ref_states)
        self.current_ref_times = np.array(ref_times)
    
    def extract_reference_at_time(self, current_time):
        """Extract reference values at current time by interpolation"""
        if self.current_ref_trajectory is None or self.current_ref_times is None:
            # No reference available, use zeros or last value
            if len(self.ref_base_x) > 0:
                self.ref_base_x.append(self.ref_base_x[-1])
                self.ref_base_y.append(self.ref_base_y[-1])
                self.ref_base_z.append(self.ref_base_z[-1])
                self.ref_base_yaw.append(self.ref_base_yaw[-1])
                self.ref_base_pitch.append(self.ref_base_pitch[-1])
                self.ref_base_roll.append(self.ref_base_roll[-1])
            else:
                self.ref_base_x.append(0.0)
                self.ref_base_y.append(0.0)
                self.ref_base_z.append(0.0)
                self.ref_base_yaw.append(0.0)
                self.ref_base_pitch.append(0.0)
                self.ref_base_roll.append(0.0)
            return
        
        # Interpolate reference trajectory at current time
        if len(self.current_ref_times) == 0:
            return
        
        # Clamp time to trajectory bounds
        t_min = self.current_ref_times[0]
        t_max = self.current_ref_times[-1]
        t_clamped = max(t_min, min(t_max, current_time))
        
        # Find interpolation indices
        if t_clamped <= t_min:
            idx = 0
            alpha = 0.0
        elif t_clamped >= t_max:
            idx = len(self.current_ref_times) - 2
            alpha = 1.0
        else:
            # Find the segment containing t_clamped
            idx = 0
            for i in range(len(self.current_ref_times) - 1):
                if self.current_ref_times[i] <= t_clamped <= self.current_ref_times[i + 1]:
                    idx = i
                    break
            
            # Linear interpolation
            if self.current_ref_times[idx + 1] != self.current_ref_times[idx]:
                alpha = (t_clamped - self.current_ref_times[idx]) / (self.current_ref_times[idx + 1] - self.current_ref_times[idx])
            else:
                alpha = 0.0
        
        # Interpolate reference state
        if idx < len(self.current_ref_trajectory) - 1:
            ref_state = (1 - alpha) * self.current_ref_trajectory[idx] + alpha * self.current_ref_trajectory[idx + 1]
        else:
            ref_state = self.current_ref_trajectory[idx]
        
        # Store reference values
        self.ref_base_x.append(ref_state[0])
        self.ref_base_y.append(ref_state[1])
        self.ref_base_z.append(ref_state[2])
        self.ref_base_yaw.append(ref_state[3])
        self.ref_base_pitch.append(ref_state[4])
        self.ref_base_roll.append(ref_state[5])
    
    def update_plot(self):
        """Update the live plot"""
        if len(self.time_data) < 2:
            return
        
        # Initialize figure on first call
        if not self.plot_initialized:
            self.fig, self.axes = plt.subplots(2, 3, figsize=(16, 10))
            self.fig.suptitle('Robot Base Coordinates (Actual vs Reference)', fontsize=16, fontweight='bold')
            self.plot_initialized = True
            plt.ion()  # Turn on interactive mode
            plt.show(block=False)
        
        time_array = np.array(self.time_data)
        
        # Clear and replot position with reference
        self.axes[0, 0].clear()
        self.axes[0, 0].plot(time_array, np.array(self.base_x), 'r-', linewidth=2, label='Actual X', alpha=0.8)
        if len(self.ref_base_x) == len(time_array):
            self.axes[0, 0].plot(time_array, np.array(self.ref_base_x), 'r--', linewidth=2, label='Reference X', alpha=0.6)
        self.axes[0, 0].set_title('Base Position X', fontsize=12, fontweight='bold')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Position (m)')
        self.axes[0, 0].grid(True, alpha=0.3)
        self.axes[0, 0].legend()
        
        self.axes[0, 1].clear()
        self.axes[0, 1].plot(time_array, np.array(self.base_y), 'g-', linewidth=2, label='Actual Y', alpha=0.8)
        if len(self.ref_base_y) == len(time_array):
            self.axes[0, 1].plot(time_array, np.array(self.ref_base_y), 'g--', linewidth=2, label='Reference Y', alpha=0.6)
        self.axes[0, 1].set_title('Base Position Y', fontsize=12, fontweight='bold')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Position (m)')
        self.axes[0, 1].grid(True, alpha=0.3)
        self.axes[0, 1].legend()
        
        self.axes[0, 2].clear()
        self.axes[0, 2].plot(time_array, np.array(self.base_z), 'b-', linewidth=2, label='Actual Z', alpha=0.8)
        if len(self.ref_base_z) == len(time_array):
            self.axes[0, 2].plot(time_array, np.array(self.ref_base_z), 'b--', linewidth=2, label='Reference Z', alpha=0.6)
        self.axes[0, 2].set_title('Base Position Z (Height)', fontsize=12, fontweight='bold')
        self.axes[0, 2].set_xlabel('Time (s)')
        self.axes[0, 2].set_ylabel('Position (m)')
        self.axes[0, 2].grid(True, alpha=0.3)
        self.axes[0, 2].legend()
        
        # Clear and replot orientation with reference
        self.axes[1, 0].clear()
        self.axes[1, 0].plot(time_array, np.array(self.base_yaw), 'r-', linewidth=2, label='Actual Yaw', alpha=0.8)
        if len(self.ref_base_yaw) == len(time_array):
            self.axes[1, 0].plot(time_array, np.array(self.ref_base_yaw), 'r--', linewidth=2, label='Reference Yaw', alpha=0.6)
        self.axes[1, 0].set_title('Base Orientation - Yaw', fontsize=12, fontweight='bold')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Angle (rad)')
        self.axes[1, 0].grid(True, alpha=0.3)
        self.axes[1, 0].legend()
        
        self.axes[1, 1].clear()
        self.axes[1, 1].plot(time_array, np.array(self.base_pitch), 'g-', linewidth=2, label='Actual Pitch', alpha=0.8)
        if len(self.ref_base_pitch) == len(time_array):
            self.axes[1, 1].plot(time_array, np.array(self.ref_base_pitch), 'g--', linewidth=2, label='Reference Pitch', alpha=0.6)
        self.axes[1, 1].set_title('Base Orientation - Pitch', fontsize=12, fontweight='bold')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Angle (rad)')
        self.axes[1, 1].grid(True, alpha=0.3)
        self.axes[1, 1].legend()
        
        self.axes[1, 2].clear()
        self.axes[1, 2].plot(time_array, np.array(self.base_roll), 'b-', linewidth=2, label='Actual Roll', alpha=0.8)
        if len(self.ref_base_roll) == len(time_array):
            self.axes[1, 2].plot(time_array, np.array(self.ref_base_roll), 'b--', linewidth=2, label='Reference Roll', alpha=0.6)
        self.axes[1, 2].set_title('Base Orientation - Roll', fontsize=12, fontweight='bold')
        self.axes[1, 2].set_xlabel('Time (s)')
        self.axes[1, 2].set_ylabel('Angle (rad)')
        self.axes[1, 2].grid(True, alpha=0.3)
        self.axes[1, 2].legend()
        
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def save_plot(self, filename=None):
        """Save the final plot to an image file"""
        if len(self.time_data) < 2:
            self.get_logger().warn("Not enough data to save plot")
            return
        
        if filename is None:
            from datetime import datetime
            filename = f"base_coordinates_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        
        # Use existing figure if available, otherwise create new one
        if self.fig is None:
            self.fig, self.axes = plt.subplots(2, 3, figsize=(16, 10))
            self.fig.suptitle('Robot Base Coordinates (Actual vs Reference)', fontsize=16, fontweight='bold')
        
        time_array = np.array(self.time_data)
        
        # Update all subplots with reference
        self.axes[0, 0].clear()
        self.axes[0, 0].plot(time_array, np.array(self.base_x), 'r-', linewidth=2, label='Actual X', alpha=0.8)
        if len(self.ref_base_x) == len(time_array):
            self.axes[0, 0].plot(time_array, np.array(self.ref_base_x), 'r--', linewidth=2, label='Reference X', alpha=0.6)
        self.axes[0, 0].set_title('Base Position X', fontsize=12, fontweight='bold')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Position (m)')
        self.axes[0, 0].grid(True, alpha=0.3)
        self.axes[0, 0].legend()
        
        self.axes[0, 1].clear()
        self.axes[0, 1].plot(time_array, np.array(self.base_y), 'g-', linewidth=2, label='Actual Y', alpha=0.8)
        if len(self.ref_base_y) == len(time_array):
            self.axes[0, 1].plot(time_array, np.array(self.ref_base_y), 'g--', linewidth=2, label='Reference Y', alpha=0.6)
        self.axes[0, 1].set_title('Base Position Y', fontsize=12, fontweight='bold')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Position (m)')
        self.axes[0, 1].grid(True, alpha=0.3)
        self.axes[0, 1].legend()
        
        self.axes[0, 2].clear()
        self.axes[0, 2].plot(time_array, np.array(self.base_z), 'b-', linewidth=2, label='Actual Z', alpha=0.8)
        if len(self.ref_base_z) == len(time_array):
            self.axes[0, 2].plot(time_array, np.array(self.ref_base_z), 'b--', linewidth=2, label='Reference Z', alpha=0.6)
        self.axes[0, 2].set_title('Base Position Z (Height)', fontsize=12, fontweight='bold')
        self.axes[0, 2].set_xlabel('Time (s)')
        self.axes[0, 2].set_ylabel('Position (m)')
        self.axes[0, 2].grid(True, alpha=0.3)
        self.axes[0, 2].legend()
        
        self.axes[1, 0].clear()
        self.axes[1, 0].plot(time_array, np.array(self.base_yaw), 'r-', linewidth=2, label='Actual Yaw', alpha=0.8)
        if len(self.ref_base_yaw) == len(time_array):
            self.axes[1, 0].plot(time_array, np.array(self.ref_base_yaw), 'r--', linewidth=2, label='Reference Yaw', alpha=0.6)
        self.axes[1, 0].set_title('Base Orientation - Yaw', fontsize=12, fontweight='bold')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Angle (rad)')
        self.axes[1, 0].grid(True, alpha=0.3)
        self.axes[1, 0].legend()
        
        self.axes[1, 1].clear()
        self.axes[1, 1].plot(time_array, np.array(self.base_pitch), 'g-', linewidth=2, label='Actual Pitch', alpha=0.8)
        if len(self.ref_base_pitch) == len(time_array):
            self.axes[1, 1].plot(time_array, np.array(self.ref_base_pitch), 'g--', linewidth=2, label='Reference Pitch', alpha=0.6)
        self.axes[1, 1].set_title('Base Orientation - Pitch', fontsize=12, fontweight='bold')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Angle (rad)')
        self.axes[1, 1].grid(True, alpha=0.3)
        self.axes[1, 1].legend()
        
        self.axes[1, 2].clear()
        self.axes[1, 2].plot(time_array, np.array(self.base_roll), 'b-', linewidth=2, label='Actual Roll', alpha=0.8)
        if len(self.ref_base_roll) == len(time_array):
            self.axes[1, 2].plot(time_array, np.array(self.ref_base_roll), 'b--', linewidth=2, label='Reference Roll', alpha=0.6)
        self.axes[1, 2].set_title('Base Orientation - Roll', fontsize=12, fontweight='bold')
        self.axes[1, 2].set_xlabel('Time (s)')
        self.axes[1, 2].set_ylabel('Angle (rad)')
        self.axes[1, 2].grid(True, alpha=0.3)
        self.axes[1, 2].legend()
        
        self.fig.tight_layout()
        self.fig.savefig(filename, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"Plot saved to: {filename}")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info("Shutdown requested, saving plot...")
        self.shutdown_requested = True
        self.save_plot()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    
    node = BaseCoordinatesPlotter()
    
    try:
        # Spin with timeout to allow plot updates
        while not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_plot()
        if node.fig is not None:
            plt.close(node.fig)
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()


if __name__ == "__main__":
    main()

