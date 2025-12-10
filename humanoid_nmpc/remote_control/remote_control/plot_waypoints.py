#!/usr/bin/env python3
"""
Waypoint Plotter for Humanoid Robot
Subscribes to waypoint commands and MPC observation to plot waypoint targets and current position.
Saves the plot as an image file.
"""
import rclpy
from rclpy.node import Node
from humanoid_mpc_msgs.msg import WaypointCommand
from ocs2_ros2_msgs.msg import MpcObservation
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import signal
import sys


class WaypointPlotter(Node):
    def __init__(self, max_data_points=1000, plot_update_rate=10.0):
        super().__init__("waypoint_plotter")
        
        # Data storage for current position
        self.max_data_points = max_data_points
        self.time_data = deque(maxlen=max_data_points)
        self.current_x = deque(maxlen=max_data_points)
        self.current_y = deque(maxlen=max_data_points)
        self.current_z = deque(maxlen=max_data_points)
        self.current_yaw = deque(maxlen=max_data_points)
        
        # Data storage for waypoint targets
        self.waypoint_x = deque(maxlen=max_data_points)
        self.waypoint_y = deque(maxlen=max_data_points)
        self.waypoint_z = deque(maxlen=max_data_points)
        self.waypoint_yaw = deque(maxlen=max_data_points)
        self.waypoint_times = deque(maxlen=max_data_points)  # When waypoint was received
        
        # Track active waypoint
        self.active_waypoint = None
        self.waypoint_start_time = None
        
        # Setup subscribers
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribe to waypoint commands
        self.waypoint_subscription = self.create_subscription(
            WaypointCommand,
            "/humanoid/waypoint_command",
            self.waypoint_callback,
            qos_profile,
        )
        
        # Subscribe to observation (current position)
        self.observation_subscription = self.create_subscription(
            MpcObservation,
            "/g1/mpc_observation",
            self.observation_callback,
            qos_profile,
        )
        
        # Fallback subscription
        self.observation_subscription2 = self.create_subscription(
            MpcObservation,
            "humanoid/mpc_observation",
            self.observation_callback,
            qos_profile,
        )
        
        self.get_logger().info("Waypoint Plotter started")
        self.get_logger().info("Subscribing to /humanoid/waypoint_command and /g1/mpc_observation")
        self.get_logger().info("Press Ctrl+C to stop and save plot")
        
        self.start_time = time.time()
        self.last_plot_time = 0.0
        self.plot_interval = 1.0 / plot_update_rate
        
        # Initialize matplotlib figure (single window)
        self.fig = None
        self.axes = None
        self.plot_initialized = False
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        self.shutdown_requested = False
        
    def waypoint_callback(self, msg):
        """Handle waypoint command messages"""
        current_time = time.time() - self.start_time
        
        # Store waypoint target (absolute coordinates)
        # Note: waypoint commands are relative, but we'll compute absolute target
        # based on current position when waypoint is received
        self.active_waypoint = {
            'target_x': msg.target_x,
            'target_y': msg.target_y,
            'target_z': msg.target_z,
            'target_yaw': msg.target_yaw,
            'time_to_reach': msg.time_to_reach,
            'motion_type': msg.motion_type,
            'received_time': current_time
        }
        
        # If we have current position, compute absolute target
        if len(self.current_x) > 0:
            # Waypoint is relative to current position
            abs_x = self.current_x[-1] + msg.target_x
            abs_y = self.current_y[-1] + msg.target_y
            abs_z = msg.target_z  # Height is absolute
            abs_yaw = self.current_yaw[-1] + msg.target_yaw
            
            self.waypoint_x.append(abs_x)
            self.waypoint_y.append(abs_y)
            self.waypoint_z.append(abs_z)
            self.waypoint_yaw.append(abs_yaw)
            self.waypoint_times.append(current_time)
            
            self.get_logger().info(
                f"Waypoint received: target=({abs_x:.2f}, {abs_y:.2f}, {abs_z:.2f}), "
                f"yaw={abs_yaw:.2f}, time_to_reach={msg.time_to_reach:.2f}s"
            )
    
    def observation_callback(self, msg):
        """Extract current position from MPC observation message"""
        if len(msg.state.value) < 6:
            return
        
        state = np.array(msg.state.value)
        
        # Extract current position
        current_x = state[0]
        current_y = state[1]
        current_z = state[2]
        current_yaw = state[3]  # yaw
        
        self.current_x.append(current_x)
        self.current_y.append(current_y)
        self.current_z.append(current_z)
        self.current_yaw.append(current_yaw)
        
        # Time (relative to start)
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        
        # If we have an active waypoint but haven't computed absolute target yet
        if self.active_waypoint is not None and len(self.waypoint_x) == 0:
            abs_x = current_x + self.active_waypoint['target_x']
            abs_y = current_y + self.active_waypoint['target_y']
            abs_z = self.active_waypoint['target_z']
            abs_yaw = current_yaw + self.active_waypoint['target_yaw']
            
            self.waypoint_x.append(abs_x)
            self.waypoint_y.append(abs_y)
            self.waypoint_z.append(abs_z)
            self.waypoint_yaw.append(abs_yaw)
            self.waypoint_times.append(current_time)
        
        # Update plot periodically
        if current_time - self.last_plot_time >= self.plot_interval:
            self.update_plot()
            self.last_plot_time = current_time
    
    def update_plot(self):
        """Update the live plot"""
        if len(self.time_data) < 2:
            return
        
        # Initialize figure on first call
        if not self.plot_initialized:
            self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
            self.fig.suptitle('Waypoint Tracking', fontsize=16, fontweight='bold')
            self.plot_initialized = True
            plt.ion()  # Turn on interactive mode
            plt.show(block=False)
        
        time_array = np.array(self.time_data)
        
        # Plot position X
        self.axes[0, 0].clear()
        self.axes[0, 0].plot(time_array, np.array(self.current_x), 'b-', linewidth=2, label='Current X', alpha=0.8)
        if len(self.waypoint_x) > 0:
            waypoint_time_array = np.array(self.waypoint_times)
            self.axes[0, 0].scatter(waypoint_time_array, np.array(self.waypoint_x), 
                                   color='red', s=100, marker='*', label='Waypoint X', zorder=5)
        self.axes[0, 0].set_title('Position X', fontsize=12, fontweight='bold')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Position (m)')
        self.axes[0, 0].grid(True, alpha=0.3)
        self.axes[0, 0].legend()
        
        # Plot position Y
        self.axes[0, 1].clear()
        self.axes[0, 1].plot(time_array, np.array(self.current_y), 'g-', linewidth=2, label='Current Y', alpha=0.8)
        if len(self.waypoint_y) > 0:
            waypoint_time_array = np.array(self.waypoint_times)
            self.axes[0, 1].scatter(waypoint_time_array, np.array(self.waypoint_y), 
                                   color='red', s=100, marker='*', label='Waypoint Y', zorder=5)
        self.axes[0, 1].set_title('Position Y', fontsize=12, fontweight='bold')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Position (m)')
        self.axes[0, 1].grid(True, alpha=0.3)
        self.axes[0, 1].legend()
        
        # Plot position Z (height)
        self.axes[1, 0].clear()
        self.axes[1, 0].plot(time_array, np.array(self.current_z), 'b-', linewidth=2, label='Current Z', alpha=0.8)
        if len(self.waypoint_z) > 0:
            waypoint_time_array = np.array(self.waypoint_times)
            self.axes[1, 0].scatter(waypoint_time_array, np.array(self.waypoint_z), 
                                   color='red', s=100, marker='*', label='Waypoint Z', zorder=5)
        self.axes[1, 0].set_title('Position Z (Height)', fontsize=12, fontweight='bold')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Position (m)')
        self.axes[1, 0].grid(True, alpha=0.3)
        self.axes[1, 0].legend()
        
        # Plot Yaw
        self.axes[1, 1].clear()
        self.axes[1, 1].plot(time_array, np.array(self.current_yaw), 'b-', linewidth=2, label='Current Yaw', alpha=0.8)
        if len(self.waypoint_yaw) > 0:
            waypoint_time_array = np.array(self.waypoint_times)
            self.axes[1, 1].scatter(waypoint_time_array, np.array(self.waypoint_yaw), 
                                   color='red', s=100, marker='*', label='Waypoint Yaw', zorder=5)
        self.axes[1, 1].set_title('Orientation Yaw', fontsize=12, fontweight='bold')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Angle (rad)')
        self.axes[1, 1].grid(True, alpha=0.3)
        self.axes[1, 1].legend()
        
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
            filename = f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        
        # Use existing figure if available, otherwise create new one
        if self.fig is None:
            self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
            self.fig.suptitle('Waypoint Tracking', fontsize=16, fontweight='bold')
        
        time_array = np.array(self.time_data)
        
        # Update all subplots
        self.axes[0, 0].clear()
        self.axes[0, 0].plot(time_array, np.array(self.current_x), 'b-', linewidth=2, label='Current X', alpha=0.8)
        if len(self.waypoint_x) > 0:
            waypoint_time_array = np.array(self.waypoint_times)
            self.axes[0, 0].scatter(waypoint_time_array, np.array(self.waypoint_x), 
                                   color='red', s=100, marker='*', label='Waypoint X', zorder=5)
        self.axes[0, 0].set_title('Position X', fontsize=12, fontweight='bold')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Position (m)')
        self.axes[0, 0].grid(True, alpha=0.3)
        self.axes[0, 0].legend()
        
        self.axes[0, 1].clear()
        self.axes[0, 1].plot(time_array, np.array(self.current_y), 'g-', linewidth=2, label='Current Y', alpha=0.8)
        if len(self.waypoint_y) > 0:
            waypoint_time_array = np.array(self.waypoint_times)
            self.axes[0, 1].scatter(waypoint_time_array, np.array(self.waypoint_y), 
                                   color='red', s=100, marker='*', label='Waypoint Y', zorder=5)
        self.axes[0, 1].set_title('Position Y', fontsize=12, fontweight='bold')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Position (m)')
        self.axes[0, 1].grid(True, alpha=0.3)
        self.axes[0, 1].legend()
        
        self.axes[1, 0].clear()
        self.axes[1, 0].plot(time_array, np.array(self.current_z), 'b-', linewidth=2, label='Current Z', alpha=0.8)
        if len(self.waypoint_z) > 0:
            waypoint_time_array = np.array(self.waypoint_times)
            self.axes[1, 0].scatter(waypoint_time_array, np.array(self.waypoint_z), 
                                   color='red', s=100, marker='*', label='Waypoint Z', zorder=5)
        self.axes[1, 0].set_title('Position Z (Height)', fontsize=12, fontweight='bold')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Position (m)')
        self.axes[1, 0].grid(True, alpha=0.3)
        self.axes[1, 0].legend()
        
        self.axes[1, 1].clear()
        self.axes[1, 1].plot(time_array, np.array(self.current_yaw), 'b-', linewidth=2, label='Current Yaw', alpha=0.8)
        if len(self.waypoint_yaw) > 0:
            waypoint_time_array = np.array(self.waypoint_times)
            self.axes[1, 1].scatter(waypoint_time_array, np.array(self.waypoint_yaw), 
                                   color='red', s=100, marker='*', label='Waypoint Yaw', zorder=5)
        self.axes[1, 1].set_title('Orientation Yaw', fontsize=12, fontweight='bold')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Angle (rad)')
        self.axes[1, 1].grid(True, alpha=0.3)
        self.axes[1, 1].legend()
        
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
    
    node = WaypointPlotter()
    
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

