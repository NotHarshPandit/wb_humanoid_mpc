#!/usr/bin/env python3
"""
Contact Force Plotter for Humanoid Robot
Subscribes to MPC policy topic and plots contact forces for both legs.
Saves the plot as an image file.
"""
import rclpy
from rclpy.node import Node
from ocs2_ros2_msgs.msg import MpcFlattenedController
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import signal
import sys


class ContactForcePlotter(Node):
    def __init__(self, max_data_points=1000, plot_update_rate=10.0):
        super().__init__("contact_force_plotter")
        
        # Data storage
        self.max_data_points = max_data_points
        self.time_data = deque(maxlen=max_data_points)
        self.left_force_x = deque(maxlen=max_data_points)
        self.left_force_y = deque(maxlen=max_data_points)
        self.left_force_z = deque(maxlen=max_data_points)
        self.right_force_x = deque(maxlen=max_data_points)
        self.right_force_y = deque(maxlen=max_data_points)
        self.right_force_z = deque(maxlen=max_data_points)
        
        # Setup subscriber
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Try different topic names
        self.subscription = self.create_subscription(
            MpcFlattenedController,
            "/g1/mpc_policy",  # Primary topic
            self.policy_callback,
            qos_profile,
        )
        
        # Fallback subscription
        self.subscription2 = self.create_subscription(
            MpcFlattenedController,
            "humanoid/mpc_policy",  # Alternative topic name
            self.policy_callback,
            qos_profile,
        )
        
        self.get_logger().info("Contact Force Plotter started")
        self.get_logger().info("Subscribing to /g1/mpc_policy and humanoid/mpc_policy")
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
        
    def policy_callback(self, msg):
        """Extract contact forces from MPC policy message"""
        if len(msg.input_trajectory) == 0:
            return
            
        # Get the first input in the trajectory (current command)
        current_input = np.array(msg.input_trajectory[0].value)
        
        # Input structure: [W_l (6D), W_r (6D), qdd_j ...]
        # W_l = [f_x, f_y, f_z, M_x, M_y, M_z] for left foot
        # W_r = [f_x, f_y, f_z, M_x, M_y, M_z] for right foot
        
        if len(current_input) >= 12:
            # Left foot contact force (first 3 elements)
            self.left_force_x.append(current_input[0])
            self.left_force_y.append(current_input[1])
            self.left_force_z.append(current_input[2])
            
            # Right foot contact force (elements 6-8)
            self.right_force_x.append(current_input[6])
            self.right_force_y.append(current_input[7])
            self.right_force_z.append(current_input[8])
            
            # Time (relative to start)
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            
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
            self.fig, self.axes = plt.subplots(3, 2, figsize=(14, 10))
            self.fig.suptitle('Contact Forces for Both Legs', fontsize=16, fontweight='bold')
            self.plot_initialized = True
            plt.ion()  # Turn on interactive mode
            plt.show(block=False)
        
        time_array = np.array(self.time_data)
        
        # Clear and replot left leg forces
        self.axes[0, 0].clear()
        self.axes[0, 0].plot(time_array, np.array(self.left_force_x), 'r-', linewidth=2, label='F_x')
        self.axes[0, 0].set_title('Left Leg - Force X', fontsize=12, fontweight='bold')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Force (N)')
        self.axes[0, 0].grid(True, alpha=0.3)
        self.axes[0, 0].legend()
        
        self.axes[1, 0].clear()
        self.axes[1, 0].plot(time_array, np.array(self.left_force_y), 'g-', linewidth=2, label='F_y')
        self.axes[1, 0].set_title('Left Leg - Force Y', fontsize=12, fontweight='bold')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Force (N)')
        self.axes[1, 0].grid(True, alpha=0.3)
        self.axes[1, 0].legend()
        
        self.axes[2, 0].clear()
        self.axes[2, 0].plot(time_array, np.array(self.left_force_z), 'b-', linewidth=2, label='F_z')
        self.axes[2, 0].set_title('Left Leg - Force Z (Vertical)', fontsize=12, fontweight='bold')
        self.axes[2, 0].set_xlabel('Time (s)')
        self.axes[2, 0].set_ylabel('Force (N)')
        self.axes[2, 0].grid(True, alpha=0.3)
        self.axes[2, 0].legend()
        
        # Clear and replot right leg forces
        self.axes[0, 1].clear()
        self.axes[0, 1].plot(time_array, np.array(self.right_force_x), 'r-', linewidth=2, label='F_x')
        self.axes[0, 1].set_title('Right Leg - Force X', fontsize=12, fontweight='bold')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Force (N)')
        self.axes[0, 1].grid(True, alpha=0.3)
        self.axes[0, 1].legend()
        
        self.axes[1, 1].clear()
        self.axes[1, 1].plot(time_array, np.array(self.right_force_y), 'g-', linewidth=2, label='F_y')
        self.axes[1, 1].set_title('Right Leg - Force Y', fontsize=12, fontweight='bold')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Force (N)')
        self.axes[1, 1].grid(True, alpha=0.3)
        self.axes[1, 1].legend()
        
        self.axes[2, 1].clear()
        self.axes[2, 1].plot(time_array, np.array(self.right_force_z), 'b-', linewidth=2, label='F_z')
        self.axes[2, 1].set_title('Right Leg - Force Z (Vertical)', fontsize=12, fontweight='bold')
        self.axes[2, 1].set_xlabel('Time (s)')
        self.axes[2, 1].set_ylabel('Force (N)')
        self.axes[2, 1].grid(True, alpha=0.3)
        self.axes[2, 1].legend()
        
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
            filename = f"contact_forces_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        
        # Use existing figure if available, otherwise create new one
        if self.fig is None:
            self.fig, self.axes = plt.subplots(3, 2, figsize=(14, 10))
            self.fig.suptitle('Contact Forces for Both Legs', fontsize=16, fontweight='bold')
        
        time_array = np.array(self.time_data)
        
        # Update all subplots
        self.axes[0, 0].clear()
        self.axes[0, 0].plot(time_array, np.array(self.left_force_x), 'r-', linewidth=2, label='F_x')
        self.axes[0, 0].set_title('Left Leg - Force X', fontsize=12, fontweight='bold')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Force (N)')
        self.axes[0, 0].grid(True, alpha=0.3)
        self.axes[0, 0].legend()
        
        self.axes[1, 0].clear()
        self.axes[1, 0].plot(time_array, np.array(self.left_force_y), 'g-', linewidth=2, label='F_y')
        self.axes[1, 0].set_title('Left Leg - Force Y', fontsize=12, fontweight='bold')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Force (N)')
        self.axes[1, 0].grid(True, alpha=0.3)
        self.axes[1, 0].legend()
        
        self.axes[2, 0].clear()
        self.axes[2, 0].plot(time_array, np.array(self.left_force_z), 'b-', linewidth=2, label='F_z')
        self.axes[2, 0].set_title('Left Leg - Force Z (Vertical)', fontsize=12, fontweight='bold')
        self.axes[2, 0].set_xlabel('Time (s)')
        self.axes[2, 0].set_ylabel('Force (N)')
        self.axes[2, 0].grid(True, alpha=0.3)
        self.axes[2, 0].legend()
        
        self.axes[0, 1].clear()
        self.axes[0, 1].plot(time_array, np.array(self.right_force_x), 'r-', linewidth=2, label='F_x')
        self.axes[0, 1].set_title('Right Leg - Force X', fontsize=12, fontweight='bold')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Force (N)')
        self.axes[0, 1].grid(True, alpha=0.3)
        self.axes[0, 1].legend()
        
        self.axes[1, 1].clear()
        self.axes[1, 1].plot(time_array, np.array(self.right_force_y), 'g-', linewidth=2, label='F_y')
        self.axes[1, 1].set_title('Right Leg - Force Y', fontsize=12, fontweight='bold')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Force (N)')
        self.axes[1, 1].grid(True, alpha=0.3)
        self.axes[1, 1].legend()
        
        self.axes[2, 1].clear()
        self.axes[2, 1].plot(time_array, np.array(self.right_force_z), 'b-', linewidth=2, label='F_z')
        self.axes[2, 1].set_title('Right Leg - Force Z (Vertical)', fontsize=12, fontweight='bold')
        self.axes[2, 1].set_xlabel('Time (s)')
        self.axes[2, 1].set_ylabel('Force (N)')
        self.axes[2, 1].grid(True, alpha=0.3)
        self.axes[2, 1].legend()
        
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
    
    node = ContactForcePlotter()
    
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

