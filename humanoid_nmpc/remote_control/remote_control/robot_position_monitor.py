#!/usr/bin/env python3
"""
Monitor robot's current position and progress toward waypoint.
Subscribes to MPC observation topic to get current robot state.
"""
import rclpy
from rclpy.node import Node
from ocs2_ros2_msgs.msg import MpcObservation
import math
import time


class RobotPositionMonitor(Node):
    def __init__(self):
        super().__init__("robot_position_monitor")
        
        # Subscribe to MPC observation
        self.observation_subscriber = self.create_subscription(
            MpcObservation,
            "/g1_wb_mpc/mpc_observation",  # Adjust robot name if needed
            self.observation_callback,
            10
        )
        
        # Store current state
        self.current_pose = None
        self.last_update_time = None
        
        self.get_logger().info("Robot Position Monitor started")
        self.get_logger().info("Subscribing to /g1_wb_mpc/mpc_observation")
        self.get_logger().info("Waiting for observations...")
        
    def observation_callback(self, msg):
        """Extract position from MPC observation message."""
        # MPC observation contains state vector
        # For whole-body MPC, state structure is: [base_pos(3), base_orient(3), joints, base_vel(3), base_ang_vel(3), joint_vels]
        # But we need to check the actual structure
        
        if len(msg.state.value) < 6:
            self.get_logger().warn(f"State vector too short: {len(msg.state.value)}")
            return
            
        # Extract base position (first 3 elements are typically x, y, z)
        # Extract base orientation (next 3 are typically roll, pitch, yaw)
        x = msg.state.value[0]
        y = msg.state.value[1]
        z = msg.state.value[2]
        
        # Orientation (Euler ZYX typically)
        roll = msg.state.value[3] if len(msg.state.value) > 3 else 0.0
        pitch = msg.state.value[4] if len(msg.state.value) > 4 else 0.0
        yaw = msg.state.value[5] if len(msg.state.value) > 5 else 0.0
        
        self.current_pose = {
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'time': msg.time
        }
        self.last_update_time = time.time()
        
    def print_current_position(self):
        """Print current robot position."""
        if self.current_pose is None:
            print("No position data received yet. Waiting for MPC observations...")
            return
            
        p = self.current_pose
        print(f"\n{'='*60}")
        print(f"Robot Current Position:")
        print(f"  X:     {p['x']:8.4f} m")
        print(f"  Y:     {p['y']:8.4f} m")
        print(f"  Z:     {p['z']:8.4f} m")
        print(f"  Roll:  {p['roll']:8.4f} rad ({math.degrees(p['roll']):6.2f}°)")
        print(f"  Pitch: {p['pitch']:8.4f} rad ({math.degrees(p['pitch']):6.2f}°)")
        print(f"  Yaw:   {p['yaw']:8.4f} rad ({math.degrees(p['yaw']):6.2f}°)")
        print(f"  Time:  {p['time']:8.4f} s")
        print(f"{'='*60}")
        
    def monitor_continuously(self, update_rate=1.0):
        """Continuously monitor and print position."""
        print("\nStarting continuous monitoring (Ctrl+C to stop)...")
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                self.print_current_position()
                time.sleep(update_rate)
        except KeyboardInterrupt:
            print("\nMonitoring stopped.")


def main():
    rclpy.init()
    
    monitor = RobotPositionMonitor()
    
    print("\n=== Robot Position Monitor ===")
    print("Options:")
    print("  1. Print current position once")
    print("  2. Monitor continuously (updates every second)")
    print("\nWaiting 2 seconds for first observation...")
    
    # Wait for first observation
    start_time = time.time()
    while monitor.current_pose is None and (time.time() - start_time) < 5.0:
        rclpy.spin_once(monitor, timeout_sec=0.1)
        time.sleep(0.1)
    
    if monitor.current_pose is None:
        print("\nWARNING: No observation received!")
        print("Check if:")
        print("  1. MPC node is running")
        print("  2. Topic name is correct: /g1_wb_mpc/mpc_observation")
        print("  3. Try: ros2 topic list | grep observation")
        monitor.destroy_node()
        rclpy.shutdown()
        return
    
    # Print once
    monitor.print_current_position()
    
    # Ask if user wants continuous monitoring
    print("\nStart continuous monitoring? (y/n): ", end='')
    try:
        response = input().strip().lower()
        if response == 'y' or response == 'yes':
            monitor.monitor_continuously()
    except (EOFError, KeyboardInterrupt):
        pass
    
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

