#!/usr/bin/env python3
"""
Quick script to check robot's current position from MPC observation.
"""
import rclpy
from rclpy.node import Node
from ocs2_ros2_msgs.msg import MpcObservation
import sys


class PositionChecker(Node):
    def __init__(self):
        super().__init__("position_checker")
        
        self.subscription = self.create_subscription(
            MpcObservation,
            "/g1/mpc_observation",
            self.callback,
            10
        )
        self.received = False
        
    def callback(self, msg):
        if self.received:
            return
            
        self.received = True
        
        # Extract state values
        state = msg.state.value
        
        if len(state) >= 6:
            x = state[0]
            y = state[1]
            z = state[2]
            roll = state[3] if len(state) > 3 else 0.0
            pitch = state[4] if len(state) > 4 else 0.0
            yaw = state[5] if len(state) > 5 else 0.0
            
            print("\n" + "="*60)
            print("Robot Current Position:")
            print(f"  X:     {x:10.4f} m")
            print(f"  Y:     {y:10.4f} m")
            print(f"  Z:     {z:10.4f} m")
            print(f"  Roll:  {roll:10.4f} rad")
            print(f"  Pitch: {pitch:10.4f} rad")
            print(f"  Yaw:   {yaw:10.4f} rad")
            print(f"  Time:  {msg.time:10.4f} s")
            print("="*60 + "\n")
        else:
            print(f"Error: State vector too short ({len(state)} elements)")
            print(f"First 10 values: {state[:10]}")
        
        rclpy.shutdown()


def main():
    rclpy.init()
    checker = PositionChecker()
    
    # Wait up to 3 seconds for message
    import time
    start = time.time()
    while not checker.received and (time.time() - start) < 3.0:
        rclpy.spin_once(checker, timeout_sec=0.1)
    
    if not checker.received:
        print("ERROR: No observation received!")
        print("Make sure:")
        print("  1. MPC node is running")
        print("  2. Topic exists: ros2 topic list | grep observation")
        sys.exit(1)
    
    checker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

