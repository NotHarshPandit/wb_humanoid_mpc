#!/usr/bin/env python3
"""
Simple waypoint command publisher for testing waypoint-based motion.
Publishes waypoint commands to move robot from current position to target waypoint.
"""
import rclpy
from rclpy.node import Node
from humanoid_mpc_msgs.msg import WaypointCommand
import math


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")
        
        self.publisher_ = self.create_publisher(
            WaypointCommand,
            "/humanoid/waypoint_command",
            10
        )
        
        self.get_logger().info("Waypoint Publisher started")
        self.get_logger().info("Publishing waypoint commands to /humanoid/waypoint_command")
        
    def publish_waypoint(self, x, y, z, yaw, time_to_reach=5.0, motion_type="waypoint"):
        """
        Publish a waypoint command.
        
        Args:
            x: Target x position in global frame [m]
            y: Target y position in global frame [m]
            z: Target z position (height) in global frame [m]
            yaw: Target yaw angle in global frame [rad]
            time_to_reach: Time to reach waypoint [s] (0.0 for auto-estimate)
            motion_type: Motion type string ("waypoint", "jump", "backflip")
        """
        msg = WaypointCommand()
        msg.target_x = float(x)
        msg.target_y = float(y)
        msg.target_z = float(z)
        msg.target_yaw = float(yaw)
        msg.time_to_reach = float(time_to_reach)
        msg.motion_type = motion_type
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published waypoint: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}, "
            f"time={time_to_reach:.2f}s, type={motion_type}"
        )


def main():
    rclpy.init()
    
    node = WaypointPublisher()
    
    # Wait for subscribers to connect
    import time
    print("\n=== Waypoint Command Publisher ===")
    print("Waiting for subscribers to connect...")
    
    # Wait up to 10 seconds for subscribers
    max_wait_time = 10.0
    wait_interval = 0.5
    waited = 0.0
    subscriber_count = 0
    
    while waited < max_wait_time:
        subscriber_count = node.publisher_.get_subscription_count()
        if subscriber_count > 0:
            break
        time.sleep(wait_interval)
        waited += wait_interval
        print(f"  Waiting... ({waited:.1f}s)")
    
    print(f"\nSubscriber count: {subscriber_count}")
    
    if subscriber_count == 0:
        print("\n❌ ERROR: No subscribers detected after {:.1f}s!".format(max_wait_time))
        print("Make sure the MPC node is running:")
        print("  - Check if g1_wb_mpc_sqp_node is running")
        print("  - Verify subscription: ros2 topic info /humanoid/waypoint_command")
        print("\nExiting without publishing.")
        node.destroy_node()
        rclpy.shutdown()
        return
    else:
        print(f"✓ Found {subscriber_count} subscriber(s) - ready to publish!")
    
    # Example: Move 1 meter forward
    print("\n" + "="*60)
    print("Publishing waypoint command:")
    print("  - Move 1.0m forward (relative)")
    print("  - Height: 0.85m (absolute)")
    print("  - Time to reach: 5.0s")
    print("="*60 + "\n")
    
    # Publish waypoint command ONCE (multiple publishes cause moving target!)
    node.publish_waypoint(
        x = 7.0,      # 1 meter forward (relative)
        y=0.0,      # no lateral movement
        z=0.85,     # 0.85m height (absolute)
        yaw = 0.0,    # no rotation
        time_to_reach= 5.0,  # 5 seconds to reach
        motion_type="waypoint"
    )
    print("  ✓ Waypoint command published!")
    
    # Keep node alive a bit longer to ensure message is sent
    print("\nWaiting 1 second to ensure message delivery...")
    time.sleep(1.0)
    
    node.destroy_node()
    rclpy.shutdown()
    
    print("\nWaypoint command published! Check MPC node output for processing messages.")
    print("If robot doesn't move, check:")
    print("  1. Is the MPC node running? (g1_wb_mpc_sqp_node)")
    print("  2. Check MPC node output for waypoint processing messages")
    print("  3. Verify robot is in a valid state to move")


if __name__ == "__main__":
    main()

