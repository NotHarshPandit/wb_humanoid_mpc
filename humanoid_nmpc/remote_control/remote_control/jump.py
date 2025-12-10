#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from humanoid_mpc_msgs.msg import WalkingVelocityCommand
import time


class GoAndJump(Node):
    def __init__(self, target_x=2.0):
        super().__init__("go_and_jump_command_node")

        self.pub = self.create_publisher(
            WalkingVelocityCommand,
            "/humanoid/walking_velocity_command",
            10
        )

        self.timer = self.create_timer(0.04, self.loop)  # 25 Hz like GUI
        self.state = "walk"
        self.target_x = target_x
        self.current_x = 0.0
        self.t0 = time.time()

        # If you have a topic that provides base or COM x-position, subscribe:
        # self.sub = self.create_subscription(SomeStateMsg, "/g1/mpc_observation", self.state_cb, 10)

    def loop(self):
        msg = WalkingVelocityCommand()

        if self.state == "walk":
            msg.linear_velocity_x = 0.35
            msg.desired_pelvis_height = 0.23  # nominal height

            if self.current_x >= self.target_x:
                self.state = "crouch"
                self.t0 = time.time()

        elif self.state == "crouch":
            msg.linear_velocity_x = 0.0
            msg.desired_pelvis_height = 0.18  # crouch lower
            if time.time() - self.t0 > 0.35:
                self.state = "jump"
                self.t0 = time.time()

        elif self.state == "jump":
            msg.linear_velocity_x = 1.2   # forward impulse
            msg.desired_pelvis_height = 0.35  # strong upward target
            if time.time() - self.t0 > 0.25:
                self.state = "land"
                self.t0 = time.time()

        elif self.state == "land":
            msg.linear_velocity_x = 0.2
            msg.desired_pelvis_height = 0.23
            if time.time() - self.t0 > 0.6:
                self.state = "done"

        elif self.state == "done":
            msg.linear_velocity_x = 0.0
            msg.desired_pelvis_height = 0.23

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GoAndJump(target_x=2.0)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
