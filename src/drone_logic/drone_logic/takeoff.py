#!/usr/bin/env python3
import time

import rclpy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node


class DroneCommander(Node):
    def __init__(self):
        super().__init__("drone_commander")
        self.get_logger().info("AEROTHON Drone Brain Booting Up...")

        # 1. Create the connections to MAVROS
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        # Wait for the drone to actually be connected before sending commands
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for flight controller...")

        self.launch_sequence()

    def launch_sequence(self):
        # 2. Hijack the drone (GUIDED mode)
        self.get_logger().info("Taking control (GUIDED mode)...")
        mode_req = SetMode.Request()
        mode_req.custom_mode = "GUIDED"
        self.mode_client.call_async(mode_req)
        time.sleep(1)  # Pause for a second to let the drone process it

        # 3. Arm the motors
        self.get_logger().info("Arming propellers...")
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arm_client.call_async(arm_req)
        time.sleep(1)

        # 4. Takeoff to 5 meters
        self.get_logger().info("Liftoff! Climbing to 5 meters...")
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = 5.0
        self.takeoff_client.call_async(takeoff_req)


def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
    # Keep the node alive just long enough to send the commands
    rclpy.spin_once(node, timeout_sec=3.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
