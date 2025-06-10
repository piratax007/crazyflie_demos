#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import Takeoff, Land
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist


class CrazyflieTeleop(Node):
    def __init__(self):
        super().__init__("crazyflie_teleop")
        self.takeoff_client = self.create_client(Takeoff, '/all/takeoff')
        self.land_client = self.create_client(Land, '/all/land')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.command_callback, 10)
        self.get_logger().info('Crazyflie Teleop node initialized.')

    def command_callback(self, msg):
        self.get_logger().info(f"Received command velocity: linear.z = {msg.linear.z}")

        if msg.linear.z > 0:
            self.get_logger().info("Takeoff command detected.")
            self.takeoff()
        elif msg.linear.z < 0:
            self.get_logger().info("Land command detected.")
            self.land()

    def takeoff(self, height=1.0, duration_sec=2.0):
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for takeoff service...')
        request = Takeoff.Request()
        request.height = height
        request.duration = Duration()
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        self.get_logger().info("Sending takeoff request...")
        self.future = self.takeoff_client.call_async(request)
        self.future.add_done_callback(self.takeoff_callback)

    def takeoff_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Takeoff completed successfully.")
        except Exception as e:
            self.get_logger().error(f"Takeoff service call failed: {str(e)}")
        
    def land(self, duration_sec=2.0):
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for land service...')
        request = Land.Request()
        request.height = 0.0
        request.duration = Duration()
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        self.get_logger().info("Sending landing request...")
        self.future = self.land_client.call_async(request)
        self.future.add_done_callback(self.land_callback)

    def land_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Landing complete successfully.")
        except Exception as e:
            self.get_logger().error(f"Landing service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieTeleop()

    try:
        node.get_logger().info('Waiting for teleop commands...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
     main()