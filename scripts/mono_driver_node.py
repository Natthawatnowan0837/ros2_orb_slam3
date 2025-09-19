#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge


class MonoDriver(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        # ===== CvBridge =====
        self.br = CvBridge()

        # ===== Handshake variables =====
        self.send_config = True
        self.exp_config_msg = "EuRoC"   
        self.publish_exp_config_ = self.create_publisher(String, "/mono_py_driver/experiment_settings", 1)
        self.subscribe_exp_ack_ = self.create_subscription(
            String, "/mono_py_driver/exp_settings_ack", self.ack_callback, 10
        )

        # ===== Forward publishers =====
        self.pub_img_to_agent = self.create_publisher(Image, "/mono_py_driver/img_msg", 1)
        self.pub_depth_to_agent = self.create_publisher(Image, "/mono_py_driver/depth_msg", 1)
        self.pub_imu_to_agent = self.create_publisher(Imu, "/mono_py_driver/imu_raw", 100)
        self.pub_timestep_to_agent = self.create_publisher(Float64, "/mono_py_driver/timestep_msg", 1)

        # ===== Subscribers จากกล้อง =====
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.camera_callback, 10)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 10)
        self.create_subscription(Imu, "/camera/camera/imu", self.imu_callback, 200)

        self.get_logger().info("MonoDriver listening to camera RGB + Depth + IMU topics")

    # ---------------------------------------------------------------------------------
    def ack_callback(self, msg: String):
        self.get_logger().info(f"Got ack: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False

    # ---------------------------------------------------------------------------------
    def handshake_with_cpp_node(self):
        """ส่ง config ไปยัง C++ node จนกว่าจะได้ ACK"""
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            self.get_logger().info(f"Sent config: {msg.data}")
            time.sleep(0.05)

    # ---------------------------------------------------------------------------------
    def camera_callback(self, msg: Image):
        """รับภาพ RGB แล้ว forward ไปยัง C++ node"""
        try:
            timestep_msg = Float64()
            timestep_msg.data = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self.pub_timestep_to_agent.publish(timestep_msg)
            self.pub_img_to_agent.publish(msg)

            self.get_logger().info(f"Forwarded RGB frame at {timestep_msg.data:.6f}")
        except Exception as e:
            self.get_logger().error(f"Error forwarding RGB: {e}")

    # ---------------------------------------------------------------------------------
    def depth_callback(self, msg: Image):
        """รับ depth image แล้ว forward"""
        try:
            self.pub_depth_to_agent.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error forwarding depth: {e}")

    # ---------------------------------------------------------------------------------
    def imu_callback(self, msg: Imu):
        """รับ IMU แล้ว forward"""
        self.pub_imu_to_agent.publish(msg)


# ---------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MonoDriver()

    # ===== Handshake loop =====
    while rclpy.ok() and node.send_config:
        node.handshake_with_cpp_node()
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Handshake complete, now forwarding RGB + Depth + IMU frames...")

    # ===== Main spin =====
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
