#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String, Float64, Float64MultiArray
from cv_bridge import CvBridge

class MonoDriver(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        # ===== CvBridge =====
        self.br = CvBridge()

        # ===== Handshake variables =====
        self.send_config = True
        self.exp_config_msg = "EuRoC"   # เปลี่ยนชื่อ config ที่ต้องการ เช่น "EuRoC", "TUM2"
        self.publish_exp_config_ = self.create_publisher(String, "/mono_py_driver/experiment_settings", 1)
        self.subscribe_exp_ack_ = self.create_subscription(String, "/mono_py_driver/exp_settings_ack", self.ack_callback, 10)

        # ===== Forward publishers =====
        self.pub_img_to_agent = self.create_publisher(Image, "/mono_py_driver/img_msg", 1)
        self.pub_timestep_to_agent = self.create_publisher(Float64, "/mono_py_driver/timestep_msg", 1)
        self.pub_gyro_to_agent = self.create_publisher(Float64MultiArray, "/mono_py_driver/imu_gyro", 10)
        self.pub_accel_to_agent = self.create_publisher(Float64MultiArray, "/mono_py_driver/imu_accel", 10)

        # ===== Subscribers จากกล้อง =====
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.camera_callback, 10)
        self.create_subscription(Imu, "/camera/camera/gyro/sample", self.gyro_callback, 50)
        self.create_subscription(Imu, "/camera/camera/accel/sample", self.accel_callback, 50)

        self.get_logger().info("MonoDriver listening to camera + IMU topics")

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
        """รับภาพจากกล้องแล้ว forward ไปยัง C++ node"""
        try:
            timestep_msg = Float64()
            timestep_msg.data = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self.pub_timestep_to_agent.publish(timestep_msg)
            self.pub_img_to_agent.publish(msg)

            self.get_logger().info(f"Forwarded image frame at {timestep_msg.data:.6f}")
        except Exception as e:
            self.get_logger().error(f"Error forwarding image: {e}")

    # ---------------------------------------------------------------------------------
    def gyro_callback(self, msg: Imu):
        """รับ gyro data แล้ว forward"""
        arr = Float64MultiArray()
        arr.data = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        self.pub_gyro_to_agent.publish(arr)
        # self.get_logger().info(f"Gyro: {arr.data}")

    # ---------------------------------------------------------------------------------
    def accel_callback(self, msg: Imu):
        """รับ accel data แล้ว forward"""
        arr = Float64MultiArray()
        arr.data = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        self.pub_accel_to_agent.publish(arr)
        # self.get_logger().info(f"Accel: {arr.data}")

# ---------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MonoDriver()

    # ===== Handshake loop =====
    while rclpy.ok() and node.send_config:
        node.handshake_with_cpp_node()
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Handshake complete, now forwarding camera + IMU frames...")

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
