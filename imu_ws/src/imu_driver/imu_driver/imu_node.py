#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import serial
import serial.tools.list_ports
import threading
import struct
import time
import platform
import transforms3d as tfs

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.executors import MultiThreadedExecutor

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE

# 宏定义参数
PI = 3.141592653589793
FRAME_HEAD = 'fc'
FRAME_END = 'fd'
TYPE_IMU = '40'
TYPE_AHRS = '41'
TYPE_INSGPS = '42'
TYPE_GEODETIC_POS = '5c'
TYPE_GROUND = 'f0'
TYPE_SYS_STATE = '50'
TYPE_BODY_ACCELERATION = '62'
TYPE_ACCELERATION = '61'
TYPE_MSG_BODY_VEL = '60'
IMU_LEN = '38' 
AHRS_LEN = '30' 
INSGPS_LEN = '48'  
GEODETIC_POS_LEN = '20'  
SYS_STATE_LEN = '64'  
BODY_ACCELERATION_LEN = '10'  
ACCELERATION_LEN = '0c'  
DEG_TO_RAD = 0.017453292519943295
isrun = True

class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        # 声明并获取参数
        self.declare_parameter('port', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.declare_parameter('bps', 921600)
        self.declare_parameter('timeout', 1)
        self.declare_parameter('frame_id', 'imu_link')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.bps = self.get_parameter('bps').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # 原始IMU数据发布到 /imu/data_raw（供滤波节点订阅）
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)

        # 初始化串口
        self.serial_ = None
        self.init_serial()

        # 创建数据接收线程
        self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.receive_thread.start()

        self.get_logger().info(f"IMU驱动节点已启动，串口: {self.port}, 波特率: {self.bps}")

    def init_serial(self):
        """初始化串口"""
        try:
            self.serial_ = serial.Serial(
                port=self.port,
                baudrate=self.bps,
                bytesize=EIGHTBITS,
                parity=PARITY_NONE,
                stopbits=STOPBITS_ONE,
                timeout=self.timeout
            )
            self.serial_.flushInput()
            self.serial_.flushOutput()
            self.get_logger().info(f"成功打开串口: {self.port}")
        except Exception as e:
            self.get_logger().error(f"打开串口失败: {e}")
            sys.exit(1)

    def receive_data(self):
        """接收并解析IMU数据线程"""
        self.get_logger().info("数据接收线程已启动，开始监听IMU数据...")
        while rclpy.ok() and self.serial_.isOpen():
            # 读取帧头并校验
            check_head = self.serial_.read().hex()
            if check_head != FRAME_HEAD:
                continue

            # 读取数据类型
            head_type = self.serial_.read().hex()
            self.get_logger().debug(f"检测到数据类型: {head_type}")

            # 只处理核心的IMU和AHRS类型
            if head_type not in [TYPE_IMU, TYPE_AHRS]:
                self.skip_remaining_bytes()
                continue

            # 读取长度并校验
            check_len = self.serial_.read().hex()
            valid = True
            if head_type == TYPE_IMU and check_len != IMU_LEN:
                valid = False
                self.get_logger().debug(f"IMU长度校验失败: 实际{check_len}, 预期{IMU_LEN}")
            elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
                valid = False
                self.get_logger().debug(f"AHRS长度校验失败: 实际{check_len}, 预期{AHRS_LEN}")

            if not valid:
                self.skip_remaining_bytes()
                continue

          
            self.serial_.read(4)

            # 解析IMU数据并发布
            if head_type == TYPE_IMU:
                try:
                    data_s = self.serial_.read(int(IMU_LEN, 16))
                    IMU_DATA = struct.unpack('12f ii', data_s[0:56])
                    self.get_logger().debug(f"解析到IMU数据: 陀螺仪({IMU_DATA[0]:.4f}, {IMU_DATA[1]:.4f}, {IMU_DATA[2]:.4f})")

                    # 发布原始IMU数据（/imu/data_raw）
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = self.frame_id

                    # 角速度（rad/s）
                    imu_msg.angular_velocity.x = IMU_DATA[0]
                    imu_msg.angular_velocity.y = IMU_DATA[1]
                    imu_msg.angular_velocity.z = IMU_DATA[2]

                    # 线加速度（m/s²）
                    imu_msg.linear_acceleration.x = IMU_DATA[3]
                    imu_msg.linear_acceleration.y = IMU_DATA[4]
                    imu_msg.linear_acceleration.z = IMU_DATA[5]

                    # 填充协方差
                    imu_msg.angular_velocity_covariance = [
                        0.001**2, 0.0, 0.0,
                        0.0, 0.001**2, 0.0,
                        0.0, 0.0, 0.001**2
                    ]
                    imu_msg.linear_acceleration_covariance = [
                        0.1**2, 0.0, 0.0,
                        0.0, 0.1**2, 0.0,
                        0.0, 0.0, 0.1**2
                    ]
                    # 姿态暂时设为初始值
                    imu_msg.orientation.x = 0.0
                    imu_msg.orientation.y = 0.0
                    imu_msg.orientation.z = 0.0
                    imu_msg.orientation.w = 1.0
                    imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                    self.imu_publisher.publish(imu_msg)
                    self.get_logger().info(f"成功发布原始IMU数据: 加速度Z={IMU_DATA[5]:.2f} m/s²", throttle_duration_sec=1)

                    # 发布磁力计数据
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = imu_msg.header.stamp
                    mag_msg.header.frame_id = self.frame_id
                    mag_msg.magnetic_field.x = IMU_DATA[6]
                    mag_msg.magnetic_field.y = IMU_DATA[7]
                    mag_msg.magnetic_field.z = IMU_DATA[8]
                    mag_msg.magnetic_field_covariance = [
                        0.1**2, 0.0, 0.0,
                        0.0, 0.1**2, 0.0,
                        0.0, 0.0, 0.1**2
                    ]
                    self.mag_publisher.publish(mag_msg)

                except struct.error as e:
                    self.get_logger().error(f"解析IMU数据失败: {e}")
                    self.skip_remaining_bytes()

            # 解析AHRS数据（姿态）
            elif head_type == TYPE_AHRS:
                try:
                    data_s = self.serial_.read(int(AHRS_LEN, 16))
                    AHRS_DATA = struct.unpack('10f ii', data_s[0:48])
                    # 可在这里解析姿态角并更新Imu消息的orientation
                except struct.error as e:
                    self.get_logger().error(f"解析AHRS数据失败: {e}")
                    self.skip_remaining_bytes()

    def skip_remaining_bytes(self):
        """跳过当前帧剩余字节，避免数据错位"""
        for _ in range(100):
            byte = self.serial_.read()
            if byte.hex() == FRAME_HEAD:
                break

    def destroy_node(self):
        """关闭节点时清理资源"""
        if self.serial_ and self.serial_.isOpen():
            self.serial_.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(imu_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        imu_node.get_logger().info("用户终止程序")
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()