#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
from typing import Optional

# 定义串口指令与运动指令的映射关系
CMD_MAPPING = {
    0x01: (0.0, 0.0, 0.0),    # 停止：x线速度、y线速度、z角速度均为0
    0x02: (0.0, 0.0, 0.0),    # 停止
    0x04: (0.2, 0.0, 0.0),    # x方向200mm/s → 0.2m/s（ROS Twist单位为m/s）
    0x05: (-0.2, 0.0, 0.0),   # x方向-200mm/s
    0x06: (0.0, 0.2, 0.0),    # y方向200mm/s
    0x07: (0.0, -0.2, 0.0),   # y方向-200mm/s
    0x08: (0.0, 0.0, 0.62832),# z方向角速度0.62832rad/s
    0x09: (0.0, 0.0, -0.62832) # z方向角速度0.62832rad/s
}

class SerialToCmdVelNode(Node):
    def __init__(self):
        super().__init__('serial_cmd_vel_node')
        
        # 1. 初始化串口配置（波特率改为115200）
        self.serial_port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
        self.baudrate = 115200  
        self.ser: Optional[serial.Serial] = None
        self.init_serial()

        # 2. 创建cmd_vel发布者，队列大小10
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 3. 保存当前需要持续发布的速度指令（初始为停止）
        self.current_twist = Twist()
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = 0.0
        self.current_twist.angular.z = 0.0

        # 4. 创建定时器（50Hz发布，保证小车连续运动，可根据需求调整）
        self.timer = self.create_timer(0.02, self.read_serial_and_publish)

        self.get_logger().info("Serial to cmd_vel node started successfully! (Baudrate: 115200)")

    def init_serial(self):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.01,  
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f"Connected to serial port: {self.serial_port} (Baudrate: {self.baudrate})")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

    def read_serial_and_publish(self):
        """读取串口数据更新速度指令，并持续发布当前速度"""
        # 第一步：读取串口数据，更新当前速度指令
        if self.ser and self.ser.is_open:
            try:
                # 读取5字节数据（指令帧长度固定为5）
                data = self.ser.read(5)
                if len(data) == 5:
                    # 解析指令帧：0xAA 0x55 0x00 [cmd] 0xFB
                    frame_header1, frame_header2, reserved, cmd, frame_footer = data
                    # 校验帧头、帧尾和保留位
                    if frame_header1 == 0xAA and frame_header2 == 0x55 and reserved == 0x00 and frame_footer == 0xFB:
                        self.get_logger().debug(f"Received valid cmd: 0x{cmd:02X}")
                        # 根据指令映射更新当前速度
                        if cmd in CMD_MAPPING:
                            linear_x, linear_y, angular_z = CMD_MAPPING[cmd]
                            self.current_twist.linear.x = linear_x
                            self.current_twist.linear.y = linear_y
                            self.current_twist.angular.z = angular_z
                            self.get_logger().info(
                                f"Update speed: linear_x={linear_x} m/s, linear_y={linear_y} m/s, angular_z={angular_z} rad/s"
                            )
                # 数据长度不足时，不更新速度，继续发布当前指令

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                self.ser.close()
        else:
            self.init_serial()  # 串口断开时尝试重连

        # 第二步：无条件发布当前速度指令（核心修改：持续发布，保证小车连续运动）
        self.cmd_vel_pub.publish(self.current_twist)

    def destroy_node(self):
        """节点销毁时关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()