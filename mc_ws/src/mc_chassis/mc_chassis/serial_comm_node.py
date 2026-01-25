#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray, String
import threading
import time
import os

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')
        self.serial_config = {
            'port': '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00',
            'baudrate': 115200,
            'bytesize': serial.EIGHTBITS,
            'parity': serial.PARITY_NONE,
            'stopbits': serial.STOPBITS_ONE,
            'timeout': 0.1
        }
        
        self.ser = None
        self.serial_connected = False
        self.serial_lock = threading.Lock()
        self.reconnect_success_logged = False

        self.init_serial()

        self.wheel_speed_pub = self.create_publisher(Float32MultiArray, 'wheel_speed', 10)
        self.serial_cmd_sub = self.create_subscription(
            String, 'serial_raw_cmd', self.send_cmd, 10
        )

        self.reconnect_timer = self.create_timer(5.0, self.check_and_reconnect_serial)

        self.read_thread_running = True
        self.read_thread = threading.Thread(target=self.read_wheel_speed, daemon=True)
        self.read_thread.start()

        self.valid_cmds = ['A','B','C','D','E','F','G','H','K','I','J','X','Y','Z','0']

        if self.serial_connected:
            self.get_logger().info("串口通信节点已启动 → 串口连接成功")
        else:
            self.get_logger().warn(f"串口通信节点已启动 → 串口未连接（设备：{self.serial_config['port']}），每5秒重试，暂发布全0数据")

    def check_device_exists(self):
        try:
            return os.path.exists(self.serial_config['port'])
        except:
            return False

    def init_serial(self):
        with self.serial_lock:
            if not self.check_device_exists():
                self.get_logger().error(f"串口设备不存在：{self.serial_config['port']}")
                self.serial_connected = False
                return False

            try:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
                    time.sleep(0.2)
            except:
                pass

            try:
                self.ser = serial.Serial(**self.serial_config)
                time.sleep(0.5)
                if self.ser.is_open:
                    self.ser.flushInput()
                    self.ser.flushOutput()
                    self.serial_connected = True
                    self.reconnect_success_logged = False
                    self.get_logger().info(f"串口连接成功：{self.serial_config['port']}（波特率：{self.serial_config['baudrate']}）")
                    return True
            except serial.SerialException as e:
                self.get_logger().error(f"串口打开失败：{str(e)}")
            except Exception as e:
                self.get_logger().error(f"串口初始化未知错误：{str(e)}")

            self.serial_connected = False
            return False

    def check_and_reconnect_serial(self):
        if not self.serial_connected:
            if self.check_device_exists():
                self.get_logger().warn("尝试重新连接串口...")
                self.init_serial()
            else:
                self.get_logger().warn(f"串口设备仍不存在：{self.serial_config['port']}，等待设备插入")

    def send_cmd(self, msg):
        """过滤空指令，避免刷屏"""
        # 第一步：过滤空指令/全空白指令
        cmd = msg.data.strip().upper()
        if not cmd:  # 空字符串或仅空格
            self.get_logger().debug("收到空指令，直接忽略（不再替换为Z）")  
            return

        if not self.serial_connected:
            self.get_logger().warn("串口未连接，跳过指令下发")
            return

        with self.serial_lock:
            try:
                # 收到0xxxx（5位、以0开头），转换为{0xxxx}后发送
                if len(cmd) == 5 and cmd.startswith('0'):
                    send_cmd = f"{{{cmd}}}"  # 拼接成{0xxxx}格式，{{}}转义为单个{/}
                    self.ser.write(send_cmd.encode('utf-8'))
                    self.get_logger().debug(f"下发速度指令：原指令{cmd} → 转换为{send_cmd}（速度设置）")
                    return


                if cmd not in self.valid_cmds:
                    self.get_logger().warning(f"非法指令[{cmd}] → 替换为停车Z")
                    cmd = 'Z'

                self.ser.write(cmd.encode('utf-8'))
                self.get_logger().debug(f"下发指令：{cmd}")

            except serial.SerialException as e:
                self.get_logger().error(f"指令下发失败（串口断开）：{str(e)}")
                self.serial_connected = False
            except Exception as e:
                self.get_logger().error(f"指令下发异常（非串口断开）：{str(e)}")

    def read_wheel_speed(self):
        self.get_logger().info("轮速读取线程启动（最终修复版）")
        data_buffer = ""
        while self.read_thread_running and rclpy.ok():
            try:
                if not self.serial_connected:
                    zero_msg = Float32MultiArray()
                    zero_msg.data = [0.0, 0.0, 0.0, 0.0]
                    self.wheel_speed_pub.publish(zero_msg)
                    time.sleep(0.05)
                    continue

                with self.serial_lock:
                    if self.ser.in_waiting > 0:
                        new_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                        data_buffer += new_data

                        if '$' in data_buffer:
                            packets = data_buffer.split('$')
                            data_buffer = packets[-1]
                            for packet in packets[:-1]:
                                full_packet = packet + '$'
                                if full_packet.startswith('{A') and full_packet.endswith('$'):
                                    clean_data = full_packet.lstrip('{A').rstrip('$')
                                    data_parts = clean_data.split(':')
                                    if len(data_parts) == 5:
                                        try:
                                            wheel_speeds = [float(part) * 0.01 for part in data_parts[:4]]
                                            speed_msg = Float32MultiArray()
                                            speed_msg.data = wheel_speeds
                                            self.wheel_speed_pub.publish(speed_msg)
                                            self.get_logger().debug(f"发布轮速：{wheel_speeds} m/s")
                                        except ValueError as e:
                                            self.get_logger().warning(f"轮速解析错误：{full_packet} → {e}")
                                    else:
                                        self.get_logger().warning(f"轮速数据长度错误：{full_packet}（分割后{len(data_parts)}段）")

                time.sleep(0.01)

            except serial.SerialException as e:
                self.get_logger().error(f"串口读取失败（串口断开）：{str(e)}")
                self.serial_connected = False
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().warning(f"轮速读取非致命错误：{str(e)}")
                time.sleep(0.1)

    def destroy_node(self):
        self.read_thread_running = False
        if hasattr(self, 'reconnect_timer'):
            self.reconnect_timer.cancel()
        with self.serial_lock:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
               
                if rclpy.ok():
                    self.get_logger().info("串口已安全关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok(): 
            node.get_logger().info("收到中断，关闭节点")
    finally:
        
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()