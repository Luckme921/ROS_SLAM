#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import threading
import math
import numpy as np

class ChassisController(Node):
    def __init__(self):
        super().__init__('chassis_controller')
        # 1. 订阅标准化cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        # 2. 发布串口指令
        self.serial_cmd_pub = self.create_publisher(String, 'serial_raw_cmd', 10)
        
        # ========== 基础配置（适配STM32新规则） ==========
        self.in_turn_mode = False  # 是否在转向模式
        self.last_cmd_time = time.time()
        self.timeout_threshold = 0.5  # 超时阈值（超时发空格停止）
        self.cmd_delay = 0.08         # 模式切换指令延迟
        self.motion_pub_freq = 10    # 运动指令下发频率（10Hz=100ms/次）
        self.motion_pub_interval = 1.0 / self.motion_pub_freq  # 100ms
        self.last_motion_pub_time = 0.0  # 最后一次下发运动指令时间
        self.current_motion_cmd = ''     # 当前运动指令（空=无）
        
        # STM32指令规则 - 调整初始速度和加减速步长
        self.INIT_SPEED = 0.2        # 初始速度200mm/s = 0.2m/s
        self.current_speed = self.INIT_SPEED
        self.speed_step = 0.01       # 加减速步长10mm/s
        self.STOP_CHAR = 'Z'         # 停车字符（Z，串口节点识别为停车）
        self.MAX_SPEED_MM = 1000     # 最大速度限制1000mm/s
        self.MIN_SPEED_MM = 0        # 最小速度限制0mm/s
        
        # ========== 速度缓存（避免频繁下发速度指令） ==========
        self.last_linear_speed_mm = 0  # 上一次下发的线速度（mm/s）
        self.last_angular_speed_mm = 0 # 上一次下发的角速度对应线速度（mm/s）
        self.speed_change_thresh = 5   # 速度变化阈值（mm/s），超过才重新下发速度指令
        
        # ========== 指令映射（完全保留原有映射，不改动） ==========
        # 移动指令（A-H）
        self.LINEAR_CMD_MAP = {
            'forward': 'A',      # linear.x>0
            'backward': 'E',     # linear.x<0
            'left': 'G',         # linear.y>0
            'right': 'C',        # linear.y<0
            'forward_left': 'H', # linear.x>0 & linear.y>0
            'forward_right': 'B',# linear.x>0 & linear.y<0
            'backward_left': 'F',# linear.x<0 & linear.y>0
            'backward_right': 'D'# linear.x<0 & linear.y<0
        }
        # 转向指令（C/G）
        self.ANGULAR_CMD_MAP = {
            'turn_left': 'G',    # angular.z>0
            'turn_right': 'C'    # angular.z<0
        }
        # 有效运动指令集合（A-H/C/G）
        self.VALID_MOTION_CMDS = {'A','B','C','D','E','F','G','H'}
        
        # 速度阈值（过滤微小指令）
        self.linear_thresh = 0.05  # 线速度阈值（m/s）
        self.angular_thresh = 0.05 # 角速度阈值（rad/s）
        
        # 定时器
        self.timeout_timer = self.create_timer(0.05, self.check_timeout)  # 超时检测
        self.motion_timer = self.create_timer(self.motion_pub_interval, self.publish_motion_cmd) # 持续发运动指令
        
        self.get_logger().info("底盘控制节点（适配STM32新规则）已启动")

    # ========== 生成速度设置指令 ==========
    def generate_speed_cmd(self, speed_mm):
        """生成0xxxx格式的速度设置指令"""
        speed_clamped = max(self.MIN_SPEED_MM, min(self.MAX_SPEED_MM, int(round(speed_mm))))
        speed_cmd = f"0{speed_clamped:04d}"
        return speed_cmd

    # ========== 指令下发核心逻辑 ==========
    def publish_cmd(self, cmd, desc):
        """封装指令发布（仅保留rclpy.ok()判断）"""
        if not rclpy.ok():  
            return
        cmd_msg = String()
        cmd_msg.data = cmd
        try:
            self.serial_cmd_pub.publish(cmd_msg)
            log_level = "debug" if cmd in self.VALID_MOTION_CMDS or cmd.startswith('0') else "info"
            if log_level == "info":
                self.get_logger().info(f"[{desc}] 下发串口指令：'{cmd}'（ASCII：{ord(cmd) if len(cmd)==1 else 'N/A'}）")
            else:
                if cmd.startswith('0'):
                    self.get_logger().debug(f"[{desc}] 下发速度指令：'{cmd}' | 对应速度：{int(cmd[1:])} mm/s")
                else:
                    self.get_logger().debug(f"[{desc}] 下发运动指令：'{cmd}'")
        except Exception as e:
            self.get_logger().warning(f"下发指令失败：{e}")

    def publish_motion_cmd(self):
        """持续下发当前运动指令"""
        if self.current_motion_cmd and self.current_motion_cmd in self.VALID_MOTION_CMDS:
            self.publish_cmd(self.current_motion_cmd, f"持续运动（{self.motion_pub_freq}Hz）")
            self.last_motion_pub_time = time.time()

    def enter_turn_mode(self, turn_cmd, target_omega):
        """进入转向模式，先下发角速度对应的速度指令（仅速度变化时）"""
        # 根据角速度反推目标线速度（mm/s）
        rc_velocity = abs(target_omega) * 500 / (math.pi / 2)
        # 仅当速度变化超过阈值时，才下发速度指令
        if abs(rc_velocity - self.last_angular_speed_mm) >= self.speed_change_thresh:
            speed_cmd = self.generate_speed_cmd(rc_velocity)
            self.publish_cmd(speed_cmd, "转向模式-设置目标速度")
            self.last_angular_speed_mm = rc_velocity  # 更新缓存
        
        # 延迟发送转向指令
        self.publish_cmd('K', "进入转向模式")
        self.in_turn_mode = True
        
        def delayed_pub():
            self.current_motion_cmd = turn_cmd
            self.publish_cmd(turn_cmd, "转向模式-运动")
        threading.Timer(self.cmd_delay, delayed_pub).start()

    def exit_turn_mode(self, dir_cmd, target_speed_mm=None):
        """退出转向模式，先下发线速度指令（仅速度变化时）"""
        # 确定目标线速度（默认使用当前速度）
        if target_speed_mm is None:
            target_speed_mm = self.current_speed * 1000  # m/s → mm/s
        
        # 仅当速度变化超过阈值时，才下发速度指令
        if abs(target_speed_mm - self.last_linear_speed_mm) >= self.speed_change_thresh:
            speed_cmd = self.generate_speed_cmd(target_speed_mm)
            self.publish_cmd(speed_cmd, "移动模式-设置目标速度")
            self.last_linear_speed_mm = target_speed_mm  # 更新缓存
        
        # 延迟发送移动指令
        self.publish_cmd('K', "退出转向模式-发送停止指令K")
        self.publish_cmd('I', "退出转向模式")
        self.in_turn_mode = False
        
        def delayed_pub():
            self.current_motion_cmd = dir_cmd
            self.publish_cmd(dir_cmd, "移动模式-运动")
        threading.Timer(self.cmd_delay, delayed_pub).start()

    # ========== cmd_vel解析（ ==========
    def cmd_vel_callback(self, msg):
        """解析cmd_vel，仅速度变化时下发速度指令；停止时先设速度0再发停车字符"""
        self.last_cmd_time = time.time()
        # 1. 提取期望速度
        target_vx = msg.linear.x if abs(msg.linear.x) > self.linear_thresh else 0.0
        target_vy = msg.linear.y if abs(msg.linear.y) > self.linear_thresh else 0.0
        target_omega = msg.angular.z if abs(msg.angular.z) > self.angular_thresh else 0.0

        # ========== 转向模式下仅处理角速度，屏蔽线速度指令 ==========
        if self.in_turn_mode:
            # 转向模式中，只有角速度为0时才退出转向模式，否则忽略所有线速度指令
            if target_omega != 0.0:
                # 继续处理转向指令，完全忽略线速度
                turn_cmd = self.ANGULAR_CMD_MAP['turn_left'] if target_omega > 0 else self.ANGULAR_CMD_MAP['turn_right']
                rc_velocity = abs(target_omega) * 500 / (math.pi / 2)
                
                # 仅速度变化超过阈值时，才调整速度
                if abs(rc_velocity - self.last_angular_speed_mm) >= self.speed_change_thresh:
                    speed_cmd = self.generate_speed_cmd(rc_velocity)
                    self.publish_cmd(speed_cmd, "转向-调整速度")
                    self.last_angular_speed_mm = rc_velocity  # 更新缓存
                
                self.current_motion_cmd = turn_cmd
                self.get_logger().debug(f"转向模式：目标角速度{target_omega:.2f} rad/s → {turn_cmd}（忽略线速度指令）")
                return  # 直接返回，不处理后续平移/停止逻辑
            else:
                # 角速度为0，退出转向模式，继续处理平移/停止
                self.get_logger().debug("转向模式收到角速度0，准备退出转向模式")
        
        # 2. 优先级：旋转 > 移动 > 停止
        if target_omega != 0.0:
            # 旋转指令
            turn_cmd = self.ANGULAR_CMD_MAP['turn_left'] if target_omega > 0 else self.ANGULAR_CMD_MAP['turn_right']
            rc_velocity = abs(target_omega) * 500 / (math.pi / 2)
            
            # 转向模式处理
            if not self.in_turn_mode:
                self.enter_turn_mode(turn_cmd, target_omega)
            else:
                # 仅速度变化超过阈值时，才调整速度
                if abs(rc_velocity - self.last_angular_speed_mm) >= self.speed_change_thresh:
                    speed_cmd = self.generate_speed_cmd(rc_velocity)
                    self.publish_cmd(speed_cmd, "转向-调整速度")
                    self.last_angular_speed_mm = rc_velocity  # 更新缓存
                
                self.current_motion_cmd = turn_cmd
                self.get_logger().debug(f"转向指令：目标角速度{target_omega:.2f} rad/s → {turn_cmd}")
        elif target_vx != 0.0 or target_vy != 0.0:
            # 移动指令
            dir_cmd = 'A' # 默认前进
            if target_vx > 0 and target_vy == 0:
                dir_cmd = self.LINEAR_CMD_MAP['forward']
            elif target_vx < 0 and target_vy == 0:
                dir_cmd = self.LINEAR_CMD_MAP['backward']
            elif target_vx == 0 and target_vy > 0:
                dir_cmd = self.LINEAR_CMD_MAP['left']
            elif target_vx == 0 and target_vy < 0:
                dir_cmd = self.LINEAR_CMD_MAP['right']
            elif target_vx > 0 and target_vy > 0:
                dir_cmd = self.LINEAR_CMD_MAP['forward_left']
            elif target_vx > 0 and target_vy < 0:
                dir_cmd = self.LINEAR_CMD_MAP['forward_right']
            elif target_vx < 0 and target_vy > 0:
                dir_cmd = self.LINEAR_CMD_MAP['backward_left']
            elif target_vx < 0 and target_vy < 0:
                dir_cmd = self.LINEAR_CMD_MAP['backward_right']
            
            # 计算目标线速度（mm/s）
            target_speed_m = math.hypot(target_vx, target_vy)
            target_speed_mm = min(self.MAX_SPEED_MM, max(self.MIN_SPEED_MM, target_speed_m * 1000))
            
            # 移动模式处理
            if self.in_turn_mode:
                self.exit_turn_mode(dir_cmd, target_speed_mm)
            else:
                # 仅速度变化超过阈值时，才下发速度指令
                if abs(target_speed_mm - self.last_linear_speed_mm) >= self.speed_change_thresh:
                    speed_cmd = self.generate_speed_cmd(target_speed_mm)
                    self.publish_cmd(speed_cmd, "移动模式-更新速度")
                    self.last_linear_speed_mm = target_speed_mm  # 更新缓存
                
                self.current_motion_cmd = dir_cmd
                self.get_logger().debug(f"移动指令：vx={target_vx:.2f} vy={target_vy:.2f} → {dir_cmd} | 目标速度：{target_speed_mm} mm/s")
        else:
            # 停止时先设速度0，再发停车字符Z
            self.current_motion_cmd = ''
            # 1. 仅当上次速度非0时，下发速度归零指令
            if self.last_linear_speed_mm != 0 or self.last_angular_speed_mm != 0:
                zero_speed_cmd = self.generate_speed_cmd(0)
                self.publish_cmd(zero_speed_cmd, "停止-设置速度0")
                self.last_linear_speed_mm = 0  # 重置缓存
                self.last_angular_speed_mm = 0
            # 2. 下发停车字符Z（触发小车执行速度0的动作）
            self.publish_cmd(self.STOP_CHAR, "停止-下发停车字符")
            # 3. 退出转向模式（如果在转向模式）
            if self.in_turn_mode:
                self.publish_cmd('I', "退出转向模式")
                self.in_turn_mode = False

    def check_timeout(self):
        """超时逻辑：先设速度0，再发停车字符"""
        if time.time() - self.last_cmd_time > self.timeout_threshold:
            self.get_logger().info(f"指令超时（>{self.timeout_threshold}s），先设速度0再停止")
            self.current_motion_cmd = ''
            # 1. 仅当上次速度非0时，下发速度归零指令
            if self.last_linear_speed_mm != 0 or self.last_angular_speed_mm != 0:
                zero_speed_cmd = self.generate_speed_cmd(0)
                self.publish_cmd(zero_speed_cmd, "超时-设置速度0")
                self.last_linear_speed_mm = 0  # 重置缓存
                self.last_angular_speed_mm = 0
            # 2. 下发停车字符Z
            self.publish_cmd(self.STOP_CHAR, "超时-下发停车字符")
            # 3. 退出转向模式
            if self.in_turn_mode:
                self.publish_cmd('I', "超时-退出转向模式")
                self.in_turn_mode = False

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ChassisController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            try:
                # 中断时：先设速度0，再发停车字符
                node.get_logger().info("收到中断信号，先设速度0再停止")  
                # 1. 速度归零
                if node.last_linear_speed_mm != 0 or node.last_angular_speed_mm != 0:
                    zero_speed_cmd = node.generate_speed_cmd(0)
                    node.publish_cmd(zero_speed_cmd, "手动中断-设置速度0")
                    node.last_linear_speed_mm = 0
                    node.last_angular_speed_mm = 0
                # 2. 下发停车字符Z
                node.publish_cmd(node.STOP_CHAR, "手动中断-下发停车字符")
            except Exception as e:
                node.get_logger().warning(f"中断时发送指令失败：{e}")
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"节点运行异常：{e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
