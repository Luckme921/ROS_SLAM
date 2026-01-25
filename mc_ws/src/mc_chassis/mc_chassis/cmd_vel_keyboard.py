#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import tty
import termios
import select
import signal
import math

class CmdVelKeyboard(Node):
    def __init__(self):
        super().__init__('cmd_vel_keyboard')
        # QoS配置：DurabilityPolicy正确引用VOLATILE
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 可靠交付
            history=HistoryPolicy.KEEP_LAST,        # 保留最后N条
            depth=10,                                # 缓存10条
            durability=DurabilityPolicy.VOLATILE     # 临时数据
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        
        # 终端配置
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        self.should_exit = False
        signal.signal(signal.SIGINT, self.signal_handler)

        # 速度配置
        self.current_linear_speed = 0.2  # 200mm/s
        self.linear_step = 0.01           # 10mm/s步长
        self.max_linear = 1.0             # 1000mm/s上限
        self.min_linear = 0.1             # 100mm/s下限

        # 当前速度状态
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # 当前动作
        self.current_action = None

        # 按键映射
        self.KEY_ACTION_MAP = {
            'w': ('forward',),    's': ('backward',),   'd': ('right',),      'a': ('left',),
            'e': ('forward_right',), 'f': ('forward_left',), 'g': ('backward_right',), 'h': ('backward_left',),
            'z': ('turn_left',),  'c': ('turn_right',), 'x': ('speed_up',),   'y': ('speed_down',),
            '0': ('stop',),       'q': ('exit',),
        }

        # 10Hz定时器（0.1秒周期）
        self.pub_freq = 10
        self.timer_period = 1.0 / self.pub_freq
        self.timer = self.create_timer(self.timer_period, self.timer_publish_cmd_vel)
        
        self.print_help()

    def print_help(self):
        """打印按键提示"""
        self.get_logger().info("="*70)
        self.get_logger().info("麦轮小车键盘控制（10Hz低资源版 | Q键退出立即停止）")
        self.get_logger().info(f"初始线速度：{self.current_linear_speed*1000}mm/s")
        self.get_logger().info(f"cmd_vel发布频率：{self.pub_freq}Hz")
        self.get_logger().info("【操作】W前 S后 A左 D右 | Z左转 C右转 | X加速 Y减速")
        self.get_logger().info("【停止】0键临时停止 | Q键退出（立即发布停止指令）")
        self.get_logger().info("="*70)

    def calc_angular_speed(self):
        """计算旋转角速度"""
        rc_velocity = self.current_linear_speed * 1000
        return (math.pi / 2) * (rc_velocity / 500)

    def signal_handler(self, signum, frame):
        """Ctrl+C中断处理：立即停止"""
        self.should_exit = True
        self.send_stop_cmd(immediate_publish=True)
        self.get_logger().info("\n收到Ctrl+C，立即发送停止指令...")

    def get_key(self):
        """非阻塞读取按键"""
        try:
            tty.setraw(self.fd)
            rlist, _, _ = select.select([sys.stdin], [], [], self.timer_period/2)
            key = sys.stdin.read(1).lower() if rlist else ''
            return key
        except Exception as e:
            self.get_logger().error(f"读取按键失败：{e}")
            return ''

    def send_stop_cmd(self, immediate_publish=False):
        """发送停止指令（支持立即发布）"""
        # 重置速度状态
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        self.current_action = None
        
        # 立即发布停止消息
        if immediate_publish:
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.get_logger().info("✅ 立即发布停止指令：cmd_vel所有速度置0")
        
        self.get_logger().info("停止：所有速度置0，10Hz发布零速度指令")

    def timer_publish_cmd_vel(self):
        """10Hz定时器回调：持续发布当前速度"""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.linear.y = self.current_linear_y
        twist.angular.z = self.current_angular_z
        self.cmd_vel_pub.publish(twist)

    def update_current_vel(self, linear_x, linear_y, angular_z):
        """更新速度状态"""
        self.current_linear_x = linear_x
        self.current_linear_y = linear_y
        self.current_angular_z = angular_z
        self.get_logger().info(
            f"更新状态 → x:{linear_x:.2f}, y:{linear_y:.2f}, z:{angular_z:.2f} | 线速度：{self.current_linear_speed*1000}mm/s"
        )

    def refresh_current_action(self):
        """加减速后刷新当前动作速度"""
        if not self.current_action:
            return
        
        linear_x, linear_y, angular_z = 0.0, 0.0, 0.0
        if self.current_action == 'forward':
            linear_x = self.current_linear_speed
        elif self.current_action == 'backward':
            linear_x = -self.current_linear_speed
        elif self.current_action == 'left':
            linear_y = self.current_linear_speed
        elif self.current_action == 'right':
            linear_y = -self.current_linear_speed
        elif self.current_action == 'forward_right':
            linear_x = self.current_linear_speed * math.cos(math.pi/4)
            linear_y = -self.current_linear_speed * math.sin(math.pi/4)
        elif self.current_action == 'forward_left':
            linear_x = self.current_linear_speed * math.cos(math.pi/4)
            linear_y = self.current_linear_speed * math.sin(math.pi/4)
        elif self.current_action == 'backward_right':
            linear_x = -self.current_linear_speed * math.cos(math.pi/4)
            linear_y = -self.current_linear_speed * math.sin(math.pi/4)
        elif self.current_action == 'backward_left':
            linear_x = -self.current_linear_speed * math.cos(math.pi/4)
            linear_y = self.current_linear_speed * math.sin(math.pi/4)
        elif self.current_action == 'turn_left':
            angular_z = self.calc_angular_speed()
        elif self.current_action == 'turn_right':
            angular_z = -self.calc_angular_speed()
        
        self.update_current_vel(linear_x, linear_y, angular_z)

    def handle_key(self, key):
        """按键处理"""
        if key not in self.KEY_ACTION_MAP:
            return
        
        action = self.KEY_ACTION_MAP[key][0]

        if action == 'exit':
            # Q键退出：先立即停止，再标记退出
            self.send_stop_cmd(immediate_publish=True)
            self.should_exit = True
            self.get_logger().info("⚠️ 按下Q键，发布停止指令并准备退出...")
        elif action == 'stop':
            # 0键停止：立即生效
            self.send_stop_cmd(immediate_publish=True)
        elif action == 'speed_up':
            self.current_linear_speed = min(self.current_linear_speed + self.linear_step, self.max_linear)
            self.get_logger().info(f"加速 → 线速度：{self.current_linear_speed*1000}mm/s")
            self.refresh_current_action()
        elif action == 'speed_down':
            self.current_linear_speed = max(self.current_linear_speed - self.linear_step, self.min_linear)
            self.get_logger().info(f"减速 → 线速度：{self.current_linear_speed*1000}mm/s")
            self.refresh_current_action()
        # 平移控制
        elif action in ['forward', 'backward', 'left', 'right', 
                        'forward_right', 'forward_left', 'backward_right', 'backward_left']:
            self.current_action = action
            self.refresh_current_action()
        # 旋转控制
        elif action in ['turn_left', 'turn_right']:
            self.current_action = action
            self.refresh_current_action()

    def run(self):
        """主循环"""
        try:
            while not self.should_exit and rclpy.ok():
                key = self.get_key()
                if key:
                    self.handle_key(key)
                rclpy.spin_once(self, timeout_sec=self.timer_period/10)
        finally:
            # 退出循环后，再次确保停止
            self.send_stop_cmd(immediate_publish=True)
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except Exception as e:
                self.get_logger().error(f"恢复终端失败：{e}")
            self.destroy_timer(self.timer)
            self.get_logger().info("✅ 终端已恢复，小车已停止，节点退出完成！")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelKeyboard()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"运行出错：{e}")
        node.send_stop_cmd(immediate_publish=True)
        try:
            termios.tcsetattr(node.fd, termios.TCSADRAIN, node.old_settings)
        except:
            pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()