#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import math

class WheelJointStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_state_publisher')

        # 订阅轮速话题
        self.wheel_sub = self.create_subscription(
            Float32MultiArray, 'wheel_speed', self.wheel_callback, 10
        )

        # 发布关节状态话题
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 核心参数（匹配URDF）
        self.wheel_radius = 0.0375
        self.wheel_joint_names = [
            "wheel_lb_jiont",
            "wheel_lf_jiont",
            "wheel_rf_jiont",
            "wheel_rb_jiont"
        ]
        self.speed_threshold = 0.005

        # 初始化
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]
        self.last_time = None
        # 日志频率控制（减少CPU负担）
        self.log_interval = 0.1 
        self.last_log_time = None
        self.get_logger().info("轮子关节状态发布节点已启动！")

    # 角度归一化函数（解决无限积累问题）
    def normalize_angle(self, angle):
        """将角度限制在[-π, π]范围，避免无限累加"""
        angle = math.fmod(angle, 2 * math.pi)
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def wheel_callback(self, msg):
        # 校验数据长度
        if len(msg.data) != 4:
            self.get_logger().error(f"轮速数据长度错误！期望4个值，实际{len(msg.data)}个")
            return

        # 过滤微小速度
        filtered_speeds = [0.0 if abs(s) < self.speed_threshold else s for s in msg.data]

        # 时间差计算
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            self.last_log_time = current_time 
            return
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 过滤异常时间差
        if dt < 0.001 or dt > 0.2:
            self.get_logger().info(f"跳过异常时间差: {dt:.4f}s")  
            return

        # 线速度转角度速度
        wheel_angular_speeds = [-speed / self.wheel_radius for speed in filtered_speeds]

        # 积分计算轮子角度 + 归一化
        for i in range(4):
            self.wheel_angles[i] += wheel_angular_speeds[i] * dt
            self.wheel_angles[i] = self.normalize_angle(self.wheel_angles[i])

        # 发布关节状态
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.header.frame_id = "base_link"
        joint_state_msg.name = self.wheel_joint_names
        joint_state_msg.position = self.wheel_angles
        joint_state_msg.velocity = wheel_angular_speeds
        joint_state_msg.effort = []

        self.joint_state_pub.publish(joint_state_msg)

       
        if (current_time - self.last_log_time).nanoseconds / 1e9 >= self.log_interval:
            self.get_logger().info(  
                f"轮子角度：左后{self.wheel_angles[0]:.3f} 左前{self.wheel_angles[1]:.3f} "
                f"右前{self.wheel_angles[2]:.3f} 右后{self.wheel_angles[3]:.3f} (rad)"
            )
            self.last_log_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WheelJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被手动终止")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()