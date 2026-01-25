#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # 新增：增强QoS配置
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import Imu
import tf2_ros
import math
import numpy as np

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        # 1. 订阅轮速
        # 增强QoS配置，提升消息传输可靠性
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.wheel_sub = self.create_subscription(
            Float32MultiArray, 'wheel_speed', self.wheel_callback, qos_sensor
        )
        # 2. 订阅IMU（零漂校准定时器）
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos_sensor
        )
        # 3. 发布器
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_sensor)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ========== 核心参数 ==========
        self.wheel_radius = 0.0375    # 轮子半径
        self.wheel_base = 0.1719       # 前后轮中心间距
        self.wheel_track = 0.186       # 左右轮中心间距
        self.mecanum_const = (self.wheel_base + self.wheel_track) / 2  
        # =====================================================

        # X/Y轴独立校准系数
        self.speed_calib_coeff_base = 2.0  
        self.vx_calib_coeff = 1.048        
        self.vy_calib_coeff = 1.063        

        # ========== IMU核心优化参数 ==========
        # 1. MU定义：前端=y正，左=x正，下方=z正 → 小车偏航角速度对应IMU的y轴
        self.imu_angular_axis = "y"  # 替换之前的z轴，根据实际测试可改为x/y/z
        self.imu_angular_sign = 1.0  # 符号修正，旋转方向反则改为-1.0

        # 2. 零漂校准参数
        self.imu_calib_duration = 2.0  # 静态校准时长（秒）
        self.imu_calib_start_time = None
        self.imu_calib_samples = []    # 校准样本
        self.imu_bias = 0.0            # 校准后的零漂值
        self.imu_calib_done = False    # 校准完成标志

        # 3. 强化滤波参数
        self.imu_lowpass_alpha = 0.1   # 一阶低通滤波系数（越小滤波越强）
        self.imu_sliding_window = 10   # 滑动平均窗口（5→10，增强降噪）
        self.imu_omega_buffer = np.zeros(self.imu_sliding_window)
        self.imu_buffer_idx = 0

        # 4. 积分优化参数
        self.last_imu_omega = 0.0      # 上一帧IMU角速度（梯形积分用）
        self.max_angular_vel = 5.0     # 角速度限幅（防止异常值）
        # =====================================

        # 轮速滑动平均
        self.wheel_speed_buffer = np.zeros((4, 5))  
        self.buffer_idx = 0

        # 里程计初始值
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        self.last_vx = 0.0
        self.last_vy = 0.0

        # 旋转检测阈值（rad/s）
        self.rot_detect_threshold = 0.05  

        # 初始化IMU滤波后角速度，避免偶发属性未定义
        self.imu_omega_filtered = 0.0

    def imu_callback(self, msg):
        """IMU数据处理：轴映射+零漂校准+限幅+低通+滑动平均"""
        # 步骤1：轴映射（核心修正！根据你的IMU坐标系提取正确角速度）
        if self.imu_angular_axis == "x":
            raw_omega = msg.angular_velocity.x
        elif self.imu_angular_axis == "y":
            raw_omega = msg.angular_velocity.y
        else:  # z
            raw_omega = msg.angular_velocity.z
        raw_omega = raw_omega * self.imu_angular_sign

        # 步骤2：静态零漂校准（启动后静止2秒自动校准）
        if not self.imu_calib_done:
            if self.imu_calib_start_time is None:
                self.imu_calib_start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(f"开始IMU零漂校准，请保持小车静止{self.imu_calib_duration}秒！")
            else:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.imu_calib_start_time < self.imu_calib_duration:
                    # 收集校准样本（仅静态值）
                    if abs(raw_omega) < 0.1:  # 过滤动态样本
                        self.imu_calib_samples.append(raw_omega)
                else:
                    # 计算零漂值（中位数更抗噪）
                    if len(self.imu_calib_samples) == 0:
                        self.get_logger().warning("IMU校准样本为空！使用默认零漂值0.0")
                        self.imu_bias = 0.0
                    else:
                        self.imu_bias = np.median(self.imu_calib_samples)
                    self.imu_calib_done = True
                    self.get_logger().info(f"IMU零漂校准完成！零漂值：{self.imu_bias:.6f} rad/s")
            # 校准期间也初始化滤波值，避免后续计算异常
            self.imu_omega_filtered = 0.0
            self.last_imu_omega = 0.0
            return  # 校准期间不处理IMU数据

        # 步骤3：零漂补偿（减去校准后的偏置）
        omega_compensated = raw_omega - self.imu_bias

        # 步骤4：角速度限幅（防止异常值）
        omega_clamped = np.clip(omega_compensated, -self.max_angular_vel, self.max_angular_vel)

        # 步骤5：一阶低通滤波（降噪）
        omega_filtered = self.imu_lowpass_alpha * omega_clamped + (1 - self.imu_lowpass_alpha) * self.last_imu_omega

        # 步骤6：滑动平均滤波（进一步降噪）
        self.imu_omega_buffer[self.imu_buffer_idx] = omega_filtered
        self.imu_buffer_idx = (self.imu_buffer_idx + 1) % self.imu_sliding_window
        
        # 优化：过滤全零缓冲区，解决偶发空切片警告
        valid_data = self.imu_omega_buffer[self.imu_omega_buffer != 0]
        if len(valid_data) > 0:
            self.imu_omega_filtered = np.mean(valid_data)
        else:
            self.imu_omega_filtered = 0.0

        # 保存上一帧数据（梯形积分用）
        self.last_imu_omega = self.imu_omega_filtered

    def wheel_callback(self, msg):
        """核心逻辑：旋转用优化后的IMU积分，直行保留轮速"""
        wheel_speeds_raw = msg.data

        # 步骤1：零漂过滤
        threshold = 0.008
        filtered_speeds = [0.0 if abs(s) < threshold else s for s in wheel_speeds_raw]

        # 步骤2：轮速滑动平均
        self.wheel_speed_buffer[:, self.buffer_idx] = filtered_speeds
        self.buffer_idx = (self.buffer_idx + 1) % 5
        
        # 优化：轮速缓冲区判空，解决偶发空切片警告
        valid_wheel_data = self.wheel_speed_buffer[self.wheel_speed_buffer != 0]
        if len(valid_wheel_data) >= 4:
            smooth_speeds = np.mean(self.wheel_speed_buffer, axis=1).tolist()
        else:
            smooth_speeds = [0.0, 0.0, 0.0, 0.0]
            
        v_rl, v_fl, v_fr, v_rr = smooth_speeds

        # 步骤3：时间差计算
        current_time = self.get_clock().now()
        if self.last_time is None or not self.imu_calib_done:
            self.last_time = current_time
            self.last_vx = 0.0
            self.last_vy = 0.0
            
            # 即使未校准，也发布初始odom，解决偶发话题丢失
            self._publish_odom_and_tf(current_time, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        dt = np.clip(dt, 0.001, 0.05)  # 时间差限幅

        # 步骤4：麦轮逆解+X/Y校准
        vx_raw = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy_raw = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        wheel_omega = (-v_fl + v_fr - v_rl + v_rr) / (2 * self.mecanum_const)

        vx_calib = vx_raw * self.speed_calib_coeff_base * self.vx_calib_coeff
        vy_calib = vy_raw * self.speed_calib_coeff_base * self.vy_calib_coeff

        # 步骤5：旋转角速度计算（梯形积分）
        is_rotating = abs(self.imu_omega_filtered) > self.rot_detect_threshold
        if is_rotating and self.imu_calib_done:
            # 梯形积分（比欧拉积分更精准，减少累积误差）
            omega_avg = (self.imu_omega_filtered + self.last_imu_omega) / 2.0
            delta_theta = omega_avg * dt
            self.theta += delta_theta
            fused_omega = self.imu_omega_filtered
        else:
            # 直行时用轮速角速度
            fused_omega = wheel_omega
            delta_theta = fused_omega * dt
            self.theta += delta_theta

        # 偏航角归一化（-π~π）
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 步骤6：位置更新
        vx_avg = (vx_calib + self.last_vx) / 2.0
        vy_avg = (vy_calib + self.last_vy) / 2.0
        self.x += (vx_avg * math.cos(self.theta) - vy_avg * math.sin(self.theta)) * dt
        self.y += (vx_avg * math.sin(self.theta) + vy_avg * math.cos(self.theta)) * dt

        self.last_vx = vx_calib
        self.last_vy = vy_calib

        # 步骤7：发布里程计+TF
        self._publish_odom_and_tf(
            current_time, self.x, self.y, self.theta,
            vx_calib, vy_calib, fused_omega
        )

        # 调试日志（IMU校准信息）
        self.get_logger().debug(
            f"IMU校准：{self.imu_calib_done} | 零漂：{self.imu_bias:.6f} | "
            f"旋转：{is_rotating} | 滤波后角速度：{self.imu_omega_filtered:.4f} | "
            f"偏航角：{self.theta:.3f} rad ({self.theta*180/math.pi:.1f}°)"
        )

    def _publish_odom_and_tf(self, current_time, x, y, theta, vx, vy, omega):
        """优化：封装发布逻辑，确保任何场景下都能稳定发布odom和TF"""
        # 发布里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_quat = Quaternion()
        odom_quat.x = 0.0
        odom_quat.y = 0.0
        odom_quat.z = math.sin(theta / 2)
        odom_quat.w = math.cos(theta / 2)
        odom_msg.pose.pose.orientation = odom_quat
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        self.odom_pub.publish(odom_msg)

        # TF变换
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom_quat
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    # 捕获键盘中断，优雅终止节点
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("里程计节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# 若修仍有微小误差，可按以下公式微调独立系数：
# X 轴修正系数 = 1.0 / 实际前进 1m 的 odom X 值（例：odom X=0.99m → 修正系数 = 1/0.99≈1.01）
# Y 轴修正系数 = 1.0 / 实际左移 1m 的 odom Y 值（例：odom Y=0.98m → 修正系数 = 1/0.98≈1.02
# imu若仍有微小误差，微调imu_lowpass_alpha（0.05~0.2），越小滤波越强，旋转越平滑。