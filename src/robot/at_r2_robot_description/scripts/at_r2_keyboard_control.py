#!/usr/bin/env python3
"""
AT_R2机器人键盘控制节点
控制移动和升降臂
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

msg = """
========================================
AT_R2 机器人键盘控制
========================================
移动控制:
        W - 前进
   A    S    D
  左转  后退  右转
  
  空格 - 停止
  
升降臂控制:
  1 - 收起所有升降臂
  2 - 伸出后轮 (-4cm)
  3 - 伸出后轮 (-8cm)  
  4 - 伸出后轮 (-12cm)
  5 - 伸出辅助轮 (-4cm)
  6 - 伸出辅助轮 (-8cm)
  7 - 伸出辅助轮 (-12cm)
  8 - 全部伸出 (-12cm)
  
速度调节:
  Q - 增加速度 (+10%)
  Z - 减少速度 (-10%)
  
  Ctrl+C 退出
========================================
"""

class AT_R2_KeyboardControl(Node):
    def __init__(self):
        super().__init__('at_r2_keyboard_control')
        
        # 速度控制发布器
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        # 升降关节发布器
        self.rear_left_pub = self.create_publisher(
            Float64, '/model/AT_R2/joint/rear_left_lift_joint/cmd_pos', 10)
        self.rear_right_pub = self.create_publisher(
            Float64, '/model/AT_R2/joint/rear_right_lift_joint/cmd_pos', 10)
        self.lift_left_pub = self.create_publisher(
            Float64, '/model/AT_R2/joint/lift_left_lift_joint/cmd_pos', 10)
        self.lift_right_pub = self.create_publisher(
            Float64, '/model/AT_R2/joint/lift_right_lift_joint/cmd_pos', 10)
        
        # 速度参数
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # 当前速度
        self.vel_x = 0.0
        self.vel_z = 0.0
        
        self.get_logger().info('AT_R2 键盘控制启动')
        self.get_logger().info(f'线速度: {self.linear_speed} m/s')
        self.get_logger().info(f'角速度: {self.angular_speed} rad/s')
    
    def get_key(self, settings):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def publish_vel(self):
        """发布速度命令"""
        msg = Twist()
        msg.linear.x = self.vel_x
        msg.angular.z = self.vel_z
        self.cmd_vel_pub.publish(msg)
    
    def set_rear_wheels(self, position):
        """设置后轮位置"""
        msg = Float64()
        msg.data = position
        self.rear_left_pub.publish(msg)
        self.rear_right_pub.publish(msg)
        self.get_logger().info(f'后轮位置: {position:.2f}m')
    
    def set_lift_wheels(self, position):
        """设置辅助轮位置"""
        msg = Float64()
        msg.data = position
        self.lift_left_pub.publish(msg)
        self.lift_right_pub.publish(msg)
        self.get_logger().info(f'辅助轮位置: {position:.2f}m')
    
    def run(self):
        """主循环"""
        settings = termios.tcgetattr(sys.stdin)
        print(msg)
        
        try:
            while True:
                key = self.get_key(settings)
                
                # 移动控制
                if key == 'w':
                    self.vel_x = self.linear_speed
                    self.vel_z = 0.0
                elif key == 's':
                    self.vel_x = -self.linear_speed
                    self.vel_z = 0.0
                elif key == 'a':
                    self.vel_x = 0.0
                    self.vel_z = self.angular_speed
                elif key == 'd':
                    self.vel_x = 0.0
                    self.vel_z = -self.angular_speed
                elif key == ' ':
                    self.vel_x = 0.0
                    self.vel_z = 0.0
                
                # 速度调节
                elif key == 'q':
                    self.linear_speed *= 1.1
                    self.angular_speed *= 1.1
                    self.get_logger().info(
                        f'速度增加: 线速度={self.linear_speed:.2f}, 角速度={self.angular_speed:.2f}')
                elif key == 'z':
                    self.linear_speed *= 0.9
                    self.angular_speed *= 0.9
                    self.get_logger().info(
                        f'速度减少: 线速度={self.linear_speed:.2f}, 角速度={self.angular_speed:.2f}')
                
                # 升降臂控制
                elif key == '1':  # 全部收起
                    self.set_rear_wheels(0.0)
                    self.set_lift_wheels(0.0)
                elif key == '2':  # 后轮-4cm
                    self.set_rear_wheels(-0.04)
                elif key == '3':  # 后轮-8cm
                    self.set_rear_wheels(-0.08)
                elif key == '4':  # 后轮-12cm
                    self.set_rear_wheels(-0.12)
                elif key == '5':  # 辅助轮-4cm
                    self.set_lift_wheels(-0.04)
                elif key == '6':  # 辅助轮-8cm
                    self.set_lift_wheels(-0.08)
                elif key == '7':  # 辅助轮-12cm
                    self.set_lift_wheels(-0.12)
                elif key == '8':  # 全部伸出
                    self.set_rear_wheels(-0.12)
                    self.set_lift_wheels(-0.12)
                
                # 退出
                elif key == '\x03':  # Ctrl+C
                    break
                
                # 发布速度
                self.publish_vel()
                
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            # 停止机器人
            self.vel_x = 0.0
            self.vel_z = 0.0
            self.publish_vel()

def main():
    rclpy.init()
    node = AT_R2_KeyboardControl()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

