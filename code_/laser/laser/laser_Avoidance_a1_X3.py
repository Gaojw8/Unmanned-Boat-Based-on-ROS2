# 导入 ROS 和相关库
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# 导入常用的 Python 库
import math
import numpy as np
import time
from time import sleep

# 导入自定义的通用函数或类
from yahboomcar_laser.common import *

# 打印导入完成的消息
print("Import done")

# 弧度到角度的转换比例常量
RAD2DEG = 180 / math.pi


# 定义一个 ROS 节点类，用于避障功能
class laserAvoid(Node):
    def __init__(self, name):
        super().__init__(name)

        # 创建激光雷达数据的订阅器
        self.sub_laser = self.create_subscription(LaserScan, "/scan", self.registerScan, 1)

        # 创建遥控器状态的订阅器
        self.sub_JoyState = self.create_subscription(Bool, '/JoyState', self.JoyStateCallback, 1)

        # 创建控制小车运动的发布器
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        # 声明和获取节点参数
        self.declare_parameter("linear", 0.5)  # 声明直线运动速度参数，默认值为 0.5
        self.linear = self.get_parameter('linear').get_parameter_value().double_value  # 获取直线运动速度参数的值

        self.declare_parameter("angular", 1.0)  # 声明角速度参数，默认值为 1.0
        self.angular = self.get_parameter('angular').get_parameter_value().double_value  # 获取角速度参数的值

        self.declare_parameter("LaserAngle", 40.0)  # 声明激光雷达扫描角度范围参数，默认值为 40.0
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value  # 获取激光雷达扫描角度范围参数的值

        self.declare_parameter("ResponseDist", 0.80)  # 声明障碍物响应距离参数，默认值为 0.55
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value  # 获取障碍物响应距离参数的值

        self.declare_parameter("Switch", False)  # 声明开关参数，默认值为 False
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value  # 获取开关参数的布尔值

        # 初始化避障检测相关变量
        self.Right_warning = 0  # 右侧警告计数器，初始化为 0
        self.Left_warning = 0  # 左侧警告计数器，初始化为 0
        self.front_warning = 0  # 前方警告计数器，初始化为 0
        self.Joy_active = False  # 手柄激活状态，初始化为 False

        # 创建一个 PID 控制器实例
        self.ros_ctrl = SinglePID()

        # 创建一个定时器，每 0.01 秒调用一次 on_timer 方法
        self.timer = self.create_timer(0.01, self.on_timer)

    # 定时器回调函数，更新节点参数
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value

    # 遥控器状态回调函数，更新 Joy_active 状态
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data

    # 激光雷达数据回调函数，进行障碍物检测并发布速度控制指令
    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            return

        # 将激光雷达数据转换为 numpy 数组
        ranges = np.array(scan_data.ranges)

        # 初始化障碍物警告计数
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0

        # 遍历激光雷达数据，检测前方和侧方的障碍物
        for i in range(len(ranges)):
            # 计算当前激光束的角度（将弧度转换为度数）
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG

            # 检测右侧障碍物
            if 160 > angle > 180 - self.LaserAngle:
                # 如果距离小于 ResponseDist 的1.5倍，则认为检测到右侧障碍物
                if ranges[i] < self.ResponseDist * 1.5:
                    self.Right_warning += 1

            # 检测左侧障碍物
            if -160 < angle < self.LaserAngle - 180:
                # 如果距离小于 ResponseDist 的1.5倍，则认为检测到左侧障碍物
                if ranges[i] < self.ResponseDist * 1.5:
                    self.Left_warning += 1

            # 检测前方障碍物
            if abs(angle) > 160:
                # 如果距离小于等于 ResponseDist 的1.5倍，则认为检测到前方障碍物
                if ranges[i] <= self.ResponseDist * 1.5:
                    self.front_warning += 1

        # 如果遥控器活动或开关打开，则不执行避障操作
        if self.Joy_active or self.Switch == True:
            if self.Moving == True:
                self.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return

        self.Moving = True
        twist = Twist()

        # 根据障碍物位置和数量发布速度控制指令
        if self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print('1, there are obstacles in the left and right, turn right')

            # 设置线速度和角速度控制指令
            twist.linear.x = -1.0  # 设定线速度为预先声明的线速度参数值
            twist.angular.z = 0.0  # 设定角速度为负的预先声明的角速度参数值（向右转）

            # 发布速度控制指令到/cmd_vel主题
            self.pub_vel.publish(twist)

            # 等待一段时间确保机器人按照指令运动
            sleep(0.2)


        elif self.front_warning > 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            print('2, there is an obstacle in the middle right, turn left')
            # 停止前进并向左转
            twist.linear.x = 0.0  # 停止线速度
            twist.angular.z = self.angular  # 设定角速度为预先声明的角速度参数值（向左转）
            self.pub_vel.publish(twist)  # 发布速度控制指令
            sleep(0.2)  # 等待一段时间确保机器人按照指令运动
            # 进一步检查左侧和右侧障碍物状态，如果需要，向右转
            if self.Left_warning > 10 and self.Right_warning <= 10:
                twist.linear.x = 0.0  # 停止线速度
                twist.angular.z = -self.angular  # 设定角速度为负的预先声明的角速度参数值（向右转）
                self.pub_vel.publish(twist)  # 发布速度控制指令
                sleep(0.5)  # 等待一段时间确保机器人按照指令运动


        elif self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            print('4. There is an obstacle in the middle left, turn right')
            # 停止前进并向右转
            twist.linear.x = 0.0  # 停止线速度
            twist.angular.z = -self.angular  # 设定角速度为负的预先声明的角速度参数值（向右转）
            self.pub_vel.publish(twist)  # 发布速度控制指令
            sleep(0.2)  # 等待一段时间确保机器人按照指令运动
            # 进一步检查左侧和右侧障碍物状态，如果需要，向左转
            if self.Left_warning <= 10 and self.Right_warning > 10:
                twist.linear.x = 0.0  # 停止线速度
                twist.angular.z = self.angular  # 设定角速度为预先声明的角速度参数值（向左转）
                self.pub_vel.publish(twist)  # 发布速度控制指令
                sleep(0.5)  # 等待一段时间确保机器人按照指令运动

        elif self.front_warning > 10 and self.Left_warning < 10 and self.Right_warning < 10:
            print('6, there is an obstacle in the middle, turn left')
            # 停止前进并向左转
            twist.linear.x = 0.0  # 停止线速度
            twist.angular.z = self.angular  # 设定角速度为预先声明的角速度参数值（向左转）
            self.pub_vel.publish(twist)  # 发布速度控制指令
            sleep(0.2)  # 等待一段时间确保机器人按照指令运动

        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print('7. There are obstacles on the left and right, turn right')
            # 停止前进并向右转
            twist.linear.x = 0.5  # 停止线速度
            twist.angular.z = 0.0  # 设定角速度为负的预先声明的角速度参数值（向右转）
            self.pub_vel.publish(twist)  # 发布速度控制指令
            sleep(0.4)  # 等待一段时间确保机器人按照指令运动

        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            print('8, there is an obstacle on the left, turn right')
            # 停止前进并向右转
            twist.linear.x = 0.0  # 停止线速度
            twist.angular.z = -self.angular  # 设定角速度为负的预先声明的角速度参数值（向右转）
            self.pub_vel.publish(twist)  # 发布速度控制指令
            sleep(0.2)  # 等待一段时间确保机器人按照指令运动

        elif self.front_warning < 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            print('9, there is an obstacle on the right, turn left')
            # 停止前进并向左转
            twist.linear.x = 0.0  # 停止线速度
            twist.angular.z = self.angular  # 设定角速度为预先声明的角速度参数值（向左转）
            self.pub_vel.publish(twist)  # 发布速度控制指令
            sleep(0.2)  # 等待一段时间确保机器人按照指令运动

        elif self.front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
            print('10, no obstacles, go forward')
            # 前方无障碍物，直行
            twist.linear.x = self.linear  # 设定线速度为预先声明的线速度参数值
            twist.angular.z = 0.0  # 停止转弯
            self.pub_vel.publish(twist)  # 发布速度控制指令


# 主函数，初始化 ROS 节点并运行避障节点
def main():
    rclpy.init()
    laser_avoid = laserAvoid("laser_Avoidance_c1")
    print("Start it")
    rclpy.spin(laser_avoid)
