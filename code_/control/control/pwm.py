mport rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import pigpio
import time

class MotorController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_pwm = self.create_publisher(Int32MultiArray, '/motor_pwm', 10)

        # 初始化pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio not connected!")

        self.motor1_pwm_pin = 13  # 硬件PWM GPIO12 (BCM编码)
        self.motor2_pwm_pin = 12  # 硬件PWM GPIO13 (BCM编码)

        # PWM参数
        self.pwm_center = 1500
        self.pwm_forward = 1750
        self.pwm_reverse = 1250
        self.pwm_stop = self.pwm_center

        # 确保PWM频率设置为50Hz
        self.pi.set_PWM_frequency(self.motor1_pwm_pin, 50)
        self.pi.set_PWM_frequency(self.motor2_pwm_pin, 50)


        self.pi.set_servo_pulsewidth(self.motor1_pwm_pin, 1500)
        self.pi.set_servo_pulsewidth(self.motor2_pwm_pin, 1500)
        time.sleep(3)


    def set_pwm(self, motor1_pwm_value, motor2_pwm_value):
        # 设置电机的PWM值，转化为硬件PWM接口的百分比占空比
        self.pi.set_servo_pulsewidth(self.motor1_pwm_pin, motor1_pwm_value)
        self.pi.set_servo_pulsewidth(self.motor2_pwm_pin, motor2_pwm_value)

        # 发布PWM信号
        pwm_msg = Int32MultiArray()
        pwm_msg.data = [motor1_pwm_value, motor2_pwm_value]
        self.pub_pwm.publish(pwm_msg)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if linear == 0.0:
            if angular > 0.0:
                # 左转
                self.set_pwm(self.pwm_reverse, self.pwm_reverse)
            elif angular < 0.0:
                # 右转
                self.set_pwm(self.pwm_forward, self.pwm_forward)
            else:
                # 停止
                self.set_pwm(self.pwm_stop, self.pwm_stop)
        elif linear < 0.0:
                self.set_pwm(self.pwm_reverse, self.pwm_forward)
        else:
            if angular == 0.0:
                # 前进
                self.set_pwm(1650,1350)
            else:
                # 前进时左转或右转，根据角速度实现
                left_pwm = self.pwm_forward + angular * 100  # 简单计算，具体需要根据实际需求调整
                right_pwm = self.pwm_forward - angular * 100
                self.set_pwm(left_pwm, right_pwm)

    def cleanup(self):
        # 在程序停止时将电机PWM设置为1500
        self.set_pwm(self.pwm_stop, self.pwm_stop)
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController("motor_controller")
    print("Motor Controller Node Started")

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.cleanup()
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
