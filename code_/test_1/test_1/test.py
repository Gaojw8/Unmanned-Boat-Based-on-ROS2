import pigpio
import time
# 初始化pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio not connected!")

motor1_pwm_pin = 13  # 硬件PWM GPIO12 (BCM编码)
motor2_pwm_pin = 12  # 硬件PWM GPIO13 (BCM编码)

# PWM参数
pwm_center = 1500
pwm_forward = 1700
pwm_reverse = 1300
pwm_stop = pwm_center

# 确保PWM频率设置为50Hz
pi.set_PWM_frequency(motor1_pwm_pin, 50)
pi.set_PWM_frequency(motor2_pwm_pin, 50)
pi.set_servo_pulsewidth(motor1_pwm_pin, 1500)
pi.set_servo_pulsewidth(motor2_pwm_pin, 1500)
time.sleep(3)
def set_pwm(motor1_pwm_value, motor2_pwm_value):
    # 设置电机的PWM值，转化为硬件PWM接口的百分比占空比
    pi.set_servo_pulsewidth(motor1_pwm_pin, motor1_pwm_value)
    pi.set_servo_pulsewidth(motor2_pwm_pin, motor2_pwm_value)
try:
    set_pwm(1200, 1800)
    time.sleep(3)
    set_pwm(1700, 1300)
    time.sleep(1)
    set_pwm(1300, 1300)
    time.sleep(1.5)
    set_pwm(1200, 1800)
    time.sleep(3)
    set_pwm(1700, 1300)
    time.sleep(1)
finally:
    # 确保程序结束时电机停止转动
    set_pwm(pwm_stop, pwm_stop)
    print("Motors set to stop.")
