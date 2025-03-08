import pigpio
import time

def convert_us_to_duty_cycle(us):
    # 将微秒值转换为pigpio库的占空比值
    min_us = 1000
    max_us = 2000
    min_duty_cycle = 500000
    max_duty_cycle = 1000000
    return int((us - min_us) / (max_us - min_us) * (max_duty_cycle - min_duty_cycle) + min_duty_cycle)

def main():
    pi = pigpio.pi()

    if not pi.connected:
        exit()

    user_gpio = 12
    frequency = 50  # 设置频率为50Hz

    initial_us = 1500  # 初始占空比为1500微秒
    initial_duty_cycle = convert_us_to_duty_cycle(initial_us)
    pi.hardware_PWM(user_gpio, frequency, initial_duty_cycle)
    print(f"Initial PWM set to {initial_us} us ({initial_duty_cycle} duty cycle)")

    time.sleep(5)

    new_us = 1600  # 新的占空比为1600微秒
    new_duty_cycle = convert_us_to_duty_cycle(new_us)
    pi.hardware_PWM(user_gpio, frequency, new_duty_cycle)
    print(f"New PWM set to {new_us} us ({new_duty_cycle} duty cycle)")

    time.sleep(5)

    pi.hardware_PWM(user_gpio, 0, 0)  # 停止PWM输出
    print("PWM stopped")

    pi.stop()

if __name__ == '__main__':
    main()