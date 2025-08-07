import RPi.GPIO as GPIO
import time

BUZZER_PIN = 6  # 使用GPIO18（PWM0）

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    global pwm
    pwm = GPIO.PWM(BUZZER_PIN, 1)  # 初始频率1Hz
    pwm.start(50)  # 50%占空比

def beep(freq, duration):
    pwm.ChangeFrequency(freq)
    pwm.ChangeDutyCycle(50)  # 压电蜂鸣器建议50%占空比
    time.sleep(duration/1000)
    pwm.ChangeDutyCycle(0)  # 停止发声

def destroy():
    pwm.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    setup()
    try:
        # 测试不同频率
        beep(2000, 500)  # 2000Hz 0.5秒
        beep(3000, 300)  # 3000Hz 0.3秒
        beep(4000, 200)  # 4000Hz 0.2秒
    except KeyboardInterrupt:
        destroy()