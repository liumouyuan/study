import RPi.GPIO as GPIO
import time

BTN_PIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(BTN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def btn_pressed(channel):
    if GPIO.input(BTN_PIN) == GPIO.LOW:
        print('涓柇鍝嶅簲  中断响应')

GPIO.add_event_detect(BTN_PIN, GPIO.FALLING,
                      callback=btn_pressed,
                      bouncetime=200)

print("ccc")
if __name__ == '__main__':
	try:
		while True:
			time.sleep(5)
			print('涓荤▼搴忚繘琛屼腑  主程序进行中')
	except KeyboardInterrupt:
		print("涓柇  中断")
	finally:
		GPIO.cleanup()